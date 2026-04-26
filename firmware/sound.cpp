#include "sound.h"
#include "config.h"
#include <FS.h>
#include <LittleFS.h>
#include <driver/i2s.h>

struct WavHeaderInfo {
  uint16_t audioFormat;
  uint16_t numChannels;
  uint32_t sampleRate;
  uint16_t bitsPerSample;
  uint32_t dataOffset;
  uint32_t dataSize;
};

static QueueHandle_t g_soundQueue = nullptr;
static TaskHandle_t g_soundTask = nullptr;
static bool g_i2sReady = false;
static int g_soundVolume = SOUND_VOLUME_DEFAULT;

static inline uint16_t rd16(const uint8_t* p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline uint32_t rd32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline int16_t scaleSample(int16_t s) {
  int32_t v = (int32_t)s * (int32_t)g_soundVolume / 100;
  if (v > 32767) v = 32767;
  if (v < -32768) v = -32768;
  return (int16_t)v;
}

static bool parseWavHeader(File &f, WavHeaderInfo &info) {
  uint8_t hdr[12];
  if (f.read(hdr, 12) != 12) return false;
  if (memcmp(hdr, "RIFF", 4) != 0) return false;
  if (memcmp(hdr + 8, "WAVE", 4) != 0) return false;

  bool gotFmt = false, gotData = false;
  uint16_t audioFormat = 0, numChannels = 0, bitsPerSample = 0;
  uint32_t sampleRate = 0;
  uint32_t dataOffset = 0, dataSize = 0;

  while (f.available()) {
    uint8_t ch[8];
    if (f.read(ch, 8) != 8) break;
    uint32_t chunkSize = rd32(ch + 4);

    if (memcmp(ch, "fmt ", 4) == 0) {
      uint8_t fmt[32];
      uint32_t r = chunkSize;
      if (r > sizeof(fmt)) r = sizeof(fmt);
      if (f.read(fmt, r) != (int)r) return false;
      if (chunkSize > r) f.seek(f.position() + (chunkSize - r));

      audioFormat   = rd16(fmt + 0);
      numChannels   = rd16(fmt + 2);
      sampleRate    = rd32(fmt + 4);
      bitsPerSample = rd16(fmt + 14);
      gotFmt = true;
    } else if (memcmp(ch, "data", 4) == 0) {
      dataOffset = f.position();
      dataSize = chunkSize;
      f.seek(dataOffset);
      gotData = true;
      break;
    } else {
      f.seek(f.position() + chunkSize);
    }
    if (chunkSize & 1) f.seek(f.position() + 1);
  }

  if (!gotFmt || !gotData) return false;
  if (audioFormat != 1) return false;
  if (bitsPerSample != 16) return false;
  if (!(numChannels == 1 || numChannels == 2)) return false;

  info.audioFormat = audioFormat;
  info.numChannels = numChannels;
  info.sampleRate = sampleRate;
  info.bitsPerSample = bitsPerSample;
  info.dataOffset = dataOffset;
  info.dataSize = dataSize;
  return true;
}

static void i2sInitIfNeeded(uint32_t sampleRate) {
  if (!g_i2sReady) {
    i2s_config_t cfg = {};
    cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    cfg.sample_rate = sampleRate;
    cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
    cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
    cfg.communication_format = I2S_COMM_FORMAT_I2S_MSB;
    cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    cfg.dma_buf_count = 8;
    cfg.dma_buf_len = 256;
    cfg.use_apll = false;
    cfg.tx_desc_auto_clear = true;

    i2s_pin_config_t pins = {};
    pins.bck_io_num = PIN_I2S_BCLK;
    pins.ws_io_num = PIN_I2S_LRCK;
    pins.data_out_num = PIN_I2S_DIN;
    pins.data_in_num = I2S_PIN_NO_CHANGE;

    i2s_driver_install(I2S_NUM_0, &cfg, 0, nullptr);
    i2s_set_pin(I2S_NUM_0, &pins);
    g_i2sReady = true;
  }
  i2s_set_clk(I2S_NUM_0, sampleRate, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
}

static bool playWav(const char* path) {
  File f = LittleFS.open(path, "r");
  if (!f) {
    Serial.printf("SOUND: file not found %s\n", path);
    return false;
  }

  WavHeaderInfo info{};
  if (!parseWavHeader(f, info)) {
    Serial.printf("SOUND: bad wav %s\n", path);
    f.close();
    return false;
  }

  i2sInitIfNeeded(info.sampleRate);
  f.seek(info.dataOffset);

  static uint8_t inbuf[1024];
  static int16_t outbuf[2048];

  uint32_t remaining = info.dataSize;

  while (remaining > 0) {
    size_t toRead = sizeof(inbuf);
    if (toRead > remaining) toRead = remaining;

    int r = f.read(inbuf, toRead);
    if (r <= 0) break;
    remaining -= (uint32_t)r;

    int samples = r / 2;
    if (samples <= 0) continue;

    int outSamples = 0;
    if (info.numChannels == 2) {
      for (int i = 0; i < samples; i++) {
        int16_t s = (int16_t)((uint16_t)inbuf[2*i] | ((uint16_t)inbuf[2*i+1] << 8));
        outbuf[i] = scaleSample(s);
      }
      outSamples = samples;
    } else {
      for (int i = 0; i < samples; i++) {
        int16_t s = (int16_t)((uint16_t)inbuf[2*i] | ((uint16_t)inbuf[2*i+1] << 8));
        s = scaleSample(s);
        outbuf[2*i]     = s;
        outbuf[2*i + 1] = s;
      }
      outSamples = samples * 2;
    }

    size_t bytesToWrite = (size_t)outSamples * sizeof(int16_t);
    size_t written = 0;
    i2s_write(I2S_NUM_0, (const char*)outbuf, bytesToWrite, &written, pdMS_TO_TICKS(50));
    vTaskDelay(1);
  }

  f.close();
  return true;
}

static const char* soundPath(SoundId id) {
  switch (id) {
    case SND_CONNECTED:    return "/connected.wav";
    case SND_DISCONNECTED: return "/connection_miss.wav";
    case SND_LOW_BATT:     return "/low_battery.wav";
    case SND_GOING_BACK:   return "/going_back.wav";
    default:               return nullptr;
  }
}

static void soundTask(void*) {
  for (;;) {
    SoundId id;
    if (xQueueReceive(g_soundQueue, &id, portMAX_DELAY) == pdTRUE) {
      const char* p = soundPath(id);
      if (p) playWav(p);
    }
  }
}

void sound_init() {
  if (!LittleFS.begin(false)) {
    Serial.println("SOUND: LittleFS mount FAIL");
  } else {
    Serial.println("SOUND: LittleFS mounted OK");
  }

  g_soundQueue = xQueueCreate(8, sizeof(SoundId));
  xTaskCreatePinnedToCore(soundTask, "soundTask", 8192, nullptr, 1, &g_soundTask, 0);
  Serial.println("SOUND: Initialized");
}

void enqueueSound(SoundId id) {
  if (!g_soundQueue) return;
  xQueueSend(g_soundQueue, &id, 0);
}

void setAttachment(bool on) {
  digitalWrite(PIN_RELAY_ATTACH, on ? HIGH : LOW);
  Serial.printf("ATTACHMENT %s\n", on ? "ON" : "OFF");
}

void setMount(bool on) {
  digitalWrite(PIN_RELAY_MOUNT, on ? HIGH : LOW);
  Serial.printf("MOUNT %s\n", on ? "ON" : "OFF");
}
