#include <stdint.h>

// =====================================================
//  Robot FULL: drive + 2 relays + sound (LittleFS + I2S)
//  Pins (ESP32):
//   Hoverboard UART: RX=16  TX=17   (Serial2)
//   I2S MAX98357A:   DIN=27 BCLK=26 LRCK=25
//   Relay ATTACHMENT = GPIO32  (active HIGH)
//   Relay MOUNT      = GPIO33  (active HIGH)
// =====================================================

// =======================
// WAV header info (MUST be before Arduino auto-prototypes)
// =======================
struct WavHeaderInfo {
  uint16_t audioFormat;
  uint16_t numChannels;
  uint32_t sampleRate;
  uint16_t bitsPerSample;
  uint32_t dataOffset;
  uint32_t dataSize;
};

// =======================
// Hoverboard serial protocol structs (MUST be before Arduino auto-prototypes)
// =======================
typedef struct __attribute__((packed)) {
  uint16_t start;
  int16_t  steer;
  int16_t  speed;
  uint16_t checksum;
} SerialCommand;

typedef struct __attribute__((packed)) {
  uint16_t start;
  int16_t  cmd1;
  int16_t  cmd2;
  int16_t  speedR_meas;
  int16_t  speedL_meas;
  int16_t  batVoltage;
  int16_t  boardTemp;
  uint16_t cmdLed;
  uint16_t checksum;
} SerialFeedback;

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "esp_system.h"

#include <FS.h>
#include <LittleFS.h>
#include <driver/i2s.h>

// =======================
// WiFi AP
// =======================
const char* ssid = "Robot";
const char* pass = "CHANGE_ME_MIN_8_CHARS";

AsyncWebServer server(81);
AsyncWebSocket ws("/ws");

// =======================
// Pins
// =======================
static constexpr int PIN_HOVER_RX = 16;
static constexpr int PIN_HOVER_TX = 17;

static constexpr int PIN_I2S_DOUT = 27; // DIN
static constexpr int PIN_I2S_BCLK = 26; // BCLK
static constexpr int PIN_I2S_LRCK = 25; // LRC/WS

static constexpr int PIN_RELAY_ATTACHMENT = 32; // active HIGH
static constexpr int PIN_RELAY_MOUNT      = 33; // active HIGH

// =======================
// Hoverboard serial protocol
// =======================
#define HOVER_SERIAL_BAUD 115200
#define START_FRAME       0xABCD

// =======================
// Drive tuning
// =======================
static constexpr int MAX_SPEED_PERCENT = 70;     // -70..70 (after input divide)
static constexpr int16_t HOVER_MAX_CMD = 300;    // hoverboard command scale
static constexpr int INPUT_DIV = 2;             // app values / 2 (calmer)

static constexpr uint32_t HOVER_SEND_MS  = 20;
static constexpr uint32_t CMD_TIMEOUT_MS = 400;

// ramp (percent domain)
static constexpr uint32_t RAMP_UPDATE_MS = 20;
static constexpr int RAMP_STEP_UP_PER_TICK   = 1;
static constexpr int RAMP_STEP_DOWN_PER_TICK = 1;

// extra smoothing in hoverboard command domain
static constexpr int16_t SLEW_SPEED_PER_SEND = 4;
static constexpr int16_t SLEW_STEER_PER_SEND = 6;

int16_t g_cmdSpeed = 0;
int16_t g_cmdSteer = 0;

// =======================
// Battery telemetry
// =======================
static constexpr uint32_t BAT_SEND_MS = 500;
static constexpr float BAT_VOLT_SCALE = 0.01f;
static constexpr float TEMP_SCALE     = 0.1f;
static constexpr float BAT_FILTER_ALPHA = 0.25f;

static constexpr int CELL_COUNT = 10; // 10S
static constexpr int SEND_BAT_PCT_ONLY = 1; // UI
static constexpr int SEND_BAT_VERBOSE  = 1; // optional

// =======================
// Sound
// =======================
static constexpr int SOUND_VOLUME_DEFAULT = 80; // (0..100)
static int g_soundVolume = SOUND_VOLUME_DEFAULT;

static constexpr uint32_t LOW_BATT_REPEAT_MS = 15000; // every 15s
static constexpr int LOW_BATT_THRESHOLD_PCT  = 30;    // <30%

enum SoundId : uint8_t {
  SND_NONE = 0,
  SND_CONNECTED = 1,
  SND_DISCONNECTED = 2,
  SND_LOW_BATT = 3,
  SND_GOING_BACK = 4,
};

static QueueHandle_t g_soundQueue = nullptr;
static TaskHandle_t  g_soundTask  = nullptr;
static bool g_i2sReady = false;
static uint32_t g_lastLowBattSoundMs = 0;

// to fix "connected / disconnected / connected" glitches:
static volatile int g_wsClientCount = 0;
static bool g_discPending = false;
static uint32_t g_discAtMs = 0;
static constexpr uint32_t DISC_GRACE_MS = 800;
static uint32_t g_bootMs = 0;

// =======================
// Debug
// =======================
static constexpr uint32_t DEBUG_PRINT_MS = 200;
uint32_t g_lastDebugMs = 0;
int g_lastDbgL = 9999;
int g_lastDbgR = 9999;

// =======================
// WS buffer
// =======================
static constexpr size_t MAX_WS_MSG = 128;

// =======================
// Command state
// =======================
volatile int16_t g_targetLeft  = 0;
volatile int16_t g_targetRight = 0;
int16_t g_curLeft  = 0;
int16_t g_curRight = 0;

volatile uint32_t g_lastCmdMs = 0;
uint32_t g_lastSendMs = 0;
uint32_t g_lastRampMs = 0;
bool g_isFailSafeStopping = false;

// =======================
// Hoverboard feedback
// =======================
SerialFeedback Feedback{};
SerialFeedback NewFeedback{};

uint8_t  fb_idx = 0;
uint16_t fb_bufStartFrame = 0;
uint8_t* fb_p = nullptr;
uint8_t  fb_in = 0;
uint8_t  fb_in_prev = 0;

volatile bool g_haveFeedback = false;
uint32_t g_lastBatSendMs = 0;
float g_batVoltFiltered = 0.0f;

// =======================
// Helpers
// =======================
static inline int16_t clampi16(int32_t v, int16_t lo, int16_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return (int16_t)v;
}

static inline int16_t stepToward16(int16_t cur, int16_t target, int16_t maxDelta) {
  int32_t diff = (int32_t)target - (int32_t)cur;
  if (diff >  maxDelta) diff =  maxDelta;
  if (diff < -maxDelta) diff = -maxDelta;
  return (int16_t)((int32_t)cur + diff);
}

static inline void trimInPlace(char* s) {
  int n = (int)strlen(s);
  while (n > 0 && (s[n - 1] == ' ' || s[n - 1] == '\r' || s[n - 1] == '\n' || s[n - 1] == '\t')) s[--n] = 0;
  int i = 0;
  while (s[i] == ' ' || s[i] == '\r' || s[i] == '\n' || s[i] == '\t') i++;
  if (i > 0) memmove(s, s + i, strlen(s + i) + 1);
}

static inline bool streq(const char* a, const char* b) { return strcmp(a, b) == 0; }
static inline bool startsWith(const char* s, const char* pref) { return strncmp(s, pref, strlen(pref)) == 0; }

// =======================
// Relay control (active HIGH)
// =======================
void setAttachment(bool on) {
  digitalWrite(PIN_RELAY_ATTACHMENT, on ? HIGH : LOW);
  Serial.printf("ATTACHMENT %s\n", on ? "ON" : "OFF");
}

void setMount(bool on) {
  digitalWrite(PIN_RELAY_MOUNT, on ? HIGH : LOW);
  Serial.printf("MOUNT %s\n", on ? "ON" : "OFF");
}

// =======================
// SOC table (rough)
// =======================
static int socFromCellV(float vc) {
  const int N = 11;
  const float V[N] = {4.20f, 4.10f, 4.00f, 3.90f, 3.80f, 3.75f, 3.70f, 3.65f, 3.60f, 3.50f, 3.40f};
  const int   P[N] = { 100 ,   90 ,   80 ,   70 ,   60 ,   50 ,   40 ,   30 ,   20 ,   10 ,    0 };

  if (vc >= V[0]) return 100;
  if (vc <= V[N - 1]) return 0;

  for (int i = 0; i < N - 1; i++) {
    if (vc <= V[i] && vc >= V[i + 1]) {
      float t = (vc - V[i + 1]) / (V[i] - V[i + 1]);
      float pct = (float)P[i + 1] + t * (float)(P[i] - P[i + 1]);
      int out = (int)(pct + 0.5f);
      if (out < 0) out = 0;
      if (out > 100) out = 100;
      return out;
    }
  }
  return 0;
}

// =======================
// Hoverboard send
// =======================
void sendHoverboard(int16_t steer, int16_t speed) {
  SerialCommand cmd;
  cmd.start = (uint16_t)START_FRAME;
  cmd.steer = steer;
  cmd.speed = speed;
  cmd.checksum = (uint16_t)(cmd.start ^ cmd.steer ^ cmd.speed);
  Serial2.write((uint8_t*)&cmd, sizeof(cmd));
}

void requestSmoothStop(const char* reason) {
  g_targetLeft = 0;
  g_targetRight = 0;
  Serial.printf("MOTORS: SMOOTH STOP (%s)\n", reason);
}

// left/right (-70..70) -> speed/steer -> SLEW -> send
void drive(int16_t leftPct, int16_t rightPct) {
  leftPct  = clampi16(leftPct,  -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);
  rightPct = clampi16(rightPct, -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);

  int32_t speedT = (int32_t)(leftPct + rightPct) * (int32_t)HOVER_MAX_CMD / (2 * MAX_SPEED_PERCENT);
  int32_t steerT = (int32_t)(rightPct - leftPct) * (int32_t)HOVER_MAX_CMD / (2 * MAX_SPEED_PERCENT);

  int16_t spT = clampi16(speedT, -HOVER_MAX_CMD, HOVER_MAX_CMD);
  int16_t stT = clampi16(steerT, -HOVER_MAX_CMD, HOVER_MAX_CMD);

  g_cmdSpeed = stepToward16(g_cmdSpeed, spT, SLEW_SPEED_PER_SEND);
  g_cmdSteer = stepToward16(g_cmdSteer, stT, SLEW_STEER_PER_SEND);

  sendHoverboard(g_cmdSteer, g_cmdSpeed);
}

// =======================
// Ramp (percent domain)
// =======================
static inline int16_t stepTowardsLimited(int16_t cur, int16_t target, int maxDelta) {
  int diff = (int)target - (int)cur;
  if (diff >  maxDelta) diff =  maxDelta;
  if (diff < -maxDelta) diff = -maxDelta;
  return (int16_t)(cur + diff);
}

static inline int limitForAxis(int16_t cur, int16_t target, int ticks) {
  int absCur = abs((int)cur);
  int absTgt = abs((int)target);
  bool up = (absTgt > absCur);
  int step = up ? RAMP_STEP_UP_PER_TICK : RAMP_STEP_DOWN_PER_TICK;
  int maxDelta = ticks * step;
  if (maxDelta < 1) maxDelta = 1;
  return maxDelta;
}

void updateRamp() {
  uint32_t now = millis();
  uint32_t dt = now - g_lastRampMs;
  if (dt < RAMP_UPDATE_MS) return;

  uint32_t ticks = dt / RAMP_UPDATE_MS;
  g_lastRampMs += ticks * RAMP_UPDATE_MS;

  int16_t tL = clampi16(g_targetLeft,  -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);
  int16_t tR = clampi16(g_targetRight, -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);

  int limL = limitForAxis(g_curLeft,  tL, (int)ticks);
  int limR = limitForAxis(g_curRight, tR, (int)ticks);

  g_curLeft  = clampi16(stepTowardsLimited(g_curLeft,  tL, limL), -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);
  g_curRight = clampi16(stepTowardsLimited(g_curRight, tR, limR), -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);

  if (g_isFailSafeStopping && g_curLeft == 0 && g_curRight == 0 && tL == 0 && tR == 0) {
    g_isFailSafeStopping = false;
    Serial.println("FAILSAFE: STOPPED");
  }
}

// =======================
// Feedback receive
// =======================
void receiveHoverboardFeedback() {
  while (Serial2.available()) {
    fb_in = (uint8_t)Serial2.read();
    fb_bufStartFrame = ((uint16_t)fb_in << 8) | fb_in_prev;

    if (fb_bufStartFrame == START_FRAME) {
      fb_p = (uint8_t*)&NewFeedback;
      *fb_p++ = fb_in_prev;
      *fb_p++ = fb_in;
      fb_idx = 2;
    } else if (fb_idx >= 2 && fb_idx < sizeof(SerialFeedback)) {
      *fb_p++ = fb_in;
      fb_idx++;
    }

    if (fb_idx == sizeof(SerialFeedback)) {
      uint16_t cs = (uint16_t)(
        NewFeedback.start ^
        NewFeedback.cmd1 ^ NewFeedback.cmd2 ^
        NewFeedback.speedR_meas ^ NewFeedback.speedL_meas ^
        NewFeedback.batVoltage ^ NewFeedback.boardTemp ^
        NewFeedback.cmdLed
      );

      if (NewFeedback.start == START_FRAME && cs == NewFeedback.checksum) {
        memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
        g_haveFeedback = true;
      }
      fb_idx = 0;
    }
    fb_in_prev = fb_in;
  }
}

// =======================
// SOUND: WAV + I2S
// =======================
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

bool parseWavHeader(File &f, WavHeaderInfo &info) {
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

void i2sInitIfNeeded(uint32_t sampleRate) {
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
    pins.data_out_num = PIN_I2S_DOUT;
    pins.data_in_num = I2S_PIN_NO_CHANGE;

    i2s_driver_install(I2S_NUM_0, &cfg, 0, nullptr);
    i2s_set_pin(I2S_NUM_0, &pins);
    g_i2sReady = true;
  }
  i2s_set_clk(I2S_NUM_0, sampleRate, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
}

bool playWav(const char* path) {
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

const char* soundPath(SoundId id) {
  switch (id) {
    case SND_CONNECTED:    return "/connected.wav";
    case SND_DISCONNECTED: return "/connection_miss.wav";
    case SND_LOW_BATT:     return "/low_battery.wav";
    case SND_GOING_BACK:   return "/going_back.wav";
    default:               return nullptr;
  }
}

static inline void enqueueSound(SoundId id) {
  if (!g_soundQueue) return;
  xQueueSend(g_soundQueue, &id, 0);
}

void soundTask(void*) {
  for (;;) {
    SoundId id;
    if (xQueueReceive(g_soundQueue, &id, portMAX_DELAY) == pdTRUE) {
      const char* p = soundPath(id);
      if (p) playWav(p);
    }
  }
}

// =======================
// Battery to app + low battery sound
// =======================
void sendBatteryToAppIfNeeded() {
  if (!g_haveFeedback) return;

  uint32_t now = millis();
  if (now - g_lastBatSendMs < BAT_SEND_MS) return;
  g_lastBatSendMs = now;

  float vPack = (float)Feedback.batVoltage * BAT_VOLT_SCALE;
  float tempC = (float)Feedback.boardTemp * TEMP_SCALE;

  if (g_batVoltFiltered <= 0.01f) g_batVoltFiltered = vPack;
  g_batVoltFiltered = g_batVoltFiltered + BAT_FILTER_ALPHA * (vPack - g_batVoltFiltered);

  float vCell = (CELL_COUNT > 0) ? (g_batVoltFiltered / (float)CELL_COUNT) : 0.0f;
  int pct = socFromCellV(vCell);

  if (pct < LOW_BATT_THRESHOLD_PCT) {
    if (now - g_lastLowBattSoundMs >= LOW_BATT_REPEAT_MS) {
      g_lastLowBattSoundMs = now;
      enqueueSound(SND_LOW_BATT);
    }
  }

  if (ws.count() > 0) {
    if (SEND_BAT_PCT_ONLY) {
      char sPct[24];
      snprintf(sPct, sizeof(sPct), "BAT_PCT,%d", pct);
      ws.textAll(sPct);
    }
    if (SEND_BAT_VERBOSE) {
      char out[160];
      snprintf(out, sizeof(out),
               "BAT,V=%.2fV,P=%d%%,Vf=%.2fV,temp=%.1fC",
               vPack, pct, g_batVoltFiltered, tempC);
      ws.textAll(out);
    }
  }
}

// =======================
// Debug wheel commands
// =======================
void debugPrintWheelSpeeds() {
  uint32_t now = millis();
  if (now - g_lastDebugMs < DEBUG_PRINT_MS) return;

  if (g_curLeft != g_lastDbgL || g_curRight != g_lastDbgR) {
    Serial.printf("LEFT CMD: %d   RIGHT CMD: %d\n", (int)g_curLeft, (int)g_curRight);
    g_lastDbgL = g_curLeft;
    g_lastDbgR = g_curRight;
  }
  g_lastDebugMs = now;
}

// =======================
// Parse "M,left,right"
// =======================
bool parseMove(const char* msg, int16_t& outL, int16_t& outR) {
  if (!startsWith(msg, "M,")) return false;

  const char* p = msg + 2;
  char* end1 = nullptr;
  long L = strtol(p, &end1, 10);
  if (!end1 || *end1 != ',') return false;

  const char* p2 = end1 + 1;
  char* end2 = nullptr;
  long R = strtol(p2, &end2, 10);
  if (!end2) return false;

  L = L / INPUT_DIV;
  R = R / INPUT_DIV;

  outL = clampi16(L, -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);
  outR = clampi16(R, -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);
  return true;
}

// =======================
// Hoverboard disconnect sound debounce
// =======================
void handleDisconnectSoundDebounce() {
  if (!g_discPending) return;
  uint32_t now = millis();
  if (now >= g_discAtMs) {
    g_discPending = false;
    if (g_wsClientCount == 0) {
      enqueueSound(SND_DISCONNECTED);
    }
  }
}

// =======================
// WebSocket handler
// =======================
void onWsEvent(AsyncWebSocket *serverPtr, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {

  if (type == WS_EVT_CONNECT) {
    Serial.printf("WS CONNECT id=%u ip=%s\n", client->id(), client->remoteIP().toString().c_str());
    client->text("STATE,CONNECTED");

    g_isFailSafeStopping = false;

    // cancel pending disconnect sound
    g_discPending = false;

    g_wsClientCount++;
    // play CONNECTED only on 0->1 and not instantly at boot
    if (g_wsClientCount == 1 && (millis() - g_bootMs) > 1500) {
      enqueueSound(SND_CONNECTED);
    }
    return;
  }

  if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WS DISCONNECT id=%u -> SMOOTH STOP\n", client->id());
    requestSmoothStop("ws_disconnect");

    if (g_wsClientCount > 0) g_wsClientCount--;
    if (g_wsClientCount == 0) {
      // delay DISCONNECTED a bit (fix connect/disconnect/connect glitch)
      g_discPending = true;
      g_discAtMs = millis() + DISC_GRACE_MS;
    }
    return;
  }

  if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;

    if (!info->final || info->index != 0 || info->len != len) return;
    if (info->opcode != WS_TEXT) return;
    if (len >= MAX_WS_MSG) { client->text("ERR,TOO_LONG"); return; }

    char msg[MAX_WS_MSG];
    memcpy(msg, data, len);
    msg[len] = 0;
    trimInPlace(msg);

    g_lastCmdMs = millis();
    g_isFailSafeStopping = false;

    // ---- ping/stop
    if (streq(msg, "PING")) { client->text("PONG"); return; }
    if (streq(msg, "STOP")) { requestSmoothStop("STOP_cmd"); client->text("OK STOP"); return; }

    // ---- movement
    int16_t L=0, R=0;
    if (parseMove(msg, L, R)) {
      g_targetLeft = L;
      g_targetRight = R;
      client->text("OK M");
      return;
    }

    // ---- relays from app
    if (streq(msg, "ATTACHMENT_ON"))  { setAttachment(true);  client->text("OK ATTACHMENT_ON");  return; }
    if (streq(msg, "ATTACHMENT_OFF")) { setAttachment(false); client->text("OK ATTACHMENT_OFF"); return; }
    if (streq(msg, "MOUNT_ON"))       { setMount(true);       client->text("OK MOUNT_ON");       return; }
    if (streq(msg, "MOUNT_OFF"))      { setMount(false);      client->text("OK MOUNT_OFF");      return; }

    // ---- optional sound trigger:
    // SOUND:1 / SOUND,1  -> plays by id (1=connected,2=disconnect,3=low,4=going_back)
    if (startsWith(msg, "SOUND:") || startsWith(msg, "SOUND,")) {
      const char* p = msg + 6;
      if (*p == ':' || *p == ',') p++;
      int n = atoi(p);
      if (n >= 1 && n <= 4) {
        enqueueSound((SoundId)n);
        client->text("OK SOUND");
      } else {
        client->text("ERR SOUND_RANGE");
      }
      return;
    }

    client->text("ERR,UNKNOWN");
  }
}

// =======================
// Setup / Loop
// =======================
void setup() {
  Serial.begin(115200);
  delay(200);

  g_bootMs = millis();
  Serial.printf("\n=== Robot START | reset reason=%d ===\n", (int)esp_reset_reason());

  // relays
  pinMode(PIN_RELAY_ATTACHMENT, OUTPUT);
  pinMode(PIN_RELAY_MOUNT, OUTPUT);
  setAttachment(false);
  setMount(false);

  // hoverboard
  Serial2.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, PIN_HOVER_RX, PIN_HOVER_TX);
  Serial.printf("HoverSerial: %d baud, RX=%d TX=%d\n", HOVER_SERIAL_BAUD, PIN_HOVER_RX, PIN_HOVER_TX);

  // LittleFS for sounds
  if (!LittleFS.begin(false)) {
    Serial.println("LittleFS mount FAIL");
  } else {
    Serial.println("LittleFS mounted OK");
    File root = LittleFS.open("/");
    File file = root.openNextFile();
    while (file) {
      Serial.printf("FILE: %s  SIZE: %u\n", file.name(), (unsigned)file.size());
      file = root.openNextFile();
    }
  }

  // Sound task on CORE 0
  g_soundQueue = xQueueCreate(8, sizeof(SoundId));
  xTaskCreatePinnedToCore(soundTask, "soundTask", 8192, nullptr, 1, &g_soundTask, 0);

  // WiFi AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, pass);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/ping", HTTP_GET, [](AsyncWebServerRequest *req){
    req->send(200, "text/plain", "OK");
  });

  server.begin();
  Serial.println("HTTP+WS server on port 81, WS path /ws");
  Serial.println("Commands: M,left,right | STOP | ATTACHMENT_ON/OFF | MOUNT_ON/OFF | SOUND:1..4");

  g_lastCmdMs = millis();
  g_lastSendMs = millis();
  g_lastRampMs = millis();
  g_lastDebugMs = millis();
  g_lastBatSendMs = millis();
  g_lastLowBattSoundMs = 0;

  g_targetLeft = 0;
  g_targetRight = 0;
  g_curLeft = 0;
  g_curRight = 0;

  g_cmdSpeed = 0;
  g_cmdSteer = 0;

  drive(0, 0);
  Serial.println("READY");
}

void loop() {
  ws.cleanupClients();

  const uint32_t now = millis();

  // hoverboard feedback
  receiveHoverboardFeedback();

  // failsafe -> smooth stop
  if (now - g_lastCmdMs > CMD_TIMEOUT_MS) {
    if (!g_isFailSafeStopping && (g_targetLeft != 0 || g_targetRight != 0 || g_curLeft != 0 || g_curRight != 0)) {
      g_isFailSafeStopping = true;
      requestSmoothStop("timeout");
    }
  }

  // ramp
  updateRamp();

  // send drive cmd
  if (now - g_lastSendMs >= HOVER_SEND_MS) {
    g_lastSendMs = now;
    drive(g_curLeft, g_curRight);
  }

  // battery + low battery sound
  sendBatteryToAppIfNeeded();

  // disconnect sound debounce
  handleDisconnectSoundDebounce();

  // debug
  debugPrintWheelSpeeds();
}
