#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

static constexpr int PIN_GPS_RX = 4; // ESP32 RX <- F9P TX
static constexpr int PIN_GPS_TX = 5; // ESP32 TX -> F9P RX
static constexpr uint32_t GPS_BAUD = 38400;

static constexpr char ROVER_WIFI_SSID[] = "RTK-Rover";
static constexpr char ROVER_WIFI_PASS[] = "rtk-rover-123";
static const IPAddress ROVER_IP(192, 168, 4, 1);
static constexpr uint16_t ROVER_RTCM_PORT = 2101;

static constexpr uint32_t SURVEY_IN_SECONDS = 60;
static constexpr uint32_t SURVEY_IN_ACC_LIMIT_0_1MM = 50000; // 5.0 m, fast field test
static constexpr uint32_t WIFI_RETRY_MS = 3000;
static constexpr uint32_t STATUS_MS = 1000;
static constexpr uint32_t SVIN_POLL_MS = 1000;

// u-blox configuration keys for VALSET.
static constexpr uint32_t CFG_UART1INPROT_UBX = 0x11731001;
static constexpr uint32_t CFG_UART1INPROT_NMEA = 0x11731002;
static constexpr uint32_t CFG_UART1INPROT_RTCM3 = 0x11731004;
static constexpr uint32_t CFG_UART1OUTPROT_UBX = 0x11741001;
static constexpr uint32_t CFG_UART1OUTPROT_NMEA = 0x11741002;
static constexpr uint32_t CFG_UART1OUTPROT_RTCM3 = 0x11741004;
static constexpr uint32_t CFG_RTCM_1005_UART1 = 0x219122BE;
static constexpr uint32_t CFG_RTCM_1074_UART1 = 0x2191235F;
static constexpr uint32_t CFG_RTCM_1084_UART1 = 0x21912364;
static constexpr uint32_t CFG_RTCM_1094_UART1 = 0x21912369;
static constexpr uint32_t CFG_RTCM_1124_UART1 = 0x2191236E;
static constexpr uint32_t CFG_RTCM_1230_UART1 = 0x21912304;
static constexpr uint32_t CFG_RTCM_4072_0_UART1 = 0x219122FF;
static constexpr uint32_t CFG_TMODE_MODE = 0x2103C001;
static constexpr uint32_t CFG_TMODE_SVIN_MIN_DUR = 0x41034010;
static constexpr uint32_t CFG_TMODE_SVIN_ACC_LIMIT = 0x41034011;

static HardwareSerial GpsSerial(1);
static WiFiUDP rtcmUdp;

static uint32_t lastWiFiAttemptMs = 0;
static uint32_t lastStatusMs = 0;
static uint32_t lastSvinPollMs = 0;
static uint32_t rtcmBytesSent = 0;
static uint32_t rtcmPacketsSent = 0;

struct SvinStatus {
  bool active = false;
  bool valid = false;
  uint32_t dur = 0;
  uint32_t meanAcc01mm = 0;
  uint32_t lastMs = 0;
};

static SvinStatus svin;

static void putU16(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
}

static void putU32(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

static uint32_t getU32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void writeUbx(uint8_t cls, uint8_t id, const uint8_t* payload,
                     uint16_t len) {
  uint8_t ckA = 0;
  uint8_t ckB = 0;
  auto ck = [&](uint8_t b) {
    ckA += b;
    ckB += ckA;
  };

  GpsSerial.write(0xB5);
  GpsSerial.write(0x62);
  GpsSerial.write(cls);
  ck(cls);
  GpsSerial.write(id);
  ck(id);
  GpsSerial.write((uint8_t)len);
  ck((uint8_t)len);
  GpsSerial.write((uint8_t)(len >> 8));
  ck((uint8_t)(len >> 8));
  for (uint16_t i = 0; i < len; i++) {
    GpsSerial.write(payload[i]);
    ck(payload[i]);
  }
  GpsSerial.write(ckA);
  GpsSerial.write(ckB);
}

static void pollUbx(uint8_t cls, uint8_t id) {
  writeUbx(cls, id, nullptr, 0);
}

struct ValItem {
  uint32_t key;
  uint32_t value;
  uint8_t size;
};

static void sendValset(const ValItem* items, size_t count) {
  uint8_t payload[160];
  size_t n = 0;
  payload[n++] = 0;    // version
  payload[n++] = 0x01; // RAM layer only
  payload[n++] = 0;
  payload[n++] = 0;

  for (size_t i = 0; i < count; i++) {
    putU32(payload + n, items[i].key);
    n += 4;
    for (uint8_t b = 0; b < items[i].size; b++) {
      payload[n++] = (uint8_t)(items[i].value >> (8 * b));
    }
  }

  writeUbx(0x06, 0x8A, payload, (uint16_t)n);
  delay(80);
}

static void configureBase() {
  const ValItem portItems[] = {
      {CFG_UART1INPROT_UBX, 1, 1},
      {CFG_UART1INPROT_NMEA, 0, 1},
      {CFG_UART1INPROT_RTCM3, 0, 1},
      {CFG_UART1OUTPROT_UBX, 1, 1},
      {CFG_UART1OUTPROT_NMEA, 0, 1},
      {CFG_UART1OUTPROT_RTCM3, 1, 1},
  };
  sendValset(portItems, sizeof(portItems) / sizeof(portItems[0]));

  const ValItem rtcmItems[] = {
      {CFG_RTCM_1005_UART1, 1, 1},   {CFG_RTCM_1074_UART1, 1, 1},
      {CFG_RTCM_1084_UART1, 1, 1},   {CFG_RTCM_1094_UART1, 1, 1},
      {CFG_RTCM_1124_UART1, 1, 1},   {CFG_RTCM_1230_UART1, 10, 1},
      {CFG_RTCM_4072_0_UART1, 1, 1},
  };
  sendValset(rtcmItems, sizeof(rtcmItems) / sizeof(rtcmItems[0]));

  const ValItem disableTmode[] = {
      {CFG_TMODE_MODE, 0, 1},
  };
  sendValset(disableTmode, sizeof(disableTmode) / sizeof(disableTmode[0]));

  const ValItem surveyItems[] = {
      {CFG_TMODE_SVIN_MIN_DUR, SURVEY_IN_SECONDS, 4},
      {CFG_TMODE_SVIN_ACC_LIMIT, SURVEY_IN_ACC_LIMIT_0_1MM, 4},
      {CFG_TMODE_MODE, 1, 1},
  };
  sendValset(surveyItems, sizeof(surveyItems) / sizeof(surveyItems[0]));

  Serial.println("GNSS: base survey-in and RTCM configured");
}

enum UbxState { U_SYNC1, U_SYNC2, U_CLASS, U_ID, U_LEN1, U_LEN2, U_PAYLOAD, U_CKA, U_CKB };
static UbxState ubxState = U_SYNC1;
static uint8_t ubxClass = 0;
static uint8_t ubxId = 0;
static uint16_t ubxLen = 0;
static uint16_t ubxCount = 0;
static uint8_t ubxPayload[128];
static uint8_t ubxCkA = 0;
static uint8_t ubxCkB = 0;

static void processUbx() {
  if (ubxClass == 0x01 && ubxId == 0x3B && ubxLen >= 40) {
    svin.dur = getU32(ubxPayload + 8);
    svin.meanAcc01mm = getU32(ubxPayload + 28);
    svin.valid = ubxPayload[36] != 0;
    svin.active = ubxPayload[37] != 0;
    svin.lastMs = millis();
  }
}

static void feedUbx(uint8_t b) {
  switch (ubxState) {
    case U_SYNC1:
      if (b == 0xB5) ubxState = U_SYNC2;
      break;
    case U_SYNC2:
      if (b == 0x62) {
        ubxState = U_CLASS;
        ubxCkA = 0;
        ubxCkB = 0;
      } else {
        ubxState = U_SYNC1;
      }
      break;
    case U_CLASS:
      ubxClass = b;
      ubxCkA += b;
      ubxCkB += ubxCkA;
      ubxState = U_ID;
      break;
    case U_ID:
      ubxId = b;
      ubxCkA += b;
      ubxCkB += ubxCkA;
      ubxState = U_LEN1;
      break;
    case U_LEN1:
      ubxLen = b;
      ubxCkA += b;
      ubxCkB += ubxCkA;
      ubxState = U_LEN2;
      break;
    case U_LEN2:
      ubxLen |= ((uint16_t)b << 8);
      ubxCkA += b;
      ubxCkB += ubxCkA;
      ubxCount = 0;
      ubxState = (ubxLen == 0) ? U_CKA
                               : (ubxLen > sizeof(ubxPayload) ? U_SYNC1 : U_PAYLOAD);
      break;
    case U_PAYLOAD:
      ubxPayload[ubxCount++] = b;
      ubxCkA += b;
      ubxCkB += ubxCkA;
      if (ubxCount >= ubxLen) ubxState = U_CKA;
      break;
    case U_CKA:
      ubxState = (b == ubxCkA) ? U_CKB : U_SYNC1;
      break;
    case U_CKB:
      if (b == ubxCkB) processUbx();
      ubxState = U_SYNC1;
      break;
  }
}

enum RtcmState { R_WAIT, R_LEN1, R_LEN2, R_BODY };
static RtcmState rtcmState = R_WAIT;
static uint8_t rtcmBuf[1100];
static uint16_t rtcmLen = 0;
static uint16_t rtcmTarget = 0;

static void sendRtcmFrame() {
  if (WiFi.status() != WL_CONNECTED || rtcmLen == 0) return;
  rtcmUdp.beginPacket(ROVER_IP, ROVER_RTCM_PORT);
  rtcmUdp.write(rtcmBuf, rtcmLen);
  rtcmUdp.endPacket();
  rtcmBytesSent += rtcmLen;
  rtcmPacketsSent++;
}

static void feedRtcm(uint8_t b) {
  switch (rtcmState) {
    case R_WAIT:
      if (b == 0xD3) {
        rtcmBuf[0] = b;
        rtcmLen = 1;
        rtcmState = R_LEN1;
      }
      break;
    case R_LEN1:
      rtcmBuf[rtcmLen++] = b;
      rtcmTarget = ((uint16_t)(b & 0x03) << 8);
      rtcmState = R_LEN2;
      break;
    case R_LEN2:
      rtcmBuf[rtcmLen++] = b;
      rtcmTarget = (rtcmTarget | b) + 6; // header + payload + CRC
      if (rtcmTarget > sizeof(rtcmBuf)) {
        rtcmState = R_WAIT;
      } else {
        rtcmState = R_BODY;
      }
      break;
    case R_BODY:
      rtcmBuf[rtcmLen++] = b;
      if (rtcmLen >= rtcmTarget) {
        sendRtcmFrame();
        rtcmState = R_WAIT;
      }
      break;
  }
}

static void connectWiFi() {
  const uint32_t now = millis();
  if (WiFi.status() == WL_CONNECTED) return;
  if (now - lastWiFiAttemptMs < WIFI_RETRY_MS) return;
  lastWiFiAttemptMs = now;
  WiFi.mode(WIFI_STA);
  WiFi.begin(ROVER_WIFI_SSID, ROVER_WIFI_PASS);
}

static void pollSvin() {
  const uint32_t now = millis();
  if (now - lastSvinPollMs < SVIN_POLL_MS) return;
  lastSvinPollMs = now;
  pollUbx(0x01, 0x3B);
}

static void printStatus() {
  const uint32_t now = millis();
  if (now - lastStatusMs < STATUS_MS) return;
  lastStatusMs = now;
  Serial.printf("BASE wifi=%s svin_active=%u svin_valid=%u dur=%lus meanAcc=%.3fm rtcm=%lubytes/%lupkts\n",
                WiFi.status() == WL_CONNECTED ? "connected" : "not_connected",
                svin.active ? 1 : 0,
                svin.valid ? 1 : 0,
                (unsigned long)svin.dur,
                (double)svin.meanAcc01mm / 10000.0,
                (unsigned long)rtcmBytesSent,
                (unsigned long)rtcmPacketsSent);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("ESP32 ZED-F9P RTK BASE");

  GpsSerial.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  delay(500);
  configureBase();
  connectWiFi();
}

void loop() {
  connectWiFi();
  while (GpsSerial.available()) {
    const uint8_t b = (uint8_t)GpsSerial.read();
    feedUbx(b);
    feedRtcm(b);
  }
  pollSvin();
  printStatus();
}
