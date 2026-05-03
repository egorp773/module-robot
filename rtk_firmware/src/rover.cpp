#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <ArduinoOTA.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Adafruit_BNO08x.h>

#if __has_include("rtk_config_private.h")
#include "rtk_config_private.h"
#else
#include "rtk_config.example.h"
#endif

static constexpr uint32_t GPS_BAUDS[] = {38400, 9600, 115200};

struct GpsPortConfig {
  const char* name;
  int rx; // ESP32 RX <- F9P TX
  int tx; // ESP32 TX -> F9P RX
};

static constexpr GpsPortConfig GPS_PORTS[] = {
    {"GPIO4/GPIO5", 4, 5},
    {"GPIO5/GPIO4", 5, 4},
};

static constexpr bool ROUTER_MODE = true;
static constexpr char ROUTER_WIFI_SSID[] = RTK_ROUTER_WIFI_SSID;
static constexpr char ROUTER_WIFI_PASS[] = RTK_ROUTER_WIFI_PASS;
static const IPAddress ROVER_STA_IP(RTK_ROVER_IP_A, RTK_ROVER_IP_B,
                                    RTK_ROVER_IP_C, RTK_ROVER_IP_D);
#ifndef RTK_BASE_IP_A
#define RTK_BASE_IP_A 192
#define RTK_BASE_IP_B 168
#define RTK_BASE_IP_C 31
#define RTK_BASE_IP_D 207
#endif
static const IPAddress BASE_STA_IP(RTK_BASE_IP_A, RTK_BASE_IP_B,
                                   RTK_BASE_IP_C, RTK_BASE_IP_D);
static const IPAddress ROUTER_GATEWAY(RTK_ROUTER_GATEWAY_A,
                                      RTK_ROUTER_GATEWAY_B,
                                      RTK_ROUTER_GATEWAY_C,
                                      RTK_ROUTER_GATEWAY_D);
static const IPAddress ROUTER_SUBNET(255, 255, 255, 0);

static constexpr char AP_WIFI_SSID[] = "RTK-Rover";
static constexpr char AP_WIFI_PASS[] = "rtk-rover-123";
static constexpr uint16_t WS_PORT = 81;
static constexpr uint16_t RTCM_UDP_PORT = 2101;
static constexpr uint16_t RTCM_TCP_PORT = 2102;
static constexpr uint32_t GPS_BROADCAST_MS = 200;
static constexpr uint32_t LEGACY_BROADCAST_MS = 1000;
static constexpr uint32_t STATUS_MS = 3000;
static constexpr uint32_t WIFI_RETRY_MS = 5000;
static constexpr uint32_t RTCM_TCP_CONNECT_TIMEOUT_MS = 250;
static constexpr uint32_t RTCM_WARN_AGE_MS = 5000;
static constexpr uint32_t RTCM_RESTART_AGE_MS = 5000;
static constexpr uint32_t RTCM_WIFI_RECOVER_AGE_MS = 30000;
static constexpr uint32_t RTCM_WIFI_RECOVER_MS = 15000;
static constexpr int PIN_MOTOR_RX = 16;
static constexpr int PIN_MOTOR_TX = 17;
static constexpr int PIN_IMU_SDA = 21;
static constexpr int PIN_IMU_SCL = 22;
static constexpr uint32_t MOTOR_BAUD = 115200;
static constexpr uint32_t IMU_BROADCAST_MS = 1000;
static constexpr uint32_t MOTOR_SEND_MS = 20;
static constexpr uint32_t MOTOR_RAMP_MS = 20;
static constexpr uint32_t MOTOR_CMD_TIMEOUT_MS = 400;
static constexpr int16_t MAX_SPEED_PERCENT = 70;
static constexpr int16_t HOVER_MAX_CMD = 300;
static constexpr int16_t INPUT_DIV = 2;
static constexpr int16_t RAMP_STEP_PER_TICK = 1;
static constexpr int16_t SLEW_SPEED_PER_SEND = 4;
static constexpr int16_t SLEW_STEER_PER_SEND = 6;
static constexpr uint16_t HOVER_START_FRAME = 0xABCD;

static constexpr uint32_t CFG_UART1INPROT_UBX = 0x10730001;
static constexpr uint32_t CFG_UART1INPROT_NMEA = 0x10730002;
static constexpr uint32_t CFG_UART1INPROT_RTCM3 = 0x10730004;
static constexpr uint32_t CFG_UART1OUTPROT_UBX = 0x10740001;
static constexpr uint32_t CFG_UART1OUTPROT_NMEA = 0x10740002;
static constexpr uint32_t CFG_UART1OUTPROT_RTCM3 = 0x10740004;
static constexpr uint32_t CFG_MSGOUT_UBX_NAV_PVT_UART1 = 0x20910007;
static constexpr uint32_t CFG_MSGOUT_UBX_RXM_RTCM_UART1 = 0x20910269;
static constexpr uint32_t CFG_RATE_MEAS = 0x30210001;
static constexpr uint32_t CFG_RATE_NAV = 0x30210002;
static constexpr uint32_t CFG_RATE_TIMEREF = 0x20210003;

struct GpsFix {
  double lat = 0.0;
  double lon = 0.0;
  float heightM = 0.0f;
  float heading = 0.0f;
  float speedMps = 0.0f;
  float pDop = 0.0f;
  uint8_t fixType = 0;
  uint8_t carrier = 0;
  bool diff = false;
  uint8_t numSv = 0;
  uint32_t hAccMm = 0;
  uint32_t vAccMm = 0;
  uint32_t lastMs = 0;
  bool valid = false;
};

static HardwareSerial GpsSerial(1);
static HardwareSerial MotorSerial(2);
static AsyncWebServer server(WS_PORT);
static AsyncWebSocket ws("/ws");
static WiFiUDP rtcmUdp;
static WiFiClient rtcmTcpClient;
static GpsFix gps;
static uint32_t rtcmBytesRx = 0;
static uint32_t rtcmPacketsRx = 0;
static uint32_t lastRtcmMs = 0;
static uint32_t lastGpsBroadcastMs = 0;
static uint32_t lastImuBroadcastMs = 0;
static uint32_t lastStatusMs = 0;
static uint32_t lastWiFiAttemptMs = 0;
static uint32_t wifiDisconnectedSinceMs = 0;
static bool wasWiFiConnected = false;
static uint32_t wifiReconnectCount = 0;
static uint32_t udpRestartCount = 0;
static uint32_t lastRtcmWarnMs = 0;
static uint32_t lastRtcmUdpRestartMs = 0;
static uint32_t lastRtcmWifiRecoverMs = 0;
static uint32_t lastLegacyBroadcastMs = 0;
static uint32_t lastRtcmTcpAttemptMs = 0;
static uint32_t lastRtcmTcpMs = 0;
static uint32_t rtcmTcpReconnectCount = 0;
static uint32_t rtcmTcpBytesRx = 0;
static uint32_t rtcmTcpReads = 0;
static uint32_t lastRoverConfigRetryMs = 0;
static uint32_t activeGpsBaud = 0;
static const char* activeGpsPort = GPS_PORTS[0].name;
static uint32_t gpsRawBytes = 0;
static uint32_t gpsParsedMessages = 0;
static uint32_t f9pRtcmMessages = 0;
static uint32_t f9pRtcmCrcFail = 0;
static uint16_t f9pLastRtcmType = 0;
static uint32_t f9pLastRtcmMs = 0;
static IPAddress lastRtcmRemoteIp(0, 0, 0, 0);
static uint16_t lastRtcmPacketSize = 0;
static const char* rtcmInputSource = "none";
static const char* gpsSource = "none";

struct HoverSerialCommand {
  uint16_t start;
  int16_t steer;
  int16_t speed;
  uint16_t checksum;
} __attribute__((packed));

static int16_t motorTargetLeft = 0;
static int16_t motorTargetRight = 0;
static int16_t motorCurrentLeft = 0;
static int16_t motorCurrentRight = 0;
static int16_t motorCmdSpeed = 0;
static int16_t motorCmdSteer = 0;
static uint32_t lastMotorCmdMs = 0;
static uint32_t lastMotorRampMs = 0;
static uint32_t lastMotorSendMs = 0;
static uint32_t motorCommandCount = 0;
static uint32_t motorStopCount = 0;
static TaskHandle_t motorTaskHandle = nullptr;

static Adafruit_BNO08x bno08x;
static sh2_SensorValue_t imuValue;
static bool imuDetected = false;
static bool imuValid = false;
static float imuYaw = 0.0f;
static uint32_t lastImuMs = 0;
static uint32_t lastImuRecoverMs = 0;

static bool imuFresh(uint32_t now) {
  return imuDetected && imuValid && lastImuMs != 0 && now - lastImuMs <= 1000;
}

static void putU32(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

static void putU16(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
}

static int32_t getI32(const uint8_t* p) {
  return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                   ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

static uint32_t getU32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint16_t getU16(const uint8_t* p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static int16_t clampI16(int32_t v, int16_t lo, int16_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return (int16_t)v;
}

static int16_t stepToward(int16_t cur, int16_t target, int16_t maxDelta) {
  int32_t diff = (int32_t)target - (int32_t)cur;
  if (diff > maxDelta) diff = maxDelta;
  if (diff < -maxDelta) diff = -maxDelta;
  return (int16_t)((int32_t)cur + diff);
}

static void writeUbx(uint8_t cls, uint8_t id, const uint8_t* payload,
                     uint16_t len) {
  uint8_t ckA = 0;
  uint8_t ckB = 0;
  auto ck = [&](uint8_t b) {
    ckA += b;
    ckB += ckA;
  };

  auto out = [&](uint8_t b) { GpsSerial.write(b); };

  out(0xB5);
  out(0x62);
  out(cls);
  ck(cls);
  out(id);
  ck(id);
  out((uint8_t)len);
  ck((uint8_t)len);
  out((uint8_t)(len >> 8));
  ck((uint8_t)(len >> 8));
  for (uint16_t i = 0; i < len; i++) {
    out(payload[i]);
    ck(payload[i]);
  }
  out(ckA);
  out(ckB);
}

struct ValItem {
  uint32_t key;
  uint32_t value;
  uint8_t size;
};

static void sendValset(const ValItem* items, size_t count) {
  uint8_t payload[160];
  size_t n = 0;
  payload[n++] = 0;
  payload[n++] = 0x01;
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

static void sendLegacyCfgPrt() {
  uint8_t payload[20] = {};
  payload[0] = 1; // UART1
  putU32(payload + 4, 0x000008D0); // 8N1
  putU32(payload + 8, activeGpsBaud == 0 ? GPS_BAUDS[0] : activeGpsBaud);
  putU16(payload + 12, 0x0023); // UBX + NMEA + RTCM3 input
  putU16(payload + 14, 0x0003); // UBX + NMEA output
  writeUbx(0x06, 0x00, payload, sizeof(payload));
  delay(80);
}

static void sendLegacyCfgMsg(uint8_t msgClass, uint8_t msgId, uint8_t rate) {
  uint8_t payload[8] = {};
  payload[0] = msgClass;
  payload[1] = msgId;
  payload[3] = rate; // UART1
  writeUbx(0x06, 0x01, payload, sizeof(payload));
  delay(80);
}

static void configureRover() {
  const ValItem portItems[] = {
      {CFG_UART1INPROT_UBX, 1, 1},
      {CFG_UART1INPROT_NMEA, 1, 1},
      {CFG_UART1INPROT_RTCM3, 1, 1},
      {CFG_UART1OUTPROT_UBX, 1, 1},
      {CFG_UART1OUTPROT_NMEA, 1, 1},
      {CFG_UART1OUTPROT_RTCM3, 0, 1},
      {CFG_MSGOUT_UBX_NAV_PVT_UART1, 1, 1},
      {CFG_MSGOUT_UBX_RXM_RTCM_UART1, 1, 1},
      {CFG_RATE_MEAS, 200, 2},
      {CFG_RATE_NAV, 1, 2},
      {CFG_RATE_TIMEREF, 1, 1},
  };
  sendValset(portItems, sizeof(portItems) / sizeof(portItems[0]));
  sendLegacyCfgPrt();
  sendLegacyCfgMsg(0x01, 0x07, 1); // NAV-PVT on UART1
  sendLegacyCfgMsg(0x02, 0x32, 1); // RXM-RTCM on UART1
  Serial.println("GNSS: rover NAV-PVT, NMEA fallback, and RTCM input configured");
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

static const char* carrierName(uint8_t carrier) {
  if (carrier == 1) return "float";
  if (carrier == 2) return "fixed";
  return "none";
}

static void parseNavPvt(const uint8_t* p, uint16_t len) {
  if (len < 92) return;
  gps.lon = (double)getI32(p + 24) * 1e-7;
  gps.lat = (double)getI32(p + 28) * 1e-7;
  gps.heightM = (float)getI32(p + 36) / 1000.0f;
  gps.hAccMm = getU32(p + 40);
  gps.vAccMm = getU32(p + 44);
  gps.speedMps = (float)getI32(p + 60) / 1000.0f;
  gps.heading = (float)getI32(p + 64) * 1e-5f;
  gps.pDop = (float)getU16(p + 76) * 0.01f;
  gps.fixType = p[20];
  gps.diff = (p[21] & 0x02) != 0;
  gps.carrier = (p[21] >> 6) & 0x03;
  gps.numSv = p[23];
  gps.valid = (p[21] & 0x01) != 0;
  gps.lastMs = millis();
  gpsParsedMessages++;
  gpsSource = "UBX";
}

static double parseNmeaCoord(const char* value, const char* hemi) {
  if (value == nullptr || hemi == nullptr || value[0] == '\0') return 0.0;

  const double raw = atof(value);
  const int degrees = (int)(raw / 100.0);
  const double minutes = raw - (double)degrees * 100.0;
  double coord = (double)degrees + minutes / 60.0;
  if (hemi[0] == 'S' || hemi[0] == 'W') coord = -coord;
  return coord;
}

static int nmeaHex(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

static bool nmeaChecksumOk(const char* s) {
  if (s == nullptr || s[0] != '$') return false;
  const char* star = strchr(s, '*');
  if (star == nullptr || star[1] == '\0' || star[2] == '\0') return false;

  uint8_t ck = 0;
  for (const char* p = s + 1; p < star; p++) ck ^= (uint8_t)*p;

  const int hi = nmeaHex(star[1]);
  const int lo = nmeaHex(star[2]);
  if (hi < 0 || lo < 0) return false;
  return ck == (uint8_t)((hi << 4) | lo);
}

static void parseNmeaSentence(char* sentence) {
  if (!nmeaChecksumOk(sentence)) return;

  char* star = strchr(sentence, '*');
  if (star != nullptr) *star = '\0';

  char* fields[24] = {};
  size_t count = 0;
  char* p = sentence + 1;
  while (count < sizeof(fields) / sizeof(fields[0])) {
    fields[count++] = p;
    char* comma = strchr(p, ',');
    if (comma == nullptr) break;
    *comma = '\0';
    p = comma + 1;
  }
  if (count == 0 || fields[0] == nullptr) return;

  if (strlen(fields[0]) < 3) return;
  const char* type = fields[0] + strlen(fields[0]) - 3;
  if (strcmp(type, "GGA") == 0 && count > 9) {
    const int quality = atoi(fields[6]);
    gps.lat = parseNmeaCoord(fields[2], fields[3]);
    gps.lon = parseNmeaCoord(fields[4], fields[5]);
    gps.fixType = quality > 0 ? 3 : 0;
    gps.diff = quality == 2 || quality == 4 || quality == 5;
    gps.carrier = quality == 4 ? 2 : (quality == 5 ? 1 : 0);
    gps.numSv = (uint8_t)atoi(fields[7]);
    gps.pDop = (float)atof(fields[8]);
    gps.heightM = (float)atof(fields[9]);
    gps.valid = quality > 0;
    gps.lastMs = millis();
    gpsParsedMessages++;
    gpsSource = "NMEA";
    return;
  }

  if (strcmp(type, "RMC") == 0 && count > 8) {
    gps.valid = fields[2][0] == 'A';
    gps.lat = parseNmeaCoord(fields[3], fields[4]);
    gps.lon = parseNmeaCoord(fields[5], fields[6]);
    gps.speedMps = (float)(atof(fields[7]) * 0.514444);
    gps.heading = (float)atof(fields[8]);
    if (!gps.valid) gps.fixType = 0;
    gps.lastMs = millis();
    gpsParsedMessages++;
    gpsSource = "NMEA";
  }
}

static void feedNmea(uint8_t b) {
  static char nmea[128];
  static uint8_t n = 0;
  static bool collecting = false;

  if (b == '$') {
    collecting = true;
    n = 0;
    nmea[n++] = (char)b;
    return;
  }

  if (!collecting) return;

  if (b == '\r') return;
  if (b == '\n') {
    nmea[n] = '\0';
    collecting = false;
    parseNmeaSentence(nmea);
    return;
  }

  if (n < sizeof(nmea) - 1) {
    nmea[n++] = (char)b;
  } else {
    collecting = false;
    n = 0;
  }
}

static void processUbx() {
  if (ubxClass == 0x01 && ubxId == 0x07) {
    parseNavPvt(ubxPayload, ubxLen);
    return;
  }

  if (ubxClass == 0x02 && ubxId == 0x32 && ubxLen >= 8) {
    f9pRtcmMessages++;
    if ((ubxPayload[1] & 0x01) != 0) f9pRtcmCrcFail++;
    f9pLastRtcmType = getU16(ubxPayload + 6);
    f9pLastRtcmMs = millis();
  }
}

static void feedUbx(uint8_t b);

static bool waitForPvt(uint32_t timeoutMs, uint32_t* bytesSeen) {
  const uint32_t started = millis();
  const uint32_t previousGpsMs = gps.lastMs;
  uint32_t bytes = 0;

  while (millis() - started < timeoutMs) {
    while (GpsSerial.available()) {
      bytes++;
      gpsRawBytes++;
      const uint8_t b = (uint8_t)GpsSerial.read();
      feedUbx(b);
      feedNmea(b);
    }
    if (gps.lastMs != 0 && gps.lastMs != previousGpsMs) {
      if (bytesSeen != nullptr) *bytesSeen = bytes;
      return true;
    }
    delay(2);
  }

  if (bytesSeen != nullptr) *bytesSeen = bytes;
  return false;
}

static bool startGpsAtConfig(const GpsPortConfig& port, uint32_t baud) {
  Serial.printf("GNSS: trying UART1 %s baud %lu\n", port.name,
                (unsigned long)baud);
  GpsSerial.end();
  delay(80);
  GpsSerial.begin(baud, SERIAL_8N1, port.rx, port.tx);
  delay(300);
  configureRover();

  uint32_t bytesSeen = 0;
  if (waitForPvt(2200, &bytesSeen)) {
    activeGpsBaud = baud;
    activeGpsPort = port.name;
    Serial.printf("GNSS: %s detected on %s at %lu baud, raw=%lu parsed=%lu\n",
                  gpsSource, port.name, (unsigned long)baud,
                  (unsigned long)bytesSeen, (unsigned long)gpsParsedMessages);
    return true;
  }

  Serial.printf("GNSS: no GPS parse on %s at %lu baud, raw=%lu\n", port.name,
                (unsigned long)baud, (unsigned long)bytesSeen);
  return false;
}

static void autoDetectGpsBaud() {
  const GpsPortConfig* bestPort = &GPS_PORTS[0];
  uint32_t bestBaud = GPS_BAUDS[0];
  uint32_t bestRaw = 0;
  const uint32_t rawBefore = gpsRawBytes;

  for (const GpsPortConfig& port : GPS_PORTS) {
    for (uint32_t baud : GPS_BAUDS) {
      const uint32_t before = gpsRawBytes;
      if (startGpsAtConfig(port, baud)) {
        return;
      }
      const uint32_t rawDelta = gpsRawBytes - before;
      if (rawDelta > bestRaw) {
        bestRaw = rawDelta;
        bestPort = &port;
        bestBaud = baud;
      }
    }
  }

  activeGpsBaud = bestBaud;
  activeGpsPort = bestPort->name;
  GpsSerial.end();
  delay(80);
  GpsSerial.begin(activeGpsBaud, SERIAL_8N1, bestPort->rx, bestPort->tx);
  delay(300);
  configureRover();
  Serial.printf("GNSS: GPS messages not detected, staying on %s at %lu baud, total raw=%lu\n",
                activeGpsPort, (unsigned long)activeGpsBaud,
                (unsigned long)(gpsRawBytes - rawBefore));
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

static bool relayRtcmToF9p() {
  uint8_t buf[1200];
  uint8_t packetsHandled = 0;

  while (packetsHandled < 24) {
    int packetSize = rtcmUdp.parsePacket();
    if (packetSize <= 0) return packetsHandled > 0;

    lastRtcmRemoteIp = rtcmUdp.remoteIP();
    lastRtcmPacketSize = (uint16_t)packetSize;
    int remaining = packetSize;
    int written = 0;
    while (remaining > 0) {
      const int chunkSize =
          remaining > (int)sizeof(buf) ? (int)sizeof(buf) : remaining;
      const int len = rtcmUdp.read(buf, chunkSize);
      if (len <= 0) break;
      GpsSerial.write(buf, len);
      written += len;
      remaining -= len;
    }

    if (written > 0) {
      rtcmBytesRx += written;
      rtcmPacketsRx++;
      lastRtcmMs = millis();
      rtcmInputSource = "udp";
    }
    packetsHandled++;
  }
  return packetsHandled > 0;
}

static void connectWiFi(bool force = false);

static void connectRtcmTcp() {
  if (!ROUTER_MODE || WiFi.status() != WL_CONNECTED) return;
  if (rtcmTcpClient.connected()) return;

  const uint32_t now = millis();
  if (lastRtcmTcpAttemptMs != 0 &&
      now - lastRtcmTcpAttemptMs < WIFI_RETRY_MS) {
    return;
  }
  lastRtcmTcpAttemptMs = now;
  rtcmTcpReconnectCount++;

  rtcmTcpClient.stop();
  Serial.printf("RTCM TCP connect #%lu to %s:%u\n",
                (unsigned long)rtcmTcpReconnectCount,
                BASE_STA_IP.toString().c_str(), RTCM_TCP_PORT);
  rtcmTcpClient.setTimeout(RTCM_TCP_CONNECT_TIMEOUT_MS);
  if (rtcmTcpClient.connect(BASE_STA_IP, RTCM_TCP_PORT,
                            RTCM_TCP_CONNECT_TIMEOUT_MS)) {
    rtcmTcpClient.setNoDelay(true);
    Serial.println("RTCM TCP connected");
  } else {
    Serial.println("WARN RTCM TCP connect failed");
    rtcmTcpClient.stop();
  }
}

static void relayTcpRtcmToF9p() {
  if (!rtcmTcpClient.connected()) return;

  uint8_t buf[512];
  uint8_t reads = 0;
  while (rtcmTcpClient.available() > 0 && reads < 24) {
    const int len = rtcmTcpClient.read(buf, sizeof(buf));
    if (len <= 0) break;
    GpsSerial.write(buf, len);
    rtcmBytesRx += len;
    rtcmTcpBytesRx += len;
    rtcmPacketsRx++;
    rtcmTcpReads++;
    lastRtcmMs = millis();
    lastRtcmTcpMs = lastRtcmMs;
    rtcmInputSource = "tcp";
    lastRtcmRemoteIp = BASE_STA_IP;
    lastRtcmPacketSize = (uint16_t)len;
    reads++;
  }
}

static void restartRtcmUdp(const char* reason) {
  rtcmUdp.stop();
  delay(20);
  const uint8_t ok = rtcmUdp.begin(RTCM_UDP_PORT);
  udpRestartCount++;
  lastRtcmUdpRestartMs = millis();
  Serial.printf("RTCM UDP restart #%lu port=%u ok=%u reason=%s\n",
                (unsigned long)udpRestartCount, RTCM_UDP_PORT, ok, reason);
}

static void recoverRtcmWiFi(const char* reason) {
  const uint32_t now = millis();
  if (now - lastRtcmWifiRecoverMs < RTCM_WIFI_RECOVER_MS) return;
  lastRtcmWifiRecoverMs = now;
  Serial.printf("WARN rover RTCM transport recover reason=%s age=%lums\n", reason,
                (unsigned long)(lastRtcmMs == 0 ? 0 : now - lastRtcmMs));
  rtcmTcpClient.stop();
  lastRtcmTcpAttemptMs = 0;
  restartRtcmUdp(reason);
}

static void checkRtcmWatchdog() {
  const uint32_t now = millis();
  const uint32_t referenceMs =
      lastRtcmMs != 0 ? lastRtcmMs : lastRtcmUdpRestartMs;
  if (referenceMs == 0) return;

  const uint32_t age = now - referenceMs;
  if (age > RTCM_WARN_AGE_MS && now - lastRtcmWarnMs > 1000) {
    lastRtcmWarnMs = now;
    Serial.printf("WARN rover RTCM age=%lums packets=%lu udpRestart=%lu seen=%u\n",
                  (unsigned long)age, (unsigned long)rtcmPacketsRx,
                  (unsigned long)udpRestartCount, lastRtcmMs != 0 ? 1 : 0);
  }
  if (age > RTCM_RESTART_AGE_MS &&
      now - lastRtcmUdpRestartMs > WIFI_RETRY_MS) {
    restartRtcmUdp("rtcm-age");
  }
  if (WiFi.status() != WL_CONNECTED && lastRtcmMs != 0 &&
      age > RTCM_WIFI_RECOVER_AGE_MS) {
    recoverRtcmWiFi("rtcm-stale");
  }
}

static void checkF9pRtcmWatchdog() {
  const uint32_t now = millis();
  if (gpsParsedMessages == 0) return;
  if (gps.diff || gps.carrier != 0) return;
  if (rtcmPacketsRx < 10 || f9pRtcmMessages > 0) return;
  if (now - lastRoverConfigRetryMs < 10000) return;
  lastRoverConfigRetryMs = now;
  Serial.printf("WARN rover UDP RTCM is fresh but F9P reports no RTCM; reconfiguring UART protocols packets=%lu bytes=%lu\n",
                (unsigned long)rtcmPacketsRx, (unsigned long)rtcmBytesRx);
  configureRover();
}

static void broadcastGps() {
  const uint32_t now = millis();
  if (now - lastGpsBroadcastMs < GPS_BROADCAST_MS) return;
  lastGpsBroadcastMs = now;

  const uint32_t gpsAgeMs = gps.lastMs == 0 ? 0 : now - gps.lastMs;
  const uint32_t rtcmAgeMs = lastRtcmMs == 0 ? 0 : now - lastRtcmMs;
  const uint32_t imuAgeMs = lastImuMs == 0 ? 0 : now - lastImuMs;
  const bool freshImu = imuFresh(now);

  char tel[256];
  snprintf(tel, sizeof(tel),
           "TEL,%.8f,%.8f,%.3f,%.2f,%u,%s,%u,%u,%lu,%lu,%.3f,%.2f,%lu,%lu,%lu,%.2f,%lu,%u",
           gps.lat, gps.lon, gps.heightM, gps.heading, gps.fixType,
           carrierName(gps.carrier), gps.diff ? 1 : 0, gps.numSv,
           (unsigned long)gps.hAccMm, (unsigned long)gps.vAccMm,
           gps.speedMps, gps.pDop, (unsigned long)gpsAgeMs,
           (unsigned long)rtcmBytesRx, (unsigned long)rtcmAgeMs,
           imuYaw, (unsigned long)imuAgeMs, freshImu ? 1 : 0);
  ws.textAll(tel);

  if (now - lastLegacyBroadcastMs < LEGACY_BROADCAST_MS) return;
  lastLegacyBroadcastMs = now;

  char msg[192];
  snprintf(msg, sizeof(msg), "GPS,%.8f,%.8f,%.2f,%u,%lu",
           gps.lat, gps.lon, gps.heading, gps.fixType,
           (unsigned long)gps.hAccMm);
  ws.textAll(msg);

  snprintf(msg, sizeof(msg),
           "GPSDBG,%.8f,%.8f,%.3f,%.2f,%u,%s,%u,%u,%lu,%lu,%.3f,%.2f,%lu",
           gps.lat, gps.lon, gps.heightM, gps.heading, gps.fixType,
           carrierName(gps.carrier), gps.diff ? 1 : 0, gps.numSv,
           (unsigned long)gps.hAccMm, (unsigned long)gps.vAccMm,
           gps.speedMps, gps.pDop, (unsigned long)gpsAgeMs);
  ws.textAll(msg);

  snprintf(msg, sizeof(msg), "RTCM,%lu,%lu",
           (unsigned long)rtcmBytesRx, (unsigned long)rtcmAgeMs);
  ws.textAll(msg);
}

static void connectWiFi(bool force) {
  if (!ROUTER_MODE) return;
  if (!force && WiFi.status() == WL_CONNECTED) return;

  const uint32_t now = millis();
  if (!force && lastWiFiAttemptMs != 0 &&
      now - lastWiFiAttemptMs < WIFI_RETRY_MS) {
    return;
  }
  lastWiFiAttemptMs = now;
  wifiReconnectCount++;

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.config(ROVER_STA_IP, ROUTER_GATEWAY, ROUTER_SUBNET);
  if (force) {
    rtcmTcpClient.stop();
    WiFi.disconnect(false);
    delay(50);
  }
  WiFi.begin(ROUTER_WIFI_SSID, ROUTER_WIFI_PASS);
  Serial.printf("WiFi STA reconnect #%lu to %s, static IP %s\n",
                (unsigned long)wifiReconnectCount, ROUTER_WIFI_SSID,
                ROVER_STA_IP.toString().c_str());
}

static void checkWiFiWatchdog() {
  if (!ROUTER_MODE) return;

  const uint32_t now = millis();
  const bool connected = WiFi.status() == WL_CONNECTED;

  if (connected) {
    if (!wasWiFiConnected) {
      Serial.printf("WiFi STA connected ip=%s, reopening UDP/WebSocket\n",
                    WiFi.localIP().toString().c_str());
      restartRtcmUdp("wifi-connected");
      ws.closeAll();
    }
    wifiDisconnectedSinceMs = 0;
    wasWiFiConnected = true;
    return;
  }

  if (wifiDisconnectedSinceMs == 0) wifiDisconnectedSinceMs = now;
  wasWiFiConnected = false;

  if (now - wifiDisconnectedSinceMs >= WIFI_RETRY_MS &&
      (lastWiFiAttemptMs == 0 || now - lastWiFiAttemptMs >= WIFI_RETRY_MS)) {
    connectWiFi(true);
  } else {
    connectWiFi(false);
  }
}

static void motorsInit() {
  MotorSerial.begin(MOTOR_BAUD, SERIAL_8N1, PIN_MOTOR_RX, PIN_MOTOR_TX);
  motorTargetLeft = 0;
  motorTargetRight = 0;
  motorCurrentLeft = 0;
  motorCurrentRight = 0;
  motorCmdSpeed = 0;
  motorCmdSteer = 0;
  lastMotorCmdMs = millis();
  lastMotorRampMs = millis();
  lastMotorSendMs = millis();
  Serial.printf("MOTORS: UART2 baud=%lu RX=%d TX=%d\n",
                (unsigned long)MOTOR_BAUD, PIN_MOTOR_RX, PIN_MOTOR_TX);
}

static void motorsStop(const char* reason) {
  motorTargetLeft = 0;
  motorTargetRight = 0;
  lastMotorCmdMs = millis();
  motorStopCount++;
  if (reason != nullptr && reason[0] != '\0') {
    Serial.printf("MOTORS: stop %s\n", reason);
  }
}

static void motorsSetTarget(int left, int right) {
  motorTargetLeft =
      clampI16(left / INPUT_DIV, -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);
  motorTargetRight =
      clampI16(right / INPUT_DIV, -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);
  lastMotorCmdMs = millis();
  motorCommandCount++;
}

static void motorsCheckFailsafe() {
  const uint32_t now = millis();
  if (now - lastMotorCmdMs <= MOTOR_CMD_TIMEOUT_MS) return;
  if (motorTargetLeft != 0 || motorTargetRight != 0) {
    motorTargetLeft = 0;
    motorTargetRight = 0;
    Serial.println("MOTORS: command timeout, stopping targets");
  }
}

static void motorsUpdateRamp() {
  const uint32_t now = millis();
  const uint32_t dt = now - lastMotorRampMs;
  if (dt < MOTOR_RAMP_MS) return;
  const uint32_t ticks = dt / MOTOR_RAMP_MS;
  lastMotorRampMs += ticks * MOTOR_RAMP_MS;
  uint32_t delta = ticks * RAMP_STEP_PER_TICK;
  if (delta < 1) delta = 1;
  const int16_t maxDelta = (int16_t)delta;

  motorCurrentLeft = stepToward(motorCurrentLeft, motorTargetLeft, maxDelta);
  motorCurrentRight = stepToward(motorCurrentRight, motorTargetRight, maxDelta);
  motorCurrentLeft =
      clampI16(motorCurrentLeft, -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);
  motorCurrentRight =
      clampI16(motorCurrentRight, -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);
}

static void motorsSend() {
  const uint32_t now = millis();
  if (now - lastMotorSendMs < MOTOR_SEND_MS) return;
  lastMotorSendMs = now;

  const int16_t left =
      clampI16(motorCurrentLeft, -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);
  const int16_t right =
      clampI16(motorCurrentRight, -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);
  const int32_t speedTarget =
      (int32_t)(left + right) * HOVER_MAX_CMD / (2 * MAX_SPEED_PERCENT);
  const int32_t steerTarget =
      (int32_t)(right - left) * HOVER_MAX_CMD / (2 * MAX_SPEED_PERCENT);

  motorCmdSpeed = stepToward(
      motorCmdSpeed, clampI16(speedTarget, -HOVER_MAX_CMD, HOVER_MAX_CMD),
      SLEW_SPEED_PER_SEND);
  motorCmdSteer = stepToward(
      motorCmdSteer, clampI16(steerTarget, -HOVER_MAX_CMD, HOVER_MAX_CMD),
      SLEW_STEER_PER_SEND);

  HoverSerialCommand cmd{};
  cmd.start = HOVER_START_FRAME;
  cmd.steer = motorCmdSteer;
  cmd.speed = motorCmdSpeed;
  cmd.checksum = (uint16_t)(cmd.start ^ cmd.steer ^ cmd.speed);
  MotorSerial.write((uint8_t*)&cmd, sizeof(cmd));
}

static void motorTask(void*) {
  for (;;) {
    motorsCheckFailsafe();
    motorsUpdateRamp();
    motorsSend();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

static void startMotorTask() {
  if (motorTaskHandle != nullptr) return;
  xTaskCreatePinnedToCore(
      motorTask,
      "motorTask",
      4096,
      nullptr,
      2,
      &motorTaskHandle,
      0);
  Serial.println("MOTORS: heartbeat task started");
}

static bool parseMoveCommand(const String& msg, int* left, int* right) {
  if (!msg.startsWith("M,")) return false;
  const int comma = msg.indexOf(',', 2);
  if (comma < 0) return false;
  *left = msg.substring(2, comma).toInt();
  *right = msg.substring(comma + 1).toInt();
  return true;
}

static void imuInit() {
  Wire.begin(PIN_IMU_SDA, PIN_IMU_SCL);
  Serial.printf("IMU: I2C SDA=%d SCL=%d\n", PIN_IMU_SDA, PIN_IMU_SCL);

  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("IMU: I2C device found at 0x%02X\n", addr);
      found++;
    }
  }
  if (found == 0) {
    Serial.println("IMU: I2C scan found no devices");
  }

  if (!bno08x.begin_I2C(0x4A, &Wire) && !bno08x.begin_I2C(0x4B, &Wire)) {
    imuDetected = false;
    imuValid = false;
    Serial.println("IMU: BNO085 not found at 0x4A/0x4B");
    return;
  }
  imuDetected = true;
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 20000)) {
    imuValid = false;
    Serial.println("IMU: BNO085 game rotation vector enable failed");
    return;
  }
  imuValid = true;
  lastImuMs = millis();
  Serial.println("IMU: BNO085 found, game rotation vector 50Hz");
}

static void imuReset(const char* reason) {
  Serial.printf("IMU: reset requested reason=%s\n", reason);
  imuDetected = false;
  imuValid = false;
  lastImuMs = 0;
  lastImuRecoverMs = 0;
  Wire.end();
  delay(80);
  imuInit();
}

static void imuUpdate() {
  if (!imuDetected) return;
  if (bno08x.wasReset()) {
    Serial.println("IMU: BNO085 reset, re-enabling game rotation vector");
    imuValid = bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 20000);
    if (!imuValid) return;
  }

  while (bno08x.getSensorEvent(&imuValue)) {
    if (imuValue.sensorId != SH2_GAME_ROTATION_VECTOR &&
        imuValue.sensorId != SH2_ROTATION_VECTOR) {
      continue;
    }
    const float qw = imuValue.un.rotationVector.real;
    const float qx = imuValue.un.rotationVector.i;
    const float qy = imuValue.un.rotationVector.j;
    const float qz = imuValue.un.rotationVector.k;
    float yaw = atan2(2.0f * (qw * qz + qx * qy),
                      1.0f - 2.0f * (qy * qy + qz * qz)) *
                180.0f / PI;
    if (yaw < 0) yaw += 360.0f;
    imuYaw = yaw;
    imuValid = true;
    lastImuMs = millis();
  }

  const uint32_t now = millis();
  if (lastImuMs != 0 && now >= lastImuMs && now - lastImuMs > 1500 &&
      now - lastImuRecoverMs > 1500) {
    lastImuRecoverMs = now;
    Serial.printf("WARN IMU stale age=%lums, re-enabling game rotation vector\n",
                  (unsigned long)(now - lastImuMs));
    imuValid = bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 20000);
  }
}

static void broadcastImu() {
  const uint32_t now = millis();
  if (now - lastImuBroadcastMs < IMU_BROADCAST_MS) return;
  lastImuBroadcastMs = now;
  if (!imuFresh(now)) return;
  const uint32_t imuAgeMs = now - lastImuMs;
  char msg[48];
  snprintf(msg, sizeof(msg), "IMU,%.2f,%lu,1", imuYaw,
           (unsigned long)imuAgeMs);
  ws.textAll(msg);
}

static void setupWeb() {
  if (ROUTER_MODE) {
    connectWiFi();
    Serial.printf("Router mode: WebSocket will be ws://%s:%u/ws\n",
                  ROVER_STA_IP.toString().c_str(), WS_PORT);
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_WIFI_SSID, AP_WIFI_PASS);
    Serial.printf("AP %s IP %s\n", AP_WIFI_SSID,
                  WiFi.softAPIP().toString().c_str());
  }

  ws.onEvent([](AsyncWebSocket*, AsyncWebSocketClient* client, AwsEventType type,
                void*, uint8_t* data, size_t len) {
    if (type == WS_EVT_CONNECT) {
      client->text("STATE,CONNECTED");
      return;
    }
    if (type == WS_EVT_DISCONNECT) {
      motorsStop("websocket disconnect");
      return;
    }
    if (type != WS_EVT_DATA || len == 0) return;

    String msg;
    msg.reserve(len);
    for (size_t i = 0; i < len; i++) msg += (char)data[i];
    msg.trim();
    msg.toUpperCase();

    if (msg == "PING") {
      client->text("PONG");
    } else if (msg == "STOP") {
      motorsStop("ws STOP");
      client->text("OK STOP");
    } else if (msg == "UDP_RESET") {
      restartRtcmUdp("ws UDP_RESET");
      client->text("OK UDP_RESET");
    } else if (msg == "IMU_RESET") {
      client->text("OK IMU_RESET");
      imuReset("ws IMU_RESET");
    } else if (msg == "IMU_STATUS") {
      char status[96];
      const uint32_t now = millis();
      const uint32_t imuAgeMs = lastImuMs == 0 ? 0 : now - lastImuMs;
      snprintf(status, sizeof(status),
               "IMU_STATUS,detected=%d,valid=%d,fresh=%d,age=%lu,yaw=%.2f",
               imuDetected ? 1 : 0,
               imuValid ? 1 : 0,
               imuFresh(now) ? 1 : 0,
               (unsigned long)imuAgeMs,
               imuYaw);
      client->text(status);
    } else if (msg == "STATUS") {
      char status[192];
      const uint32_t now = millis();
      const uint32_t rtcmAgeMs = lastRtcmMs == 0 ? 0 : now - lastRtcmMs;
      const uint32_t imuAgeMs = lastImuMs == 0 ? 0 : now - lastImuMs;
      snprintf(status, sizeof(status),
               "STATUS,wifi=%d,udpRestart=%lu,rtcmBytes=%lu,rtcmAge=%lu,imu=%d,imuAge=%lu,yaw=%.2f,motor=%d/%d",
               WiFi.status(),
               (unsigned long)udpRestartCount,
               (unsigned long)rtcmBytesRx,
               (unsigned long)rtcmAgeMs,
               imuFresh(now) ? 1 : 0,
               (unsigned long)imuAgeMs,
               imuYaw,
               motorTargetLeft,
               motorTargetRight);
      client->text(status);
    } else if (msg == "ESP_RESTART") {
      client->text("OK ESP_RESTART");
      delay(50);
      ESP.restart();
    } else if (msg == "GPS_STATUS") {
      client->text(gps.valid ? "OK GPS" : "NO GPS");
    } else {
      int left = 0;
      int right = 0;
      if (parseMoveCommand(msg, &left, &right)) {
        motorsSetTarget(left, right);
      } else {
        client->text("ERR,UNKNOWN");
      }
    }
  });

  server.addHandler(&ws);
  server.on("/ping", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send(200, "text/plain", "OK");
  });
  server.begin();
}

static void setupOta() {
  ArduinoOTA.setHostname("rtk-rover");
  ArduinoOTA.onStart([]() {
    motorsStop("OTA start");
    Serial.println("OTA: start");
  });
  ArduinoOTA.onEnd([]() { Serial.println("OTA: end"); });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA: error %u\n", (unsigned)error);
  });
  ArduinoOTA.begin();
  Serial.println("OTA: enabled hostname=rtk-rover");
}

static void printStatus() {
  const uint32_t now = millis();
  if (now - lastStatusMs < STATUS_MS) return;
  lastStatusMs = now;

  const uint32_t gpsAgeMs = gps.lastMs == 0 ? 0 : now - gps.lastMs;
  const uint32_t rtcmAgeMs = lastRtcmMs == 0 ? 0 : now - lastRtcmMs;
  const uint32_t f9pRtcmAgeMs =
      f9pLastRtcmMs == 0 ? 0 : now - f9pLastRtcmMs;
  const uint32_t imuAgeMs = lastImuMs == 0 ? 0 : now - lastImuMs;
  Serial.printf("ROVER wifi=%s ip=%s wifiReconnect=%lu udpRestart=%lu tcpReconnect=%lu tcpBytes=%lu tcpReads=%lu port=%s baud=%lu src=%s parsed=%lu clients=%u fix=%u carrier=%s diff=%u sv=%u hAcc=%lumm gpsAge=%lums raw=%lu rtcmSrc=%s rtcm=%lubytes/%lupkts age=%lums from=%s size=%u f9pRtcm=%lu crcFail=%lu lastType=%u age=%lums imu=%s yaw=%.2f age=%lums motorTarget=%d/%d motorCur=%d/%d motorCmd=%d/%d motorCount=%lu stopCount=%lu\n",
                WiFi.status() == WL_CONNECTED ? "connected" : "not_connected",
                WiFi.localIP().toString().c_str(),
                (unsigned long)wifiReconnectCount,
                (unsigned long)udpRestartCount,
                (unsigned long)rtcmTcpReconnectCount,
                (unsigned long)rtcmTcpBytesRx,
                (unsigned long)rtcmTcpReads,
                activeGpsPort,
                (unsigned long)activeGpsBaud,
                gpsSource,
                (unsigned long)gpsParsedMessages,
                ws.count(),
                gps.fixType,
                carrierName(gps.carrier),
                gps.diff ? 1 : 0,
                gps.numSv,
                (unsigned long)gps.hAccMm,
                (unsigned long)gpsAgeMs,
                (unsigned long)gpsRawBytes,
                rtcmInputSource,
                (unsigned long)rtcmBytesRx,
                (unsigned long)rtcmPacketsRx,
                (unsigned long)rtcmAgeMs,
                lastRtcmRemoteIp.toString().c_str(),
                lastRtcmPacketSize,
                (unsigned long)f9pRtcmMessages,
                (unsigned long)f9pRtcmCrcFail,
                f9pLastRtcmType,
                (unsigned long)f9pRtcmAgeMs,
                (!imuDetected
                     ? "not_found"
                     : (imuValid && imuAgeMs <= 1000 ? "ok" : "stale")),
                imuYaw,
                (unsigned long)imuAgeMs,
                motorTargetLeft,
                motorTargetRight,
                motorCurrentLeft,
                motorCurrentRight,
                motorCmdSpeed,
                motorCmdSteer,
                (unsigned long)motorCommandCount,
                (unsigned long)motorStopCount);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("ESP32 ZED-F9P RTK ROVER");

  motorsInit();
  startMotorTask();
  imuInit();
  autoDetectGpsBaud();
  setupWeb();
  setupOta();
  restartRtcmUdp("setup");
}

void loop() {
  ArduinoOTA.handle();
  checkWiFiWatchdog();
  connectRtcmTcp();
  imuUpdate();
  relayTcpRtcmToF9p();
  relayRtcmToF9p();
  while (GpsSerial.available()) {
    gpsRawBytes++;
    const uint8_t b = (uint8_t)GpsSerial.read();
    feedUbx(b);
    feedNmea(b);
  }

  broadcastGps();
  broadcastImu();
  checkRtcmWatchdog();
  checkF9pRtcmWatchdog();
  printStatus();
  ws.cleanupClients();
}
