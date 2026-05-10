/**
 * RTK Rover Autopilot
 * Architecture:
 *   - GPS Input Layer (UBX-NAV-PVT)
 *   - Local Coordinate System (origin + local x/y meters)
 *   - Navigation State Machine (6 quality states)
 *   - Pure Pursuit Route Follower
 *   - Dead Reckoning for GPS gaps
 *   - Motor Mixer with quality-based speed limiting
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Preferences.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Adafruit_BNO08x.h>

#if __has_include("rtk_config_private.h")
#include "rtk_config_private.h"
#else
#include "rtk_config.example.h"
#endif

// ============== CONFIGURATION ==============

// Wi-Fi
static constexpr char WIFI_SSID[] = RTK_ROUTER_WIFI_SSID;
static constexpr char WIFI_PASS[] = RTK_ROUTER_WIFI_PASS;

#ifndef RTK_ROVER_IP_A
#define RTK_ROVER_IP_A 192
#define RTK_ROVER_IP_B 168
#define RTK_ROVER_IP_C 31
#define RTK_ROVER_IP_D 222
#endif
#ifndef RTK_BASE_IP_A
#define RTK_BASE_IP_A 192
#define RTK_BASE_IP_B 168
#define RTK_BASE_IP_C 31
#define RTK_BASE_IP_D 207
#endif
#ifndef RTK_ROUTER_GATEWAY_A
#define RTK_ROUTER_GATEWAY_A 192
#define RTK_ROUTER_GATEWAY_B 168
#define RTK_ROUTER_GATEWAY_C 31
#define RTK_ROUTER_GATEWAY_D 1
#endif

static const IPAddress ROVER_IP(RTK_ROVER_IP_A, RTK_ROVER_IP_B, RTK_ROVER_IP_C, RTK_ROVER_IP_D);
static const IPAddress GATEWAY(RTK_ROUTER_GATEWAY_A, RTK_ROUTER_GATEWAY_B, RTK_ROUTER_GATEWAY_C, RTK_ROUTER_GATEWAY_D);
static const IPAddress SUBNET(255, 255, 255, 0);

// Network
static constexpr uint16_t WS_PORT = 81;
static constexpr uint16_t RTCM_UDP_PORT = 2101;

// GPS
static constexpr int GPS_RX = 4;
static constexpr int GPS_TX = 5;
static constexpr uint32_t GPS_BAUD = 38400;

// Motor
static constexpr int MOTOR_RX = 16;
static constexpr int MOTOR_TX = 17;
static constexpr uint32_t MOTOR_BAUD = 115200;

// IMU
static constexpr int IMU_SDA = 21;
static constexpr int IMU_SCL = 22;

// Navigation Quality Thresholds
static constexpr uint32_t RTK_FIXED_AGE_MS = 500;
static constexpr uint32_t RTK_FLOAT_AGE_MS = 1000;
static constexpr uint32_t GPS_HOLD_AGE_MS = 2000;
static constexpr uint32_t GPS_DEAD_AGE_MS = 5000;
static constexpr uint32_t IMU_TIMEOUT_MS = 2000;
static constexpr uint32_t RTK_CORRECTION_TIMEOUT_MS = 30000;

// Accuracy thresholds (mm)
static constexpr uint32_t RTK_FIXED_HACC_MM = 50;
static constexpr uint32_t RTK_FLOAT_HACC_MM = 300;
static constexpr uint32_t DEGRADED_HACC_MM = 1500;

// Speed settings (m/s)
static constexpr float MAX_SPEED = 0.5f;
static constexpr float FLOAT_SPEED = 0.25f;
static constexpr float DEGRADED_SPEED = 0.15f;
static constexpr float HOLD_SPEED = 0.05f;

// Timing
static constexpr uint32_t NAV_LOOP_MS = 50;
static constexpr uint32_t MOTOR_SEND_MS = 20;
static constexpr uint32_t STATUS_MS = 5000;
static constexpr uint32_t TELEMETRY_MS = 500;
static constexpr uint32_t MANUAL_CMD_TIMEOUT_MS = 400;

// Motor control - same as sound.ino
static constexpr int MAX_SPEED_PERCENT = 70;     // -70..70 (after input divide)
static constexpr int16_t HOVER_MAX_CMD = 300;    // hoverboard command scale
static constexpr int INPUT_DIV = 2;             // app values / 2 (calmer)
static constexpr uint32_t HOVER_SEND_MS = 20;
static constexpr uint32_t CMD_TIMEOUT_MS = 400;

// ramp (percent domain)
static constexpr uint32_t RAMP_UPDATE_MS = 20;
static constexpr int RAMP_STEP_UP_PER_TICK = 1;
static constexpr int RAMP_STEP_DOWN_PER_TICK = 1;

// extra smoothing in hoverboard command domain
static constexpr int16_t SLEW_SPEED_PER_SEND = 4;
static constexpr int16_t SLEW_STEER_PER_SEND = 6;

// Motor state
int16_t g_cmdSpeed = 0;
int16_t g_cmdSteer = 0;
volatile int16_t g_targetLeft = 0, g_targetRight = 0;
int16_t g_curLeft = 0, g_curRight = 0;
volatile uint32_t g_lastCmdMs = 0;
uint32_t g_lastSendMs = 0;
uint32_t g_lastRampMs = 0;
bool g_isFailSafeStopping = false;

static Preferences g_prefs;
static void saveNavPrefs();

// ============== UTILITIES ==============

static inline float degToRad(float d) { return d * PI / 180.0f; }
static inline float radToDeg(float r) { return r * 180.0f / PI; }

static inline float normalizeAngle(float a) {
  while (a > 180) a -= 360;
  while (a < -180) a += 360;
  return a;
}

static inline float normalizeAngle360(float a) {
  while (a >= 360) a -= 360;
  while (a < 0) a += 360;
  return a;
}

static int16_t stepToward(int16_t cur, int16_t target, int16_t maxStep) {
  int16_t diff = target - cur;
  if (diff > maxStep) diff = maxStep;
  if (diff < -maxStep) diff = -maxStep;
  return cur + diff;
}

// ============== COORDINATE SYSTEM ==============

struct LocalCoords {
  float x;  // east, meters
  float y;  // north, meters
};

struct Origin {
  double lat = 0, lon = 0;
  float mPerDegLat = 111132.92f;
  float mPerDegLon = 111412.84f;
  bool valid = false;
};

static Origin g_origin;

static void setOrigin(double lat, double lon) {
  g_origin.lat = lat;
  g_origin.lon = lon;
  const double latRad = lat * PI / 180.0;
  g_origin.mPerDegLat = 111132.92f - 559.82f * cos(2 * latRad) + 1.175f * cos(4 * latRad);
  g_origin.mPerDegLon = 111412.84f * cos(latRad) - 93.5f * cos(3 * latRad);
  g_origin.valid = true;
  Serial.printf("ORIGIN: lat=%.8f lon=%.8f m/deg=%.2f,%.2f\n",
    lat, lon, g_origin.mPerDegLat, g_origin.mPerDegLon);
}

static LocalCoords toLocal(double lat, double lon) {
  if (!g_origin.valid) return {0, 0};
  return {
    (float)((lon - g_origin.lon) * g_origin.mPerDegLon),
    (float)((lat - g_origin.lat) * g_origin.mPerDegLat)
  };
}

// ============== NAVIGATION STATE MACHINE ==============

enum NavQuality {
  QUAL_RTK_FIXED_GOOD,
  QUAL_RTK_FLOAT_OK,
  QUAL_GPS_DEGRADED,
  QUAL_GPS_HOLD_SHORT,
  QUAL_GPS_LOST_WAIT,
  QUAL_NAV_ERROR
};

enum NavState {
  STATE_IDLE,
  STATE_RUNNING,
  STATE_PAUSED,
  STATE_ARRIVED,
  STATE_ERROR
};

static const char* qualityString(NavQuality q) {
  switch (q) {
    case QUAL_RTK_FIXED_GOOD: return "RTK_FIXED";
    case QUAL_RTK_FLOAT_OK: return "RTK_FLOAT";
    case QUAL_GPS_DEGRADED: return "DEGRADED";
    case QUAL_GPS_HOLD_SHORT: return "HOLD_SHORT";
    case QUAL_GPS_LOST_WAIT: return "LOST_WAIT";
    case QUAL_NAV_ERROR: return "ERROR";
  }
  return "UNKNOWN";
}

static const char* stateString(NavState s) {
  switch (s) {
    case STATE_IDLE: return "IDLE";
    case STATE_RUNNING: return "RUNNING";
    case STATE_PAUSED: return "PAUSED";
    case STATE_ARRIVED: return "ARRIVED";
    case STATE_ERROR: return "ERROR";
  }
  return "UNKNOWN";
}

// ============== ESTIMATOR STATE ==============

struct Estimator {
  LocalCoords pos;
  LocalCoords vel;
  float heading = 0;
  float speed = 0;
  uint32_t lastUpdateMs = 0;
  bool rtkFixed = false;
  NavQuality quality = QUAL_NAV_ERROR;
  uint32_t qualityAgeMs = 0;
};

static Estimator g_est;
static uint32_t g_lastGoodRtkMs = 0;
static bool g_haveRtkFix = false;

// Dead reckoning state
static LocalCoords g_deadReckonPos = {0, 0};
static float g_deadReckonHeading = 0;
static uint32_t g_deadReckonStartMs = 0;
static bool g_deadReckoning = false;

// ============== GPS INPUT ==============

static HardwareSerial GpsSerial(1);
static HardwareSerial MotorSerial(2);

struct GpsRaw {
  double lat = 0, lon = 0;
  float hAcc = 999999;
  float vAcc = 999999;
  float speed = 0;
  float heading = 0;
  uint8_t fixType = 0;
  uint8_t carrier = 0;
  bool diff = false;
  bool valid = false;
  int8_t numSV = 0;
  uint32_t lastMs = 0;
};

static GpsRaw g_gps;
static uint32_t g_gpsRawBytes = 0;
static uint32_t g_gpsUbxParsed = 0;
static uint32_t g_gpsNmeaParsed = 0;
static uint32_t g_gpsPvtWatchdog = 0;
static uint32_t g_f9pRtcmMsgs = 0;
static uint32_t g_f9pRtcmCrcFail = 0;
static uint32_t g_lastF9pRtcmMs = 0;
static uint32_t g_rtcmLastType = 0;

// GPS parser state
enum GpsParserState { GPS_SYNC1, GPS_SYNC2, GPS_CLASS, GPS_ID, GPS_LEN1, GPS_LEN2, GPS_PAYLOAD, GPS_CKA, GPS_CKB };
static GpsParserState gpsState = GPS_SYNC1;
static uint8_t gpsClass = 0, gpsId = 0;
static uint16_t gpsLen = 0, gpsCount = 0;
static uint8_t gpsPayload[128];
static uint8_t gpsCkA = 0, gpsCkB = 0;
static char nmeaLine[128];
static uint8_t nmeaLen = 0;

// UBX config keys
static constexpr uint32_t CFG_RATE_MEAS = 0x30210001;
static constexpr uint32_t CFG_MSGOUT_UBX_NAV_PVT_UART1 = 0x20910007;
static constexpr uint32_t CFG_MSGOUT_UBX_NAV_RELPOSNED_UART1 = 0x2099008E;
static constexpr uint32_t CFG_MSGOUT_UBX_RXM_RTCM_UART1 = 0x20910269;
static constexpr uint32_t CFG_MSGOUT_NMEA_GGA_UART1 = 0x209100BA;
static constexpr uint32_t CFG_MSGOUT_NMEA_GLL_UART1 = 0x209100C9;
static constexpr uint32_t CFG_MSGOUT_NMEA_GSA_UART1 = 0x209100BE;
static constexpr uint32_t CFG_MSGOUT_NMEA_GSV_UART1 = 0x209100C3;
static constexpr uint32_t CFG_MSGOUT_NMEA_RMC_UART1 = 0x209100AC;
static constexpr uint32_t CFG_MSGOUT_NMEA_VTG_UART1 = 0x209100B1;

static uint32_t getU32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint16_t getU16(const uint8_t* p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static void putU32(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

static void writeUbx(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len) {
  uint8_t ckA = 0, ckB = 0;
  auto addCk = [&](uint8_t b) { ckA += b; ckB += ckA; };

  GpsSerial.write(0xB5);
  GpsSerial.write(0x62);
  GpsSerial.write(cls); addCk(cls);
  GpsSerial.write(id); addCk(id);
  GpsSerial.write((uint8_t)len); addCk((uint8_t)len);
  GpsSerial.write((uint8_t)(len >> 8)); addCk((uint8_t)(len >> 8));
  for (uint16_t i = 0; i < len; i++) {
    GpsSerial.write(payload[i]); addCk(payload[i]);
  }
  GpsSerial.write(ckA);
  GpsSerial.write(ckB);
}

static void sendValset(uint32_t key, uint32_t value, uint8_t size) {
  uint8_t payload[16] = {0};
  payload[0] = 0; payload[1] = 0x01;
  putU32(payload + 4, key);
  for (uint8_t i = 0; i < size; i++) {
    payload[8 + i] = (uint8_t)(value >> (8 * i));
  }
  writeUbx(0x06, 0x8A, payload, 8 + size);
  Serial.printf("GPS CFG key=0x%08lX val=%lu\n", (unsigned long)key, (unsigned long)value);
}

static void sendCfgMsg(uint8_t msgClass, uint8_t msgId, uint8_t uart1Rate) {
  uint8_t payload[8] = {0};
  payload[0] = msgClass;
  payload[1] = msgId;
  payload[3] = uart1Rate;
  writeUbx(0x06, 0x01, payload, sizeof(payload));
  Serial.printf("GPS CFG-MSG class=0x%02X id=0x%02X uart1=%u\n",
    msgClass, msgId, uart1Rate);
}

static void configureGpsRover() {
  Serial.println("GPS: configuring rover...");

  // CFG-PRT: UART, UBX in, UBX+RTCM out, current baud
  uint8_t cfgPrt[28] = {0};
  cfgPrt[0] = 0xB5; cfgPrt[1] = 0x62;
  cfgPrt[2] = 0x06; cfgPrt[3] = 0x00;
  cfgPrt[4] = 0x14; cfgPrt[5] = 0x00;
  cfgPrt[6] = 0x01;  // portID = UART1
  cfgPrt[12] = 0xC0; cfgPrt[13] = 0x08;  // mode = 8N1
  cfgPrt[16] = (uint8_t)GPS_BAUD;
  cfgPrt[17] = (uint8_t)(GPS_BAUD >> 8);
  cfgPrt[18] = (uint8_t)(GPS_BAUD >> 16);
  cfgPrt[19] = (uint8_t)(GPS_BAUD >> 24);
  cfgPrt[20] = 0x03; cfgPrt[21] = 0x00;  // inProtoMask = UBX + RTCM3
  cfgPrt[22] = 0x01; cfgPrt[23] = 0x00;  // outProtoMask = UBX
  uint8_t ckA = 0, ckB = 0;
  for (int i = 2; i <= 25; i++) { ckA += cfgPrt[i]; ckB += ckA; }
  cfgPrt[26] = ckA; cfgPrt[27] = ckB;
  GpsSerial.write(cfgPrt, sizeof(cfgPrt));
  delay(200);

  // Measurement rate: 10 Hz
  sendValset(CFG_RATE_MEAS, 100, 2);
  // Output once per navigation solution. With 100 ms measurement rate this is 10 Hz.
  sendValset(CFG_MSGOUT_UBX_NAV_PVT_UART1, 1, 1);
  sendValset(CFG_MSGOUT_UBX_RXM_RTCM_UART1, 1, 1);
  sendValset(CFG_MSGOUT_UBX_NAV_RELPOSNED_UART1, 1, 1);
  sendCfgMsg(0x01, 0x07, 1);  // UBX-NAV-PVT
  sendCfgMsg(0x02, 0x32, 1);  // UBX-RXM-RTCM
  sendCfgMsg(0x01, 0x3C, 1);  // UBX-NAV-RELPOSNED

  // Keep GGA as a low-bandwidth fallback while field-testing UBX stability.
  sendValset(CFG_MSGOUT_NMEA_GGA_UART1, 1, 1);
  sendValset(CFG_MSGOUT_NMEA_GLL_UART1, 0, 1);
  sendValset(CFG_MSGOUT_NMEA_GSA_UART1, 0, 1);
  sendValset(CFG_MSGOUT_NMEA_GSV_UART1, 0, 1);
  sendValset(CFG_MSGOUT_NMEA_RMC_UART1, 0, 1);
  sendValset(CFG_MSGOUT_NMEA_VTG_UART1, 0, 1);
  sendCfgMsg(0xF0, 0x00, 1);  // NMEA GGA fallback
  sendCfgMsg(0xF0, 0x01, 0);  // NMEA GLL
  sendCfgMsg(0xF0, 0x02, 0);  // NMEA GSA
  sendCfgMsg(0xF0, 0x03, 0);  // NMEA GSV
  sendCfgMsg(0xF0, 0x04, 0);  // NMEA RMC
  sendCfgMsg(0xF0, 0x05, 0);  // NMEA VTG

  Serial.println("GPS: configuration sent");
}

static void maintainGpsOutput() {
  uint32_t now = millis();
  if (now < 8000) return;
  if (g_gps.lastMs != 0 && now - g_gps.lastMs <= 3000) return;
  if (now - g_gpsPvtWatchdog < 5000) return;
  g_gpsPvtWatchdog = now;

  Serial.println("GPS: NAV-PVT stale, re-requesting rover outputs");
  sendCfgMsg(0x01, 0x07, 1);  // UBX-NAV-PVT
  sendCfgMsg(0x02, 0x32, 1);  // UBX-RXM-RTCM
  sendCfgMsg(0xF0, 0x00, 1);  // NMEA GGA fallback
  writeUbx(0x01, 0x07, nullptr, 0);
}

static void parseNavPvt(const uint8_t* p, uint16_t len) {
  if (len < 92) return;

  g_gps.lat = (int32_t)getU32(p + 28) * 1e-7;
  g_gps.lon = (int32_t)getU32(p + 24) * 1e-7;
  g_gps.hAcc = (float)getU32(p + 40);
  g_gps.vAcc = (float)getU32(p + 44);
  g_gps.speed = (int32_t)getU32(p + 60) * 0.001f;
  g_gps.heading = (int32_t)getU32(p + 64) * 1e-5f;
  if (g_gps.heading < 0) g_gps.heading += 360.0f;
  g_gps.fixType = p[20];
  g_gps.diff = (p[21] & 0x02) != 0;
  g_gps.carrier = (p[21] >> 6) & 0x03;
  g_gps.valid = (p[21] & 0x01) != 0;
  g_gps.numSV = p[23];
  g_gps.lastMs = millis();
  g_gpsUbxParsed++;
}

static void parseRxmRtcm(const uint8_t* p, uint16_t len) {
  if (len < 8) return;
  const bool crcFailed = (p[1] & 0x01) != 0;
  g_rtcmLastType = getU16(p + 6);
  if (crcFailed) {
    g_f9pRtcmCrcFail++;
  } else {
    g_f9pRtcmMsgs++;
    g_lastF9pRtcmMs = millis();
  }
}

static void feedGpsByte(uint8_t b) {
  g_gpsRawBytes++;

  // NMEA parser - try to parse basic GGA if UBX not working
  if (b == '$') {
    nmeaLen = 0;
    nmeaLine[nmeaLen++] = (char)b;
    return;
  }
  if (nmeaLen > 0 && nmeaLen < sizeof(nmeaLine) - 1) {
    if (b == '\n') {
      nmeaLine[nmeaLen] = 0;
      const bool ubxStale = g_gps.lastMs == 0 || millis() - g_gps.lastMs > 1500;
      if (ubxStale && nmeaLen > 6) {
        char* fields[15] = {0};
        uint8_t count = 0;
        char* save = nmeaLine;
        char* tok = strtok(nmeaLine, ",");
        while (tok && count < 15) {
          fields[count++] = tok;
          tok = strtok(NULL, ",");
        }
        if (count >= 10 && strncmp(fields[0] + 3, "GGA", 3) == 0) {
          int deg = atoi(fields[2]) / 100;
          double min = atof(fields[2]) - deg * 100;
          g_gps.lat = deg + min / 60.0;
          if (fields[3][0] == 'S') g_gps.lat = -g_gps.lat;
          deg = atoi(fields[4]) / 100;
          min = atof(fields[4]) - deg * 100;
          g_gps.lon = deg + min / 60.0;
          if (fields[5][0] == 'W') g_gps.lon = -g_gps.lon;
          int quality = atoi(fields[6]);
          g_gps.fixType = quality > 0 ? 3 : 0;
          g_gps.carrier = (quality == 4) ? 2 : (quality == 5 ? 1 : 0);
          g_gps.diff = quality >= 2;
          g_gps.valid = quality > 0;
          g_gps.numSV = atoi(fields[7]);
          float hdop = atof(fields[8]);
          if (hdop > 0.0f && hdop < 99.0f) {
            g_gps.hAcc = hdop * 1000.0f;
          }
          g_gps.lastMs = millis();
          g_gpsNmeaParsed++;
        }
      }
      nmeaLen = 0;
    } else if (b != '\r') {
      nmeaLine[nmeaLen++] = (char)b;
    }
  }

  // UBX parser
  switch (gpsState) {
    case GPS_SYNC1: gpsState = (b == 0xB5) ? GPS_SYNC2 : GPS_SYNC1; break;
    case GPS_SYNC2: gpsState = (b == 0x62) ? GPS_CLASS : GPS_SYNC1; gpsCkA = gpsCkB = 0; break;
    case GPS_CLASS: gpsClass = b; gpsCkA += b; gpsCkB += gpsCkA; gpsState = GPS_ID; break;
    case GPS_ID: gpsId = b; gpsCkA += b; gpsCkB += gpsCkA; gpsState = GPS_LEN1; break;
    case GPS_LEN1: gpsLen = b; gpsCkA += b; gpsCkB += gpsCkA; gpsState = GPS_LEN2; break;
    case GPS_LEN2:
      gpsLen |= ((uint16_t)b << 8);
      gpsCkA += b; gpsCkB += gpsCkA;
      gpsCount = 0;
      gpsState = (gpsLen == 0) ? GPS_CKA : (gpsLen > sizeof(gpsPayload) ? GPS_SYNC1 : GPS_PAYLOAD);
      break;
    case GPS_PAYLOAD: gpsPayload[gpsCount++] = b; gpsCkA += b; gpsCkB += gpsCkA;
      if (gpsCount >= gpsLen) gpsState = GPS_CKA; break;
    case GPS_CKA: gpsState = (b == gpsCkA) ? GPS_CKB : GPS_SYNC1; break;
    case GPS_CKB:
      gpsState = GPS_SYNC1;
      if (b == gpsCkB && gpsClass == 0x01 && gpsId == 0x07) {
        parseNavPvt(gpsPayload, gpsLen);
      } else if (b == gpsCkB && gpsClass == 0x02 && gpsId == 0x32) {
        parseRxmRtcm(gpsPayload, gpsLen);
      }
      break;
  }
}

// ============== RTCM ==============

static constexpr size_t RTCM_PACKET_BUF_SIZE = 1200;
static WiFiUDP rtcmUdp;
static uint32_t g_rtcmBytes = 0;
static uint32_t g_rtcmMsgs = 0;
static uint32_t g_rtcmErrors = 0;
static uint32_t g_rtcmWriteErrors = 0;
static uint32_t g_rtcmOversize = 0;
static uint32_t g_lastRtcmMs = 0;
static bool g_rtcmFresh = false;

static uint16_t rtcmMessageType(const uint8_t* frame, uint16_t len) {
  if (len < 5) return 0;
  return ((uint16_t)frame[3] << 4) | (frame[4] >> 4);
}

static void relayRtcm() {
  int pktSize = rtcmUdp.parsePacket();
  if (pktSize > 0) {
    uint8_t buf[RTCM_PACKET_BUF_SIZE];
    if ((size_t)pktSize > sizeof(buf)) {
      rtcmUdp.read(buf, sizeof(buf));
      while (rtcmUdp.available()) {
        rtcmUdp.read();
      }
      g_rtcmOversize++;
      g_rtcmErrors++;
      Serial.printf("RTCM: drop oversize packet len=%d max=%u\n",
        pktSize, (unsigned)sizeof(buf));
      return;
    }
    int len = rtcmUdp.read(buf, sizeof(buf));
    if (len > 0) {
      if (len != pktSize) {
        g_rtcmErrors++;
        Serial.printf("RTCM: short read len=%d expected=%d\n", len, pktSize);
        return;
      }
      if (buf[0] == 0xD3 && len >= 5) g_rtcmLastType = rtcmMessageType(buf, len);
      size_t written = GpsSerial.write(buf, len);
      if (written != (size_t)len) {
        g_rtcmWriteErrors++;
      }
      g_rtcmBytes += len;
      g_rtcmMsgs++;
      g_lastRtcmMs = millis();
      g_rtcmFresh = true;
    }
  }
  if (g_lastRtcmMs > 0 && millis() - g_lastRtcmMs > RTK_CORRECTION_TIMEOUT_MS) {
    g_rtcmFresh = false;
  }
}

// ============== IMU ==============

static Adafruit_BNO08x bno08x;
static float g_imuRawYaw = 0;
static float g_imuYaw = 0;
static float g_imuPitch = 0;
static float g_imuRoll = 0;
static uint32_t g_lastImuMs = 0;
static bool g_imuFresh = false;
static bool g_imuOk = false;
static float g_imuCalibrationOffset = 0;
static bool g_invertYaw = false;
static uint32_t g_imuFirstReadMs = 0;  // When IMU first gave valid data
static float g_imuPrevYaw = 0;          // Previous yaw for stability check
static uint32_t g_imuPrevYawMs = 0;     // When previous yaw was recorded

static bool initImu() {
  Serial.println("IMU: initializing BNO085...");
  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(100000);

  if (!bno08x.begin_I2C(0x4B, &Wire)) {
    Serial.println("IMU: trying 0x4A...");
    if (!bno08x.begin_I2C(0x4A, &Wire)) {
      Serial.println("IMU: ERROR - not found!");
      return false;
    }
  }
  Serial.println("IMU: found!");

  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 20000)) {
    Serial.println("IMU: ERROR - can't enable game rotation vector!");
    return false;
  }
  Serial.println("IMU: game rotation vector enabled");
  g_imuOk = true;
  return true;
}

static void updateImu() {
  if (!g_imuOk) return;
  sh2_SensorValue_t value;
  static uint32_t lastDebugMs = 0;
  while (bno08x.getSensorEvent(&value)) {
    if (value.sensorId == SH2_GAME_ROTATION_VECTOR) {
      float qw = value.un.gameRotationVector.real;
      float qx = value.un.gameRotationVector.i;
      float qy = value.un.gameRotationVector.j;
      float qz = value.un.gameRotationVector.k;

      float yaw = atan2(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
      float rawYaw = radToDeg(yaw);
      if (rawYaw < 0) rawYaw += 360.0f;
      if (g_invertYaw) rawYaw = normalizeAngle360(360.0f - rawYaw);
      g_imuRawYaw = normalizeAngle360(rawYaw);
      g_imuYaw = normalizeAngle360(g_imuRawYaw + g_imuCalibrationOffset);

      float pitch = asin(2.0f * (qw * qy - qz * qx));
      g_imuPitch = radToDeg(pitch);

      g_lastImuMs = millis();
      g_imuFresh = true;

      // Track first valid read time
      if (g_imuFirstReadMs == 0) {
        g_imuFirstReadMs = millis();
        g_imuPrevYaw = g_imuYaw;
        g_imuPrevYawMs = millis();
      }

      // Debug every 2 seconds
      uint32_t now = millis();
      if (now - lastDebugMs > 2000) {
        lastDebugMs = now;
        Serial.printf("IMU: raw=%.1f heading=%.1f offset=%.1f fresh=1 age=%lu\n",
          g_imuRawYaw, g_imuYaw, g_imuCalibrationOffset,
          (unsigned long)(now - g_imuFirstReadMs));
      }
    }
  }
  if (g_lastImuMs > 0 && millis() - g_lastImuMs > IMU_TIMEOUT_MS) {
    g_imuFresh = false;
  }
}

// Check if IMU data is available for calibration
static bool imuReadyForCalibration() {
  // Just check if IMU is fresh - robot uses its own IMU yaw
  return g_imuFresh;
}

static void calibrateImuToHeading(float desiredHeading) {
  desiredHeading = normalizeAngle360(desiredHeading);
  g_imuCalibrationOffset = normalizeAngle(desiredHeading - g_imuRawYaw);
  g_imuYaw = normalizeAngle360(g_imuRawYaw + g_imuCalibrationOffset);
  g_est.heading = g_imuYaw;
  saveNavPrefs();
}

static void updateEstimatorHeading() {
  if (g_imuFresh) {
    g_est.heading = g_imuYaw;
    return;
  }

  // GPS heading is course-over-ground, not body yaw. Use it only as a fallback
  // while moving fast enough for the F9P heading to be meaningful.
  const uint32_t gpsAge = g_gps.lastMs ? millis() - g_gps.lastMs : 99999;
  if (g_gps.valid && gpsAge <= GPS_HOLD_AGE_MS && fabsf(g_gps.speed) >= 0.15f) {
    g_est.heading = normalizeAngle360(g_gps.heading);
  }
}

// ============== ESTIMATOR ==============

static void updateEstimator() {
  uint32_t now = millis();
  uint32_t gpsAge = g_gps.lastMs ? now - g_gps.lastMs : 99999;
  uint32_t f9pRtcmAge = g_lastF9pRtcmMs ? now - g_lastF9pRtcmMs : 99999;
  bool f9pRtcmFresh = f9pRtcmAge <= RTK_CORRECTION_TIMEOUT_MS;

  // Determine quality
  NavQuality qual = QUAL_NAV_ERROR;
  if (g_gps.valid && g_gps.fixType >= 3) {
    if (g_gps.carrier == 2 && g_gps.hAcc <= RTK_FIXED_HACC_MM && gpsAge <= RTK_FIXED_AGE_MS && f9pRtcmFresh) {
      qual = QUAL_RTK_FIXED_GOOD;
    } else if (g_gps.carrier >= 1 && g_gps.hAcc <= RTK_FLOAT_HACC_MM && gpsAge <= RTK_FLOAT_AGE_MS && f9pRtcmFresh) {
      qual = QUAL_RTK_FLOAT_OK;
    } else if (g_gps.hAcc <= DEGRADED_HACC_MM && gpsAge <= GPS_HOLD_AGE_MS) {
      qual = QUAL_GPS_DEGRADED;
    } else if (gpsAge <= GPS_HOLD_AGE_MS) {
      qual = QUAL_GPS_HOLD_SHORT;
    } else {
      qual = QUAL_GPS_LOST_WAIT;
    }
  } else if (gpsAge <= GPS_HOLD_AGE_MS) {
    qual = QUAL_GPS_HOLD_SHORT;
  } else {
    qual = QUAL_GPS_LOST_WAIT;
  }

  g_est.quality = qual;
  g_est.qualityAgeMs = gpsAge;
  updateEstimatorHeading();

  // Update position estimate
  if (qual == QUAL_RTK_FIXED_GOOD || qual == QUAL_RTK_FLOAT_OK || qual == QUAL_GPS_DEGRADED) {
    LocalCoords local = toLocal(g_gps.lat, g_gps.lon);
    if (!g_origin.valid) {
      setOrigin(g_gps.lat, g_gps.lon);
      local = {0, 0};
    }
    float dt = (g_est.lastUpdateMs > 0) ? (now - g_est.lastUpdateMs) / 1000.0f : 0.05f;
    if (dt > 0.5f) dt = 0.05f;
    if (dt < 0.001f) dt = 0.05f;  // Prevent division by near-zero

    g_est.vel.x = (local.x - g_est.pos.x) / dt;
    g_est.vel.y = (local.y - g_est.pos.y) / dt;
    g_est.speed = sqrt(g_est.vel.x * g_est.vel.x + g_est.vel.y * g_est.vel.y);
    g_est.pos = local;
    g_est.lastUpdateMs = now;
    g_lastGoodRtkMs = now;
    g_haveRtkFix = (qual == QUAL_RTK_FIXED_GOOD || qual == QUAL_RTK_FLOAT_OK);
    g_est.rtkFixed = (g_gps.carrier == 2);
    g_deadReckoning = false;
  } else if (qual == QUAL_GPS_HOLD_SHORT) {
    // Dead reckoning
    if (!g_deadReckoning) {
      g_deadReckonPos = g_est.pos;
      g_deadReckonHeading = g_est.heading;
      g_deadReckonStartMs = now;
      g_deadReckoning = true;
    }

    float dt = (g_est.lastUpdateMs > 0) ? (now - g_est.lastUpdateMs) / 1000.0f : 0.0f;
    if (dt > 0 && dt < 0.5f) {
      float speed = g_est.speed * 0.8f;
      float headingRad = degToRad(g_est.heading);
      g_est.pos.x += speed * sin(headingRad) * dt;
      g_est.pos.y += speed * cos(headingRad) * dt;
    }
    g_est.lastUpdateMs = now;
  } else {
    g_est.speed = 0;
    g_est.vel = {0, 0};
    g_deadReckoning = false;
  }
}

// ============== ROUTE ==============

static constexpr uint8_t MAX_WAYPOINTS = 160;
static constexpr uint8_t MAX_AREA_POINTS = 32;
struct Waypoint {
  LocalCoords pos;
};
static Waypoint g_route[MAX_WAYPOINTS];
static bool g_routeReceived[MAX_WAYPOINTS];
static uint8_t g_routeCount = 0;
static uint8_t g_routeIndex = 0;
static LocalCoords g_area[MAX_AREA_POINTS];
static bool g_areaReceived[MAX_AREA_POINTS];
static uint8_t g_areaCount = 0;
static float g_areaLineStep = 0.42f;
static bool g_areaReady = false;

// Simple single-target navigation
static bool g_singleTargetActive = false;
static LocalCoords g_singleTarget = {0, 0};
static bool g_singleTargetSet = false;

static NavState g_navState = STATE_IDLE;
static const char* g_navReason = "idle";

// Pure pursuit parameters
static constexpr float ARRIVAL_DIST = 0.2f;
static constexpr float BASE_SPEED = 0.3f;
static constexpr float LOOKAHEAD_MIN_M = 0.55f;
static constexpr float LOOKAHEAD_MAX_M = 1.50f;
static constexpr float LOOKAHEAD_SPEED_GAIN = 1.6f;
static constexpr float K_HEADING = 0.45f;
static constexpr float K_CROSSTRACK = 18.0f;

static float g_navForwardScale = 1.0f;
static float g_navTurnScale = 1.0f;
static bool g_invertForward = false;
static bool g_invertSteering = false;

// Motor feedback
static bool g_haveMotorFeedback = false;
static int16_t g_motorFbCmd1 = 0, g_motorFbCmd2 = 0;
static int16_t g_motorSpeedR = 0, g_motorSpeedL = 0;
static int16_t g_motorBatVoltage = 0, g_motorBoardTemp = 0;
static uint32_t g_lastManualCmdMs = 0;
static bool g_manualActive = false;

struct __attribute__((packed)) SerialCommand {
  uint16_t start;
  int16_t steer;
  int16_t speed;
  uint16_t checksum;
};

struct __attribute__((packed)) SerialFeedback {
  uint16_t start;
  int16_t cmd1;
  int16_t cmd2;
  int16_t speedR_meas;
  int16_t speedL_meas;
  int16_t batVoltage;
  int16_t boardTemp;
  uint16_t cmdLed;
  uint16_t checksum;
};

// Cross track error for telemetry
static float g_crossTrackError = 0;
static float g_headingError = 0;
static float g_distToRouteEnd = 0;

static bool routeReady() {
  if (g_routeCount == 0) return false;
  for (uint8_t i = 0; i < g_routeCount; i++) {
    if (!g_routeReceived[i]) return false;
  }
  return true;
}

static bool areaReady() {
  if (g_areaCount < 3) return false;
  for (uint8_t i = 0; i < g_areaCount; i++) {
    if (!g_areaReceived[i]) return false;
  }
  return true;
}

static bool addRoutePoint(LocalCoords p) {
  if (g_routeCount >= MAX_WAYPOINTS) return false;
  if (g_routeCount > 0) {
    const LocalCoords prev = g_route[g_routeCount - 1].pos;
    if (hypotf(p.x - prev.x, p.y - prev.y) < 0.05f) return true;
  }
  g_route[g_routeCount].pos = p;
  g_routeReceived[g_routeCount] = true;
  g_routeCount++;
  return true;
}

static void sortFloatArray(float* values, uint8_t count) {
  for (uint8_t i = 1; i < count; i++) {
    float v = values[i];
    int j = (int)i - 1;
    while (j >= 0 && values[j] > v) {
      values[j + 1] = values[j];
      j--;
    }
    values[j + 1] = v;
  }
}

static uint8_t scanHorizontalIntersections(float y, float* xs, uint8_t maxXs) {
  uint8_t count = 0;
  for (uint8_t i = 0; i < g_areaCount; i++) {
    const LocalCoords a = g_area[i];
    const LocalCoords b = g_area[(i + 1) % g_areaCount];
    if ((a.y > y) == (b.y > y)) continue;
    if (fabsf(b.y - a.y) < 0.001f) continue;
    if (count >= maxXs) break;
    const float t = (y - a.y) / (b.y - a.y);
    xs[count++] = a.x + t * (b.x - a.x);
  }
  sortFloatArray(xs, count);
  return count;
}

static uint8_t scanVerticalIntersections(float x, float* ys, uint8_t maxYs) {
  uint8_t count = 0;
  for (uint8_t i = 0; i < g_areaCount; i++) {
    const LocalCoords a = g_area[i];
    const LocalCoords b = g_area[(i + 1) % g_areaCount];
    if ((a.x > x) == (b.x > x)) continue;
    if (fabsf(b.x - a.x) < 0.001f) continue;
    if (count >= maxYs) break;
    const float t = (x - a.x) / (b.x - a.x);
    ys[count++] = a.y + t * (b.y - a.y);
  }
  sortFloatArray(ys, count);
  return count;
}

static bool buildRouteFromArea() {
  if (!areaReady()) return false;
  if (!g_origin.valid) return false;

  float minX = g_area[0].x, maxX = g_area[0].x;
  float minY = g_area[0].y, maxY = g_area[0].y;
  for (uint8_t i = 1; i < g_areaCount; i++) {
    minX = min(minX, g_area[i].x);
    maxX = max(maxX, g_area[i].x);
    minY = min(minY, g_area[i].y);
    maxY = max(maxY, g_area[i].y);
  }

  const float width = maxX - minX;
  const float height = maxY - minY;
  float step = constrain(g_areaLineStep, 0.15f, 2.0f);
  g_routeCount = 0;
  g_routeIndex = 0;
  memset(g_routeReceived, 0, sizeof(g_routeReceived));
  addRoutePoint(g_est.pos);

  bool reverse = false;
  bool ok = true;
  float intersections[MAX_AREA_POINTS];

  if (width >= height) {
    for (float y = minY + step * 0.5f; y <= maxY - step * 0.5f; y += step) {
      const uint8_t n = scanHorizontalIntersections(y, intersections, MAX_AREA_POINTS);
      for (uint8_t i = 0; i + 1 < n; i += 2) {
        LocalCoords a = {intersections[i], y};
        LocalCoords b = {intersections[i + 1], y};
        if (reverse) {
          ok = addRoutePoint(b) && addRoutePoint(a);
        } else {
          ok = addRoutePoint(a) && addRoutePoint(b);
        }
        if (!ok) break;
        reverse = !reverse;
      }
      if (!ok) break;
    }
  } else {
    for (float x = minX + step * 0.5f; x <= maxX - step * 0.5f; x += step) {
      const uint8_t n = scanVerticalIntersections(x, intersections, MAX_AREA_POINTS);
      for (uint8_t i = 0; i + 1 < n; i += 2) {
        LocalCoords a = {x, intersections[i]};
        LocalCoords b = {x, intersections[i + 1]};
        if (reverse) {
          ok = addRoutePoint(b) && addRoutePoint(a);
        } else {
          ok = addRoutePoint(a) && addRoutePoint(b);
        }
        if (!ok) break;
        reverse = !reverse;
      }
      if (!ok) break;
    }
  }

  if (!ok || g_routeCount < 2) {
    g_routeCount = 0;
    memset(g_routeReceived, 0, sizeof(g_routeReceived));
    return false;
  }

  g_navState = STATE_IDLE;
  g_navReason = "route_planned_on_robot";
  Serial.printf("PLAN: area %u pts -> route %u pts, step=%.2f\n",
    g_areaCount, g_routeCount, step);
  return true;
}

static void saveNavPrefs() {
  g_prefs.putFloat("imuOffset", g_imuCalibrationOffset);
  g_prefs.putBool("invYaw", g_invertYaw);
  g_prefs.putBool("invFwd", g_invertForward);
  g_prefs.putBool("invSteer", g_invertSteering);
  g_prefs.putFloat("fwdScale", g_navForwardScale);
  g_prefs.putFloat("turnScale", g_navTurnScale);
}

static void loadNavPrefs() {
  g_imuCalibrationOffset = g_prefs.getFloat("imuOffset", 0.0f);
  g_invertYaw = g_prefs.getBool("invYaw", false);
  g_invertForward = g_prefs.getBool("invFwd", false);
  g_invertSteering = g_prefs.getBool("invSteer", false);
  g_navForwardScale = g_prefs.getFloat("fwdScale", 1.0f);
  g_navTurnScale = g_prefs.getFloat("turnScale", 1.0f);
}

// ============== WEBSOCKET ==============

static AsyncWebServer server(WS_PORT);
static AsyncWebSocket ws("/ws");
static bool g_wifiConnected = false;
static bool g_serverStarted = false;

static void handleWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                         AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    client->text("STATE,CONNECTED");
    Serial.printf("WS: client %u connected\n", client->id());
  }
  if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WS: client %u disconnected\n", client->id());
  }
  if (type != WS_EVT_DATA) return;

  char msg[256];
  if (len >= sizeof(msg)) return;
  memcpy(msg, data, len);
  msg[len] = 0;

  String cmd = String(msg);
  cmd.trim();

  Serial.printf("WS CMD: '%s'\n", cmd.c_str());

  if (cmd == "PING") { client->text("PONG"); return; }
  // FREE: stop sending motor commands (release hoverboard from lock)
  if (cmd == "FREE") {
    g_targetLeft = 0;
    g_targetRight = 0;
    g_curLeft = 0;
    g_curRight = 0;
    g_cmdSpeed = 0;
    g_cmdSteer = 0;
    g_navState = STATE_IDLE;
    g_navReason = "idle";
    g_manualActive = false;
    g_lastCmdMs = 0;
    client->text("OK FREE");
    Serial.println("MOTOR: free/idle");
    return;
  }
  if (cmd.startsWith("M,")) {
    int comma1 = cmd.indexOf(',', 2);
    if (comma1 > 2) {
      int left = cmd.substring(2, comma1).toInt();
      int right = cmd.substring(comma1 + 1).toInt();
      // Same as sound.ino: divide by INPUT_DIV
      left = left / INPUT_DIV;
      right = right / INPUT_DIV;
      g_navState = STATE_IDLE;
      g_navReason = "manual";
      g_targetLeft = constrain(left, -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);
      g_targetRight = constrain(right, -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);
      g_lastCmdMs = millis();
      g_isFailSafeStopping = false;
      g_manualActive = true;
      client->text("OK");
      return;
    }
    client->text("ERR,MOVE");
    return;
  }
  if (cmd == "STOP" || cmd == "NAV_STOP") {
    g_navState = STATE_IDLE;
    g_navReason = "stopped";
    g_targetLeft = 0;
    g_targetRight = 0;
    g_lastCmdMs = 0;
    g_manualActive = false;
    client->text("OK");
    return;
  }
  if (cmd == "PAUSE" || cmd == "NAV_PAUSE") {
    g_navState = STATE_PAUSED;
    g_navReason = "paused";
    g_targetLeft = 0;
    g_targetRight = 0;
    g_manualActive = false;
    client->text("OK");
    return;
  }
  if (cmd == "RESUME" || cmd == "NAV_RESUME") {
    if (g_navState == STATE_PAUSED) g_navState = STATE_RUNNING;
    client->text("OK");
    return;
  }
  if (cmd == "START" || cmd == "NAV_START") {
    if (routeReady() && g_routeIndex < g_routeCount) {
      g_navState = STATE_RUNNING;
      g_navReason = "started";
      g_manualActive = false;
      client->text("OK");
    } else {
      client->text("ERR,NO_ROUTE");
    }
    return;
  }
  // ROUTE_BEGIN,count,originLat,originLon
  if (cmd.startsWith("ROUTE_BEGIN,")) {
    int first = 12;
    int comma1 = cmd.indexOf(',', first);
    int comma2 = cmd.indexOf(',', comma1 + 1);
    if (comma1 > first && comma2 > comma1) {
      int count = cmd.substring(first, comma1).toInt();
      double originLat = cmd.substring(comma1 + 1, comma2).toDouble();
      double originLon = cmd.substring(comma2 + 1).toDouble();
      if (count > 0 && count <= MAX_WAYPOINTS && originLat != 0 && originLon != 0) {
        g_routeCount = count;
        g_routeIndex = 0;
        memset(g_routeReceived, 0, sizeof(g_routeReceived));
        g_navState = STATE_IDLE;
        setOrigin(originLat, originLon);
        client->text("OK");
        Serial.printf("WS: route begin %u points, origin %.8f %.8f\n", count, originLat, originLon);
        return;
      }
    }
    client->text("ERR,INVALID");
    return;
  }
  // ROUTE_WP,idx,x_m,y_m
  if (cmd.startsWith("ROUTE_WP,")) {
    int first = 9;
    int comma1 = cmd.indexOf(',', first);
    int comma2 = cmd.indexOf(',', comma1 + 1);
    if (comma1 > first && comma2 > comma1) {
      int idx = cmd.substring(first, comma1).toInt();
      float x = cmd.substring(comma1 + 1, comma2).toFloat();
      float y = cmd.substring(comma2 + 1).toFloat();
      if (idx >= 0 && idx < g_routeCount) {
        g_route[idx].pos = {x, y};
        g_routeReceived[idx] = true;
        client->text("OK");
        return;
      }
    }
    client->text("ERR,INVALID");
    return;
  }
  if (cmd == "ROUTE_END") {
    if (routeReady()) {
      client->text("OK");
      Serial.printf("WS: route ready %u waypoints\n", g_routeCount);
    } else {
      client->text("ERR,ROUTE_INCOMPLETE");
    }
    return;
  }
  // AREA_BEGIN,count,originLat,originLon,lineStep
  if (cmd.startsWith("AREA_BEGIN,")) {
    int first = 11;
    int p1 = cmd.indexOf(',', first);
    int p2 = cmd.indexOf(',', p1 + 1);
    int p3 = cmd.indexOf(',', p2 + 1);
    if (p1 > first && p2 > p1 && p3 > p2) {
      int count = cmd.substring(first, p1).toInt();
      double originLat = cmd.substring(p1 + 1, p2).toDouble();
      double originLon = cmd.substring(p2 + 1, p3).toDouble();
      float lineStep = cmd.substring(p3 + 1).toFloat();
      if (count >= 3 && count <= MAX_AREA_POINTS && originLat != 0 && originLon != 0 &&
          lineStep > 0.05f && lineStep <= 5.0f) {
        g_areaCount = (uint8_t)count;
        g_areaLineStep = lineStep;
        g_areaReady = false;
        memset(g_areaReceived, 0, sizeof(g_areaReceived));
        g_routeCount = 0;
        memset(g_routeReceived, 0, sizeof(g_routeReceived));
        g_navState = STATE_IDLE;
        g_navReason = "area_upload";
        setOrigin(originLat, originLon);
        client->text("OK");
        Serial.printf("WS: area begin %u points, origin %.8f %.8f, step=%.2f\n",
          count, originLat, originLon, lineStep);
        return;
      }
    }
    client->text("ERR,AREA_BEGIN");
    return;
  }
  // AREA_PT,idx,x_m,y_m
  if (cmd.startsWith("AREA_PT,")) {
    int first = 8;
    int p1 = cmd.indexOf(',', first);
    int p2 = cmd.indexOf(',', p1 + 1);
    if (p1 > first && p2 > p1) {
      int idx = cmd.substring(first, p1).toInt();
      float x = cmd.substring(p1 + 1, p2).toFloat();
      float y = cmd.substring(p2 + 1).toFloat();
      if (idx >= 0 && idx < g_areaCount && isfinite(x) && isfinite(y)) {
        g_area[idx] = {x, y};
        g_areaReceived[idx] = true;
        client->text("OK");
        return;
      }
    }
    client->text("ERR,AREA_PT");
    return;
  }
  if (cmd == "AREA_END" || cmd == "PLAN_CLEAN") {
    g_areaReady = areaReady();
    if (g_areaReady && buildRouteFromArea()) {
      char resp[48];
      snprintf(resp, sizeof(resp), "OK,ROUTE,%u", g_routeCount);
      client->text(resp);
    } else {
      client->text("ERR,PLAN_FAILED");
    }
    return;
  }
  // Simple single-target navigation: GO_TO,lat,lon
  // Robot saves the target and starts navigation to it
  if (cmd.startsWith("GO_TO,")) {
    int comma = cmd.indexOf(',', 6);
    if (comma > 6) {
      double lat = cmd.substring(6, comma).toDouble();
      double lon = cmd.substring(comma + 1).toDouble();
      if (lat != 0 && lon != 0) {
        // Set origin from current GPS if not set
        if (!g_origin.valid && g_gps.valid) {
          setOrigin(g_gps.lat, g_gps.lon);
        }
        if (!g_origin.valid) {
          client->text("ERR,NO_ORIGIN");
          return;
        }
        g_singleTarget = toLocal(lat, lon);
        g_singleTargetSet = true;
        g_singleTargetActive = true;
        // Build a simple 2-point route: current position -> target
        g_routeCount = 2;
        g_routeIndex = 0;
        memset(g_routeReceived, 0, sizeof(g_routeReceived));
        g_routeReceived[0] = true;
        g_routeReceived[1] = true;
        g_route[0].pos = g_est.pos;  // Current position
        g_route[1].pos = g_singleTarget;  // Target

        g_navState = STATE_RUNNING;
        g_navReason = "go_to";
        g_manualActive = false;
        client->text("OK");
        Serial.printf("GO_TO: target lat=%.8f lon=%.8f -> (%.2f, %.2f) m\n",
          lat, lon, g_singleTarget.x, g_singleTarget.y);
        return;
      }
    }
    client->text("ERR,GO_TO");
    return;
  }
  // STATUS request
  if (cmd == "STATUS") {
    char resp[256];
    snprintf(resp, sizeof(resp),
      "STATUS,%s,%s,%.2f,%.2f,%.1f,%.3f,%.3f,%u",
      stateString(g_navState),
      qualityString(g_est.quality),
      g_est.pos.x, g_est.pos.y,
      g_est.heading,
      g_est.speed,
      g_crossTrackError,
      g_routeIndex
    );
    client->text(resp);
    return;
  }
  // Calibrate IMU without GPS bearing: calibrated heading follows raw yaw.
  // Prefer CAL_IMU,<bearing_deg> from GPS Debug when a target is selected.
  if (cmd == "CAL_IMU_SELF") {
    Serial.printf("CAL_SELF: raw=%.1f heading=%.1f ready=%u headingBefore=%.1f\n",
      g_imuRawYaw, g_imuYaw, imuReadyForCalibration() ? 1 : 0, g_est.heading);
    if (imuReadyForCalibration()) {
      float oldHeading = g_est.heading;
      calibrateImuToHeading(g_imuRawYaw);
      Serial.printf("CAL_SELF: SUCCESS heading %.1f -> %.1f offset=%.1f\n",
        oldHeading, g_est.heading, g_imuCalibrationOffset);
      char resp[64];
      snprintf(resp, sizeof(resp), "CAL_OK,heading=%.1f,offset=%.1f", g_est.heading, g_imuCalibrationOffset);
      client->text(resp);
      return;
    }
    client->text("ERR,CAL_NOT_READY");
    return;
  }
  // Calibrate IMU to a real world heading: CAL_IMU,<bearing_degrees>.
  // When the robot nose is aimed at the selected target, the app sends the
  // GPS bearing to that target. From then on heading is updated continuously
  // as raw BNO085 yaw + stored offset.
  if (cmd.startsWith("CAL_IMU")) {
    float desiredHeading = NAN;
    int comma = cmd.indexOf(',');
    if (comma > 0) {
      desiredHeading = cmd.substring(comma + 1).toFloat();
    }
    Serial.printf("CAL_IMU: raw=%.1f heading=%.1f desired=%.1f ready=%u headingBefore=%.1f\n",
      g_imuRawYaw, g_imuYaw, desiredHeading, imuReadyForCalibration() ? 1 : 0, g_est.heading);
    if (imuReadyForCalibration()) {
      float oldHeading = g_est.heading;
      if (isfinite(desiredHeading)) {
        calibrateImuToHeading(desiredHeading);
      } else {
        calibrateImuToHeading(g_imuRawYaw);
      }
      Serial.printf("CAL_IMU: SUCCESS heading %.1f -> %.1f raw=%.1f offset=%.1f\n",
        oldHeading, g_est.heading, g_imuRawYaw, g_imuCalibrationOffset);
      char resp[64];
      snprintf(resp, sizeof(resp), "CAL_OK,heading=%.1f,raw=%.1f,offset=%.1f",
        g_est.heading, g_imuRawYaw, g_imuCalibrationOffset);
      client->text(resp);
      return;
    }
    client->text("ERR,CAL_NOT_READY");
    return;
  }
  if (cmd.startsWith("NAV_CFG,")) {
    int p1 = cmd.indexOf(',', 8);
    int p2 = cmd.indexOf(',', p1 + 1);
    int p3 = cmd.indexOf(',', p2 + 1);
    int p4 = cmd.indexOf(',', p3 + 1);
    int p5 = cmd.indexOf(',', p4 + 1);
    if (p1 > 0 && p2 > p1 && p3 > p2 && p4 > p3 && p5 > p4) {
      int forwardPercent = cmd.substring(8, p1).toInt();
      int turnPercent = cmd.substring(p1 + 1, p2).toInt();
      g_invertForward = cmd.substring(p2 + 1, p3).toInt() != 0;
      g_invertSteering = cmd.substring(p3 + 1, p4).toInt() != 0;
      g_imuCalibrationOffset = cmd.substring(p4 + 1, p5).toFloat();
      g_invertYaw = cmd.substring(p5 + 1).toInt() != 0;
      g_navForwardScale = constrain(forwardPercent / 100.0f, 0.15f, 1.0f);
      g_navTurnScale = constrain(turnPercent / 100.0f, 0.15f, 1.0f);
      saveNavPrefs();
      client->text("OK");
      Serial.printf("NAV_CFG: fwd=%.2f turn=%.2f invF=%u invS=%u imuOffset=%.1f invYaw=%u\n",
        g_navForwardScale, g_navTurnScale, g_invertForward ? 1 : 0,
        g_invertSteering ? 1 : 0, g_imuCalibrationOffset, g_invertYaw ? 1 : 0);
      return;
    }
    client->text("ERR,NAV_CFG");
    return;
  }
}

static void broadcastTelemetry() {
  static uint32_t lastTelem = 0;
  uint32_t now = millis();
  if (now - lastTelem < TELEMETRY_MS) return;
  lastTelem = now;

  char msg[256];
  const uint32_t gpsAge = g_gps.lastMs ? now - g_gps.lastMs : 99999;
  const uint32_t rtcmTransportAge = g_lastRtcmMs ? now - g_lastRtcmMs : 99999;
  const uint32_t rtcmF9pAge = g_lastF9pRtcmMs ? now - g_lastF9pRtcmMs : 99999;
  const uint32_t imuAge = g_lastImuMs ? now - g_lastImuMs : 99999;
  const char* carrier = g_gps.carrier == 2 ? "fixed" : (g_gps.carrier == 1 ? "float" : "none");

  // Main telemetry: keep the old Flutter parser compatible.
  // Format: TEL,lat,lon,height,heading(est),fixType,carrier,diff,numSV,hAcc,vAcc,speed,something,gpsAge,rtcmBytes,rtcmAge,imuYaw,imuAge,imuFresh,...
  snprintf(msg, sizeof(msg),
    "TEL,%.8f,%.8f,%.2f,%.1f,%u,%s,%u,%u,%.0f,%.0f,%.3f,%.2f,%lu,%lu,%lu,%.1f,%lu,%u,%lu,%lu,%s,%lu,%lu,%lu",
    g_gps.lat, g_gps.lon,
    0.0f,
    g_est.heading,  // This is the calibrated heading
    g_gps.fixType,
    carrier,
    g_gps.diff ? 1 : 0,
    (uint8_t)g_gps.numSV,
    g_gps.hAcc,
    g_gps.vAcc,
    g_gps.speed,
    0.0f,
    (unsigned long)gpsAge,
    (unsigned long)g_rtcmBytes,
    (unsigned long)rtcmTransportAge,
    g_imuYaw,
    (unsigned long)imuAge,
    g_imuFresh ? 1 : 0,
    (unsigned long)rtcmTransportAge,
    (unsigned long)rtcmF9pAge,
    g_rtcmFresh ? "udp" : "none",
    (unsigned long)g_f9pRtcmMsgs,
    (unsigned long)g_f9pRtcmCrcFail,
    (unsigned long)g_rtcmLastType
  );
  ws.textAll(msg);

  // Extended GPS info.
  snprintf(msg, sizeof(msg),
    "GPSDBG,%.8f,%.8f,%.2f,%.1f,%u,%s,%u,%u,%.0f,%.0f,%.3f,%.2f,%lu",
    g_gps.lat, g_gps.lon,
    0.0f,
    g_est.heading,
    g_gps.fixType,
    carrier,
    g_gps.diff ? 1 : 0,
    (uint8_t)g_gps.numSV,
    g_gps.hAcc,
    g_gps.vAcc,
    g_gps.speed,
    0.0f,
    (unsigned long)gpsAge
  );
  ws.textAll(msg);

  // RTCM status
  snprintf(msg, sizeof(msg),
    "RTCM,%lu,%lu,%lu,%lu,%s,%lu,%lu,%lu",
    (unsigned long)g_rtcmBytes,
    (unsigned long)rtcmTransportAge,
    (unsigned long)rtcmTransportAge,
    (unsigned long)rtcmF9pAge,
    g_rtcmFresh ? "udp" : "none",
    (unsigned long)g_f9pRtcmMsgs,
    (unsigned long)g_f9pRtcmCrcFail,
    (unsigned long)g_rtcmLastType
  );
  ws.textAll(msg);

  // IMU status
  snprintf(msg, sizeof(msg),
    "IMU,%.1f,%lu,%u",
    g_imuYaw,
    (unsigned long)imuAge,
    g_imuFresh ? 1 : 0
  );
  ws.textAll(msg);

  snprintf(msg, sizeof(msg),
    "NAV,%s,%u,%u,%.2f,%.1f",
    stateString(g_navState),
    g_routeIndex,
    g_routeCount,
    g_distToRouteEnd,
    g_headingError
  );
  ws.textAll(msg);

  // Motor status
  snprintf(msg, sizeof(msg),
    "MOTOR,%d,%d,%u,%d,%d,%d,%d",
    g_curLeft, g_curRight,
    g_haveMotorFeedback ? 1 : 0,
    g_motorSpeedL, g_motorSpeedR,
    g_motorBatVoltage, g_motorBoardTemp
  );
  ws.textAll(msg);
}

// ============== NAVIGATION UPDATE ==============

static void navUpdate() {
  uint32_t now = millis();

  if (g_navState != STATE_RUNNING) {
    if (g_manualActive) return;
    // Navigation not active - motors controlled by manual or idle
    return;
  }

  if (g_routeIndex >= g_routeCount) {
    g_navState = STATE_ARRIVED;
    g_navReason = "route_complete";
    g_targetLeft = g_targetRight = 0;
    char navMsg[48];
    snprintf(navMsg, sizeof(navMsg), "NAV,ARRIVED,%u,%u,0.00", g_routeIndex, g_routeCount);
    ws.textAll(navMsg);
    return;
  }

  // Check quality - don't move if quality is bad
  if (g_est.quality == QUAL_NAV_ERROR || g_est.quality == QUAL_GPS_LOST_WAIT) {
    g_targetLeft = g_targetRight = 0;
    g_navReason = "no_fix";
    return;
  }

  if (g_routeCount == 1) {
    LocalCoords only = g_route[0].pos;
    if (hypotf(only.x - g_est.pos.x, only.y - g_est.pos.y) < ARRIVAL_DIST) {
      g_navState = STATE_ARRIVED;
      g_navReason = "route_complete";
      g_targetLeft = g_targetRight = 0;
      char navMsg[48];
      snprintf(navMsg, sizeof(navMsg), "NAV,ARRIVED,%u,%u,0.00", g_routeIndex, g_routeCount);
      ws.textAll(navMsg);
      return;
    }
  }

  int bestSeg = -1;
  float bestT = 0.0f;
  float bestDistSq = 1.0e12f;
  LocalCoords closest = g_route[g_routeIndex].pos;

  for (uint8_t i = g_routeIndex; i + 1 < g_routeCount; i++) {
    LocalCoords a = g_route[i].pos;
    LocalCoords b = g_route[i + 1].pos;
    const float vx = b.x - a.x;
    const float vy = b.y - a.y;
    const float lenSq = vx * vx + vy * vy;
    if (lenSq < 0.0001f) continue;
    float t = ((g_est.pos.x - a.x) * vx + (g_est.pos.y - a.y) * vy) / lenSq;
    t = constrain(t, 0.0f, 1.0f);
    LocalCoords p = {a.x + vx * t, a.y + vy * t};
    const float dx = g_est.pos.x - p.x;
    const float dy = g_est.pos.y - p.y;
    const float dSq = dx * dx + dy * dy;
    if (dSq < bestDistSq) {
      bestDistSq = dSq;
      bestSeg = i;
      bestT = t;
      closest = p;
    }
  }

  if (bestSeg < 0) {
    LocalCoords target = g_route[g_routeIndex].pos;
    bestSeg = g_routeIndex;
    closest = target;
  }

  g_routeIndex = (uint8_t)bestSeg;
  g_crossTrackError = sqrtf(bestDistSq);
  if (bestSeg + 1 < g_routeCount) {
    LocalCoords a = g_route[bestSeg].pos;
    LocalCoords b = g_route[bestSeg + 1].pos;
    const float vx = b.x - a.x;
    const float vy = b.y - a.y;
    const float len = hypotf(vx, vy);
    if (len > 0.001f) {
      const float signedCross = (vx * (g_est.pos.y - a.y) - vy * (g_est.pos.x - a.x)) / len;
      g_crossTrackError = signedCross;
    }
  }

  float remainingOnPath = 0.0f;
  if (bestSeg + 1 < g_routeCount) {
    LocalCoords b = g_route[bestSeg + 1].pos;
    remainingOnPath += hypotf(b.x - closest.x, b.y - closest.y);
    for (uint8_t i = bestSeg + 1; i + 1 < g_routeCount; i++) {
      remainingOnPath += hypotf(g_route[i + 1].pos.x - g_route[i].pos.x,
                                g_route[i + 1].pos.y - g_route[i].pos.y);
    }
  }
  g_distToRouteEnd = remainingOnPath;
  if (g_routeCount > 1 && remainingOnPath < ARRIVAL_DIST && bestSeg + 1 >= g_routeCount - 1) {
    g_navState = STATE_ARRIVED;
    g_navReason = "route_complete";
    g_targetLeft = g_targetRight = 0;
    char navMsg[48];
    snprintf(navMsg, sizeof(navMsg), "NAV,ARRIVED,%u,%u,0.00", g_routeIndex, g_routeCount);
    ws.textAll(navMsg);
    return;
  }

  float lookahead = constrain(LOOKAHEAD_MIN_M + g_est.speed * LOOKAHEAD_SPEED_GAIN,
                              LOOKAHEAD_MIN_M, LOOKAHEAD_MAX_M);
  if (g_est.quality == QUAL_GPS_HOLD_SHORT) lookahead = LOOKAHEAD_MIN_M;

  LocalCoords target = g_route[min((int)g_routeCount - 1, bestSeg + 1)].pos;
  float walk = lookahead;
  LocalCoords cursor = closest;
  for (uint8_t i = bestSeg; i + 1 < g_routeCount; i++) {
    LocalCoords end = g_route[i + 1].pos;
    float segLen = hypotf(end.x - cursor.x, end.y - cursor.y);
    if (segLen >= walk && segLen > 0.001f) {
      float ratio = walk / segLen;
      target = {cursor.x + (end.x - cursor.x) * ratio,
                cursor.y + (end.y - cursor.y) * ratio};
      break;
    }
    walk -= segLen;
    cursor = end;
    target = end;
  }

  // Heading convention: 0 deg = North (+Y), 90 deg = East (+X), clockwise.
  // For local coordinates x=east, y=north this is atan2(dx, dy).
  const float dxToTarget = target.x - g_est.pos.x;
  const float dyToTarget = target.y - g_est.pos.y;
  float desiredHeading = atan2f(dxToTarget, dyToTarget) * 180.0f / PI;
  if (desiredHeading < 0) desiredHeading += 360.0f;

  float headingError = normalizeAngle(desiredHeading - g_est.heading);
  g_headingError = headingError;

  // Speed based on quality
  float speed = BASE_SPEED;
  switch (g_est.quality) {
    case QUAL_RTK_FIXED_GOOD: speed = MAX_SPEED; break;
    case QUAL_RTK_FLOAT_OK: speed = FLOAT_SPEED; break;
    case QUAL_GPS_DEGRADED: speed = DEGRADED_SPEED; break;
    case QUAL_GPS_HOLD_SHORT: speed = HOLD_SPEED; break;
    default: speed = 0; break;
  }

  // Reduce speed for sharp turns and final approach.
  if (fabsf(headingError) > 45) speed *= 0.6f;
  if (fabsf(headingError) > 90) speed *= 0.3f;
  if (remainingOnPath < 1.0f) speed *= (0.35f + remainingOnPath * 0.65f);
  if (g_est.quality == QUAL_GPS_HOLD_SHORT && fabsf(headingError) > 15) speed = 0.0f;

  float forwardCmd = (speed / MAX_SPEED) * MAX_SPEED_PERCENT * g_navForwardScale;
  // headingError > 0 means the target is clockwise/right of current heading.
  // For tank drive, right turn means left wheel faster than right wheel.
  float turnCmd = K_HEADING * headingError * g_navTurnScale;
  // Add gentle crosstrack correction (don't override heading)
  turnCmd += K_CROSSTRACK * g_crossTrackError * 0.1f * g_navTurnScale;
  turnCmd = constrain(turnCmd, -MAX_SPEED_PERCENT * 0.5f, MAX_SPEED_PERCENT * 0.5f);
  turnCmd = constrain(turnCmd, -MAX_SPEED_PERCENT * 0.65f, MAX_SPEED_PERCENT * 0.65f);

  if (g_invertForward) forwardCmd = -forwardCmd;
  if (g_invertSteering) turnCmd = -turnCmd;

  int16_t left = (int16_t)(forwardCmd + turnCmd);
  int16_t right = (int16_t)(forwardCmd - turnCmd);

  // Same as sound.ino: percent domain
  g_targetLeft = constrain(left, -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);
  g_targetRight = constrain(right, -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);
  g_lastCmdMs = now;
  g_isFailSafeStopping = false;
  g_navReason = qualityString(g_est.quality);

  // Debug nav: log every 2 seconds
  static uint32_t lastNavDebug = 0;
  if (now - lastNavDebug > 2000) {
    lastNavDebug = now;
    Serial.printf("NAV: pos=(%.2f,%.2f) heading=%.1f imuYaw=%.1f target=(%.2f,%.2f) desiredHdg=%.1f hdgErr=%.1f turnCmd=%.1f L=%d R=%d\n",
      g_est.pos.x, g_est.pos.y, g_est.heading, g_imuYaw,
      target.x, target.y, desiredHeading, headingError,
      turnCmd, g_targetLeft, g_targetRight);
  }
}

// ============== MOTOR CONTROL (same as sound.ino) ==============

static inline int16_t limitForAxis(int16_t cur, int16_t target, int ticks) {
  int absCur = abs((int)cur);
  int absTgt = abs((int)target);
  bool up = (absTgt > absCur);
  int step = up ? RAMP_STEP_UP_PER_TICK : RAMP_STEP_DOWN_PER_TICK;
  int maxDelta = ticks * step;
  if (maxDelta < 1) maxDelta = 1;
  return maxDelta;
}

static void updateRamp() {
  uint32_t now = millis();
  uint32_t dt = now - g_lastRampMs;
  if (dt < RAMP_UPDATE_MS) return;

  uint32_t ticks = dt / RAMP_UPDATE_MS;
  g_lastRampMs += ticks * RAMP_UPDATE_MS;

  int16_t tL = constrain(g_targetLeft, -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);
  int16_t tR = constrain(g_targetRight, -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);

  int limL = limitForAxis(g_curLeft, tL, (int)ticks);
  int limR = limitForAxis(g_curRight, tR, (int)ticks);

  g_curLeft = constrain(stepToward(g_curLeft, tL, limL), -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);
  g_curRight = constrain(stepToward(g_curRight, tR, limR), -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);
}

// left/right (-70..70) -> speed/steer -> SLEW -> send
static void drive(int16_t leftPct, int16_t rightPct) {
  leftPct = constrain(leftPct, -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);
  rightPct = constrain(rightPct, -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);

  int32_t speedT = (int32_t)(leftPct + rightPct) * (int32_t)HOVER_MAX_CMD / (2 * MAX_SPEED_PERCENT);
  int32_t steerT = (int32_t)(rightPct - leftPct) * (int32_t)HOVER_MAX_CMD / (2 * MAX_SPEED_PERCENT);

  int16_t spT = constrain(speedT, -HOVER_MAX_CMD, HOVER_MAX_CMD);
  int16_t stT = constrain(steerT, -HOVER_MAX_CMD, HOVER_MAX_CMD);

  g_cmdSpeed = stepToward(g_cmdSpeed, spT, SLEW_SPEED_PER_SEND);
  g_cmdSteer = stepToward(g_cmdSteer, stT, SLEW_STEER_PER_SEND);

  SerialCommand cmd;
  cmd.start = (uint16_t)0xABCD;
  cmd.steer = g_cmdSteer;
  cmd.speed = g_cmdSpeed;
  cmd.checksum = (uint16_t)(cmd.start ^ cmd.steer ^ cmd.speed);
  MotorSerial.write((uint8_t*)&cmd, sizeof(cmd));
}

static void motorsSend() {
  static uint32_t lastSend = 0;
  uint32_t now = millis();
  if (now - lastSend < HOVER_SEND_MS) return;
  lastSend = now;

  static bool init = false;
  if (!init) {
    MotorSerial.begin(MOTOR_BAUD, SERIAL_8N1, MOTOR_RX, MOTOR_TX);
    init = true;
    Serial.println("MOTOR: UART2 initialized");
  }

  drive(g_curLeft, g_curRight);
}

static void motorsReceiveFeedback() {
  static bool init = false;
  static SerialFeedback feedback{};
  static SerialFeedback incoming{};
  static uint8_t idx = 0;
  static uint8_t* p = nullptr;
  static uint8_t prev = 0;

  if (!init) {
    MotorSerial.begin(MOTOR_BAUD, SERIAL_8N1, MOTOR_RX, MOTOR_TX);
    init = true;
  }

  while (MotorSerial.available()) {
    uint8_t b = (uint8_t)MotorSerial.read();
    uint16_t start = ((uint16_t)b << 8) | prev;

    if (start == 0xABCD) {
      p = (uint8_t*)&incoming;
      *p++ = prev;
      *p++ = b;
      idx = 2;
    } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {
      *p++ = b;
      idx++;
    }

    if (idx == sizeof(SerialFeedback)) {
      uint16_t checksum = (uint16_t)(
        incoming.start ^
        incoming.cmd1 ^
        incoming.cmd2 ^
        incoming.speedR_meas ^
        incoming.speedL_meas ^
        incoming.batVoltage ^
        incoming.boardTemp ^
        incoming.cmdLed
      );
      if (incoming.start == 0xABCD && checksum == incoming.checksum) {
        feedback = incoming;
        g_haveMotorFeedback = true;
        g_motorFbCmd1 = feedback.cmd1;
        g_motorFbCmd2 = feedback.cmd2;
        g_motorSpeedR = feedback.speedR_meas;
        g_motorSpeedL = feedback.speedL_meas;
        g_motorBatVoltage = feedback.batVoltage;
        g_motorBoardTemp = feedback.boardTemp;
      }
      idx = 0;
    }
    prev = b;
  }
}

// ============== STATUS ==============

static void printStatus() {
  static uint32_t lastStatus = 0;
  uint32_t now = millis();
  if (now - lastStatus < STATUS_MS) return;
  lastStatus = now;

  Serial.println();
  Serial.println("========== ROVER STATUS ==========");
  Serial.printf("Uptime: %lus\n", (unsigned long)(now / 1000));

  Serial.println("-- WiFi --");
  Serial.printf("  %s\n", g_wifiConnected ? WiFi.localIP().toString().c_str() : "DISCONNECTED");
  if (g_wifiConnected) Serial.printf("  RSSI: %d dBm\n", WiFi.RSSI());

  Serial.println("-- GPS --");
  Serial.printf("  Fix: %u, Carrier: %u (%s)\n",
    g_gps.fixType, g_gps.carrier,
    g_gps.carrier == 2 ? "FIXED" : (g_gps.carrier == 1 ? "FLOAT" : "NONE"));
  Serial.printf("  Pos: %.8f, %.8f\n", g_gps.lat, g_gps.lon);
  Serial.printf("  hAcc: %.0f mm, Age: %lu ms\n", g_gps.hAcc, (unsigned long)g_est.qualityAgeMs);
  Serial.printf("  Raw: %lu, UBX: %lu, NMEA: %lu\n",
    (unsigned long)g_gpsRawBytes,
    (unsigned long)g_gpsUbxParsed,
    (unsigned long)g_gpsNmeaParsed);

  Serial.println("-- IMU --");
  Serial.printf("  Raw yaw: %.1f deg, Heading: %.1f deg, Offset: %.1f deg, Fresh: %u, Age: %lu ms\n",
    g_imuRawYaw, g_imuYaw, g_imuCalibrationOffset, g_imuFresh ? 1 : 0,
    (unsigned long)(g_lastImuMs ? now - g_lastImuMs : 0));

  Serial.println("-- RTCM --");
  Serial.printf("  Fresh: %u, Bytes: %lu, Msgs: %lu, Type: %lu\n",
    g_rtcmFresh ? 1 : 0,
    (unsigned long)g_rtcmBytes,
    (unsigned long)g_rtcmMsgs,
    (unsigned long)g_rtcmLastType);
  Serial.printf("  Relay errors: read=%lu write=%lu oversize=%lu\n",
    (unsigned long)g_rtcmErrors,
    (unsigned long)g_rtcmWriteErrors,
    (unsigned long)g_rtcmOversize);
  Serial.printf("  F9P decoded: msgs=%lu crcFail=%lu age=%lums\n",
    (unsigned long)g_f9pRtcmMsgs,
    (unsigned long)g_f9pRtcmCrcFail,
    (unsigned long)(g_lastF9pRtcmMs ? now - g_lastF9pRtcmMs : 0));

  Serial.println("-- Estimator --");
  Serial.printf("  Local: (%.2f, %.2f) m\n", g_est.pos.x, g_est.pos.y);
  Serial.printf("  Heading: %.1f deg, Speed: %.3f m/s\n", g_est.heading, g_est.speed);
  Serial.printf("  Quality: %s\n", qualityString(g_est.quality));

  Serial.println("-- Navigation --");
  Serial.printf("  State: %s, Reason: %s\n", stateString(g_navState), g_navReason);
  Serial.printf("  Route: %u/%u, XTE: %.2f m, headingErr: %.1f deg, remain: %.2f m\n",
    g_routeIndex, g_routeCount, g_crossTrackError, g_headingError, g_distToRouteEnd);
  Serial.printf("  Motor: L=%d R=%d (target)\n", g_targetLeft, g_targetRight);
  Serial.printf("        L=%d R=%d (current)\n", g_curLeft, g_curRight);
  Serial.printf("  Motor FB: fresh=%u cmd=(%d,%d) speed=(%d,%d) bat=%d temp=%d\n",
    g_haveMotorFeedback ? 1 : 0,
    g_motorFbCmd1, g_motorFbCmd2,
    g_motorSpeedL, g_motorSpeedR,
    g_motorBatVoltage, g_motorBoardTemp);

  Serial.println("-- Origin --");
  if (g_origin.valid) {
    Serial.printf("  Lat: %.8f, Lon: %.8f\n", g_origin.lat, g_origin.lon);
  } else {
    Serial.println("  Not set");
  }

  Serial.println();
  Serial.printf("Free heap: %lu bytes\n", (unsigned long)ESP.getFreeHeap());
  Serial.printf("===================================\n");
}

// ============== MAIN ==============

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("========================================");
  Serial.println("   RTK ROVER AUTOPILOT v2.0");
  Serial.println("========================================");
  Serial.printf("GPS UART: RX=%d TX=%d %lu baud\n", GPS_RX, GPS_TX, (unsigned long)GPS_BAUD);
  Serial.printf("Motor UART: RX=%d TX=%d %lu baud\n", MOTOR_RX, MOTOR_TX, (unsigned long)MOTOR_BAUD);
  Serial.printf("IMU I2C: SDA=%d SCL=%d\n", IMU_SDA, IMU_SCL);
  Serial.printf("WebSocket port: %u\n", WS_PORT);
  Serial.printf("Chip: %s rev=%d\n", ESP.getChipModel(), ESP.getChipRevision());

  g_prefs.begin("rover-nav", false);
  loadNavPrefs();
  Serial.printf("NAV prefs: imuOffset=%.1f invYaw=%u invFwd=%u invSteer=%u fwd=%.2f turn=%.2f\n",
    g_imuCalibrationOffset, g_invertYaw ? 1 : 0, g_invertForward ? 1 : 0,
    g_invertSteering ? 1 : 0, g_navForwardScale, g_navTurnScale);

  // GPS
  GpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  delay(200);
  Serial.println("GPS UART initialized");
  configureGpsRover();

  // IMU
  if (initImu()) {
    Serial.println("IMU: OK");
  } else {
    Serial.println("IMU: FAILED - continuing without IMU");
  }

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.config(ROVER_IP, GATEWAY, SUBNET);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.println("WiFi: connecting...");
}

void loop() {
  uint32_t now = millis();

  // WiFi
  if (WiFi.status() == WL_CONNECTED) {
    if (!g_wifiConnected) {
      g_wifiConnected = true;
      Serial.println();
      Serial.println("!!! WiFi CONNECTED !!!");
      Serial.printf("   IP: %s\n", WiFi.localIP().toString().c_str());
      Serial.printf("   RSSI: %d dBm\n", WiFi.RSSI());

      rtcmUdp.begin(RTCM_UDP_PORT);
      Serial.printf("RTCM: UDP port %u\n", RTCM_UDP_PORT);

      if (!g_serverStarted) {
        ws.onEvent(handleWsEvent);
        server.addHandler(&ws);
        server.on("/ping", HTTP_GET, [](AsyncWebServerRequest* req) { req->send(200); });
        server.begin();
        g_serverStarted = true;
        Serial.printf("WebSocket: port %u\n", WS_PORT);
      }
    }
  } else {
    if (g_wifiConnected) {
      g_wifiConnected = false;
      Serial.println("!!! WiFi DISCONNECTED !!!");
    }
  }

  // GPS parsing
  while (GpsSerial.available()) {
    feedGpsByte(GpsSerial.read());
  }

  // RTCM relay
  if (g_wifiConnected) {
    relayRtcm();
  }

  maintainGpsOutput();

  // IMU
  updateImu();

  // Estimator
  updateEstimator();

  // Navigation
  static uint32_t lastNav = 0;
  if (now - lastNav >= NAV_LOOP_MS) {
    lastNav = now;
    navUpdate();
  }

  // Failsafe -> smooth stop (same as sound.ino)
  if (now - g_lastCmdMs > CMD_TIMEOUT_MS) {
    if (!g_isFailSafeStopping && (g_targetLeft != 0 || g_targetRight != 0 || g_curLeft != 0 || g_curRight != 0)) {
      g_isFailSafeStopping = true;
      Serial.println("FAILSAFE: smooth stop");
    }
  }

  // Motor control (same as sound.ino)
  static uint32_t lastMotorUpdate = 0;
  if (now - lastMotorUpdate >= MOTOR_SEND_MS) {
    lastMotorUpdate = now;
    updateRamp();
    motorsSend();
  }
  motorsReceiveFeedback();

  // Telemetry
  if (g_wifiConnected) {
    broadcastTelemetry();
  }

  // Status
  printStatus();

  // Yield to let WiFi/AsyncWebServer process
  yield();
}
