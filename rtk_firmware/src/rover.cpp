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
static constexpr bool ENABLE_RTCM_TCP_FALLBACK = true;
static constexpr uint32_t GPS_BROADCAST_MS = 200;
static constexpr uint32_t LEGACY_BROADCAST_MS = 1000;
static constexpr uint32_t STATUS_MS = 3000;
static constexpr uint32_t WIFI_RETRY_MS = 5000;
static constexpr uint32_t WIFI_STARTUP_DELAY_MS = 3000;
static constexpr uint32_t RTCM_TCP_CONNECT_TIMEOUT_MS = 2000;
static constexpr uint32_t RTCM_TCP_RETRY_MS = 5000;
static constexpr uint32_t RTCM_TCP_PRIMARY_HOLD_MS = 2500;
static constexpr uint32_t RTCM_WARN_AGE_MS = 7000;
static constexpr uint32_t RTCM_RESTART_AGE_MS = 10000;
static constexpr uint32_t RTCM_TRANSPORT_RECOVER_AGE_MS = 25000;
static constexpr uint32_t RTCM_TRANSPORT_RECOVER_MS = 10000;
static constexpr uint32_t RTCM_WIFI_RECOVER_AGE_MS = 60000;
static constexpr uint32_t RTCM_WIFI_RECOVER_MS = 45000;
static constexpr wifi_power_t ROVER_WIFI_TX_POWER = WIFI_POWER_15dBm;
static constexpr int PIN_MOTOR_RX = 16;
static constexpr int PIN_MOTOR_TX = 17;
static constexpr int PIN_IMU_SDA = 21;
static constexpr int PIN_IMU_SCL = 22;
static constexpr uint32_t MOTOR_BAUD = 115200;
static constexpr uint32_t IMU_BROADCAST_MS = 1000;
static constexpr uint32_t IMU_REPORT_INTERVAL_US = 50000;
static constexpr uint32_t IMU_NOT_FOUND_RETRY_MS = 10000;
static constexpr uint32_t IMU_STALE_REENABLE_MS = 1500;
static constexpr uint32_t IMU_STALE_RESET_MS = 15000;
static constexpr uint32_t MOTOR_SEND_MS = 20;
static constexpr uint32_t MOTOR_RAMP_MS = 20;
static constexpr uint32_t MOTOR_CMD_TIMEOUT_MS = 400;
static constexpr int16_t MAX_SPEED_PERCENT = 70;
static constexpr int16_t HOVER_MAX_CMD = 500;
static constexpr int16_t INPUT_DIV = 1;
static constexpr int16_t RAMP_STEP_PER_TICK = 1;
static constexpr int16_t SLEW_SPEED_PER_SEND = 4;
static constexpr int16_t SLEW_STEER_PER_SEND = 6;
static constexpr uint16_t HOVER_START_FRAME = 0xABCD;
static constexpr uint8_t NAV_MAX_WAYPOINTS = 128;
static constexpr uint32_t NAV_LOOP_MS = 100;
static constexpr uint32_t NAV_BROADCAST_MS = 500;
static constexpr uint32_t NAV_MAX_GPS_AGE_MS = 5000;
static constexpr uint32_t NAV_MAX_RTCM_AGE_MS = 60000;
static constexpr uint32_t NAV_MAX_IMU_AGE_MS = 5000;
static constexpr uint32_t NAV_PRECISE_HACC_MM = 120;
static constexpr uint32_t NAV_USABLE_HACC_MM = 300;
static constexpr uint32_t NAV_DEGRADED_HACC_MM = 900;
static constexpr float NAV_MIN_GPS_COURSE_SPEED_MPS = 0.12f;
static constexpr float NAV_ARRIVED_M = 0.45f;
static constexpr float NAV_PASS_WP_M = 0.85f;
static constexpr float NAV_PIVOT_ERROR_DEG = 75.0f;
static constexpr float NAV_LOOKAHEAD_MIN_M = 0.80f;
static constexpr float NAV_LOOKAHEAD_MAX_M = 2.40f;
static constexpr float NAV_MOTION_SAMPLE_M = 0.30f;
static constexpr float NAV_BAD_MOTION_DEG = 115.0f;
static constexpr float NAV_CROSSTRACK_SLOW_M = 0.80f;
static constexpr float NAV_CLOSE_SLOW_M = 1.40f;
static constexpr uint32_t NAV_PROGRESS_CHECK_MS = 3000;
static constexpr uint32_t NAV_RECOVERY_MS = 1800;
static constexpr float NAV_MIN_PROGRESS_M = 0.08f;

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

struct NavWaypoint {
  double lat = 0.0;
  double lon = 0.0;
};

struct NavLegMetrics {
  float lengthM = 0.0f;
  float alongM = 0.0f;
  float crossTrackM = 0.0f;
  float remainingM = 0.0f;
  float aimBearing = 0.0f;
};

enum NavQuality : uint8_t {
  NAV_Q_NONE = 0,
  NAV_Q_DEGRADED = 1,
  NAV_Q_USABLE = 2,
  NAV_Q_PRECISE = 3,
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
static uint32_t wsTelemetryDropCount = 0;
static uint32_t lastStatusMs = 0;
static uint32_t lastWiFiAttemptMs = 0;
static uint32_t wifiStartAllowedMs = 0;
static uint32_t wifiDisconnectedSinceMs = 0;
static bool wasWiFiConnected = false;
static bool networkServicesStarted = false;
static bool otaCallbacksConfigured = false;
static uint32_t wifiReconnectCount = 0;
static uint32_t udpRestartCount = 0;
static uint32_t lastRtcmWarnMs = 0;
static uint32_t lastRtcmUdpRestartMs = 0;
static uint32_t lastRtcmTransportRecoverMs = 0;
static uint32_t lastRtcmWifiRecoverMs = 0;
static uint32_t lastLegacyBroadcastMs = 0;
static uint32_t lastRtcmTcpAttemptMs = 0;
static uint32_t lastRtcmTcpMs = 0;
static uint32_t rtcmTcpReconnectCount = 0;
static uint32_t rtcmTcpBytesRx = 0;
static uint32_t rtcmTcpReads = 0;
static bool rtcmUdpActive = false;
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
static NavWaypoint navRoute[NAV_MAX_WAYPOINTS];
static bool navRouteReceived[NAV_MAX_WAYPOINTS];
static uint8_t navRouteCount = 0;
static uint8_t navRouteExpected = 0;
static uint8_t navRouteReceivedCount = 0;
static bool navRouteUploading = false;
static bool navRouteReady = false;
static bool navRunning = false;
static bool navPaused = false;
static uint8_t navWpIndex = 0;
static NavWaypoint navStartPoint;
static bool navStartPointValid = false;
static uint32_t lastNavLoopMs = 0;
static uint32_t lastNavBroadcastMs = 0;
static float navLastHeading = 0.0f;
static float navLastDistanceM = 0.0f;
static float navLastBearing = 0.0f;
static float navLastHeadingError = 0.0f;
static float navLastCrossTrackM = 0.0f;
static float navLastAlongTrackM = 0.0f;
static float navLastMotionBearing = 0.0f;
static float navLastMotionError = 0.0f;
static bool navMotionValid = false;
static bool navMotionSeeded = false;
static NavWaypoint navMotionSamplePoint;
static uint32_t navLastMotionMs = 0;
static uint32_t navProgressCheckMs = 0;
static float navProgressCheckDistanceM = 0.0f;
static float navBestDistanceM = 9999.0f;
static uint8_t navStallCount = 0;
static uint8_t navBadMotionCount = 0;
static uint32_t navRecoveryUntilMs = 0;
static int8_t navRecoveryDir = 1;
static int16_t navForwardPercent = 22;
static int16_t navTurnPercent = 18;
static bool navInvertForward = false;
static bool navInvertSteering = false;
static float navHeadingOffsetDeg = 0.0f;
static bool navInvertYaw = false;
static const char* navState = "IDLE";
static const char* navReason = "idle";

static Adafruit_BNO08x bno08x;
static sh2_SensorValue_t imuValue;
static bool imuDetected = false;
static bool imuValid = false;
static float imuYaw = 0.0f;
static float imuRotationYaw = 0.0f;
static float imuGameYaw = 0.0f;
static uint32_t lastImuMs = 0;
static uint32_t lastImuRotationMs = 0;
static uint32_t lastImuGameMs = 0;
static uint32_t lastImuRecoverMs = 0;
static uint32_t lastImuHardResetMs = 0;
static const char* imuYawSource = "none";

static bool imuFresh(uint32_t now) {
  return imuDetected && imuValid && lastImuMs != 0 && now - lastImuMs <= 5000;
}

static void motorsSetTarget(int left, int right);
static void startMotorTask();

static uint32_t rtcmTransportAgeMs(uint32_t now) {
  if (lastRtcmMs != 0 && now >= lastRtcmMs) {
    return now - lastRtcmMs;
  }
  return 0;
}

static uint32_t rtcmF9pAgeMs(uint32_t now) {
  if (f9pLastRtcmMs != 0 && now >= f9pLastRtcmMs) {
    return now - f9pLastRtcmMs;
  }
  return 0;
}

static uint32_t effectiveRtcmAgeMs(uint32_t now) {
  const uint32_t f9pAge = rtcmF9pAgeMs(now);
  if (f9pAge != 0) return f9pAge;
  return rtcmTransportAgeMs(now);
}

static bool broadcastTelemetry(const char* msg) {
  if (ws.count() == 0) return false;
  if (!ws.availableForWriteAll()) {
    wsTelemetryDropCount++;
    return false;
  }
  ws.textAll(msg);
  return true;
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

static float normalizeDeg(float deg) {
  while (deg < 0.0f) deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;
  return deg;
}

static float headingErrorDeg(float current, float target) {
  float err = normalizeDeg(target) - normalizeDeg(current);
  if (err > 180.0f) err -= 360.0f;
  if (err < -180.0f) err += 360.0f;
  return err;
}

static float degToRad(float deg) {
  return deg * PI / 180.0f;
}

static void localMeters(double originLat, double originLon, double lat,
                        double lon, float* eastM, float* northM) {
  const double latRad = originLat * PI / 180.0;
  const double metersPerDegLat = 111132.92 -
                                 559.82 * cos(2.0 * latRad) +
                                 1.175 * cos(4.0 * latRad);
  const double metersPerDegLon = 111412.84 * cos(latRad) -
                                 93.5 * cos(3.0 * latRad);
  *eastM = (float)((lon - originLon) * metersPerDegLon);
  *northM = (float)((lat - originLat) * metersPerDegLat);
}

static float distanceMeters(double lat1, double lon1, double lat2, double lon2) {
  const double earthRadiusM = 6371008.8;
  const double dLat = (lat2 - lat1) * PI / 180.0;
  const double dLon = (lon2 - lon1) * PI / 180.0;
  const double rLat1 = lat1 * PI / 180.0;
  const double rLat2 = lat2 * PI / 180.0;
  const double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
                   cos(rLat1) * cos(rLat2) *
                       sin(dLon / 2.0) * sin(dLon / 2.0);
  const double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return (float)(earthRadiusM * c);
}

static float bearingDeg(double fromLat, double fromLon, double toLat,
                        double toLon) {
  const double fromLatRad = fromLat * PI / 180.0;
  const double toLatRad = toLat * PI / 180.0;
  const double dLon = (toLon - fromLon) * PI / 180.0;
  const double y = sin(dLon) * cos(toLatRad);
  const double x = cos(fromLatRad) * sin(toLatRad) -
                   sin(fromLatRad) * cos(toLatRad) * cos(dLon);
  return normalizeDeg((float)(atan2(y, x) * 180.0 / PI));
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
  if (!rtcmUdpActive) return false;
  uint8_t buf[1200];
  uint8_t packetsHandled = 0;
  const uint32_t now = millis();
  const bool tcpPrimaryFresh = ENABLE_RTCM_TCP_FALLBACK &&
                               rtcmTcpClient.connected() &&
                               lastRtcmTcpMs != 0 &&
                               now - lastRtcmTcpMs < RTCM_TCP_PRIMARY_HOLD_MS;

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
      if (!tcpPrimaryFresh) {
        GpsSerial.write(buf, len);
      }
      written += len;
      remaining -= len;
    }

    if (written > 0 && !tcpPrimaryFresh) {
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
static void startNetworkServices();

static void connectRtcmTcp() {
  if (!ENABLE_RTCM_TCP_FALLBACK) return;
  if (!ROUTER_MODE || WiFi.status() != WL_CONNECTED) return;
  if (rtcmTcpClient.connected()) return;

  const uint32_t now = millis();
  if (lastRtcmTcpAttemptMs != 0 &&
      now - lastRtcmTcpAttemptMs < RTCM_TCP_RETRY_MS) {
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
  if (!ENABLE_RTCM_TCP_FALLBACK) return;
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
  if (ROUTER_MODE && WiFi.status() != WL_CONNECTED) {
    if (rtcmUdpActive) {
      rtcmUdp.stop();
      rtcmUdpActive = false;
    }
    Serial.printf("RTCM UDP deferred reason=%s\n", reason);
    return;
  }
  rtcmUdp.stop();
  rtcmUdpActive = false;
  delay(20);
  const uint8_t ok = rtcmUdp.begin(RTCM_UDP_PORT);
  rtcmUdpActive = ok == 1;
  udpRestartCount++;
  lastRtcmUdpRestartMs = millis();
  Serial.printf("RTCM UDP restart #%lu port=%u ok=%u reason=%s\n",
                (unsigned long)udpRestartCount, RTCM_UDP_PORT, ok, reason);
}

static void recoverRtcmTransport(const char* reason) {
  const uint32_t now = millis();
  if (now - lastRtcmTransportRecoverMs < RTCM_TRANSPORT_RECOVER_MS) return;
  lastRtcmTransportRecoverMs = now;
  Serial.printf("WARN rover RTCM transport recover reason=%s age=%lums\n", reason,
                (unsigned long)(lastRtcmMs == 0 ? 0 : now - lastRtcmMs));
  if (ENABLE_RTCM_TCP_FALLBACK) {
    rtcmTcpClient.stop();
  }
  lastRtcmTcpAttemptMs = 0;
  restartRtcmUdp(reason);
}

static void recoverRtcmWiFi(const char* reason) {
  const uint32_t now = millis();
  if (now - lastRtcmWifiRecoverMs < RTCM_WIFI_RECOVER_MS) return;
  lastRtcmWifiRecoverMs = now;
  recoverRtcmTransport(reason);
  Serial.printf("WARN rover RTCM WiFi hard recover reason=%s age=%lums\n", reason,
                (unsigned long)(lastRtcmMs == 0 ? 0 : now - lastRtcmMs));
  connectWiFi(true);
}

static void checkRtcmWatchdog() {
  const uint32_t now = millis();
  const uint32_t transportAge = rtcmTransportAgeMs(now);
  const uint32_t f9pAge = rtcmF9pAgeMs(now);
  if (transportAge == 0) {
    if (lastRtcmUdpRestartMs == 0) return;
  }
  const uint32_t watchdogAge =
      transportAge != 0 ? transportAge : now - lastRtcmUdpRestartMs;
  if (watchdogAge > RTCM_WARN_AGE_MS && now - lastRtcmWarnMs > 1000) {
    lastRtcmWarnMs = now;
    Serial.printf("WARN rover RTCM transportAge=%lums f9pAge=%lums packets=%lu udpRestart=%lu src=%s seen=%u\n",
                  (unsigned long)watchdogAge, (unsigned long)f9pAge,
                  (unsigned long)rtcmPacketsRx, (unsigned long)udpRestartCount,
                  rtcmInputSource, lastRtcmMs != 0 ? 1 : 0);
  }
  if (watchdogAge > RTCM_RESTART_AGE_MS &&
      now - lastRtcmUdpRestartMs > WIFI_RETRY_MS) {
    restartRtcmUdp("rtcm-age");
    if (ENABLE_RTCM_TCP_FALLBACK) {
      rtcmTcpClient.stop();
      lastRtcmTcpAttemptMs = 0;
    }
  }
  if (watchdogAge > RTCM_TRANSPORT_RECOVER_AGE_MS) {
    recoverRtcmTransport("rtcm-stale");
  }
  if (lastRtcmMs != 0 && watchdogAge > RTCM_WIFI_RECOVER_AGE_MS) {
    recoverRtcmWiFi("rtcm-stale");
  }
  if (transportAge != 0 && transportAge <= RTCM_WARN_AGE_MS && f9pAge != 0 &&
      f9pAge > RTCM_RESTART_AGE_MS &&
      now - lastRoverConfigRetryMs >= 10000) {
    lastRoverConfigRetryMs = now;
    Serial.printf("WARN rover RTCM reaches ESP32 but F9P age=%lums; reconfiguring UART protocols\n",
                  (unsigned long)f9pAge);
    configureRover();
  }
}

static void checkF9pRtcmWatchdog() {
  const uint32_t now = millis();
  if (gpsParsedMessages == 0) return;
  if (gps.diff || gps.carrier != 0) return;
  if (rtcmPacketsRx < 10 || f9pRtcmMessages > 0) return;
  if (now - lastRoverConfigRetryMs < 10000) return;
  lastRoverConfigRetryMs = now;
  Serial.printf("WARN rover RTCM reaches ESP32 but F9P reports no RTCM; reconfiguring UART protocols src=%s packets=%lu bytes=%lu\n",
                rtcmInputSource, (unsigned long)rtcmPacketsRx,
                (unsigned long)rtcmBytesRx);
  configureRover();
}

static void broadcastGps() {
  const uint32_t now = millis();
  if (now - lastGpsBroadcastMs < GPS_BROADCAST_MS) return;
  lastGpsBroadcastMs = now;

  const uint32_t gpsAgeMs = gps.lastMs == 0 ? 0 : now - gps.lastMs;
  const uint32_t rtcmAgeMs = effectiveRtcmAgeMs(now);
  const uint32_t rtcmTransportAge = rtcmTransportAgeMs(now);
  const uint32_t rtcmF9pAge = rtcmF9pAgeMs(now);
  const uint32_t imuAgeMs = lastImuMs == 0 ? 0 : now - lastImuMs;
  const bool freshImu = imuFresh(now);

  char tel[320];
  snprintf(tel, sizeof(tel),
           "TEL,%.8f,%.8f,%.3f,%.2f,%u,%s,%u,%u,%lu,%lu,%.3f,%.2f,%lu,%lu,%lu,%.2f,%lu,%u,%lu,%lu,%s,%lu,%lu,%u",
           gps.lat, gps.lon, gps.heightM, gps.heading, gps.fixType,
           carrierName(gps.carrier), gps.diff ? 1 : 0, gps.numSv,
           (unsigned long)gps.hAccMm, (unsigned long)gps.vAccMm,
           gps.speedMps, gps.pDop, (unsigned long)gpsAgeMs,
           (unsigned long)rtcmBytesRx, (unsigned long)rtcmAgeMs,
           imuYaw, (unsigned long)imuAgeMs, freshImu ? 1 : 0,
           (unsigned long)rtcmTransportAge, (unsigned long)rtcmF9pAge,
           rtcmInputSource, (unsigned long)f9pRtcmMessages,
           (unsigned long)f9pRtcmCrcFail, f9pLastRtcmType);
  broadcastTelemetry(tel);

  if (now - lastLegacyBroadcastMs < LEGACY_BROADCAST_MS) return;
  lastLegacyBroadcastMs = now;

  char msg[192];
  snprintf(msg, sizeof(msg), "GPS,%.8f,%.8f,%.2f,%u,%lu",
           gps.lat, gps.lon, gps.heading, gps.fixType,
           (unsigned long)gps.hAccMm);
  broadcastTelemetry(msg);

  snprintf(msg, sizeof(msg),
           "GPSDBG,%.8f,%.8f,%.3f,%.2f,%u,%s,%u,%u,%lu,%lu,%.3f,%.2f,%lu",
           gps.lat, gps.lon, gps.heightM, gps.heading, gps.fixType,
           carrierName(gps.carrier), gps.diff ? 1 : 0, gps.numSv,
           (unsigned long)gps.hAccMm, (unsigned long)gps.vAccMm,
           gps.speedMps, gps.pDop, (unsigned long)gpsAgeMs);
  broadcastTelemetry(msg);

  snprintf(msg, sizeof(msg), "RTCM,%lu,%lu,%lu,%lu,%s,%lu,%lu,%u",
           (unsigned long)rtcmBytesRx, (unsigned long)rtcmAgeMs,
           (unsigned long)rtcmTransportAge, (unsigned long)rtcmF9pAge,
           rtcmInputSource, (unsigned long)f9pRtcmMessages,
           (unsigned long)f9pRtcmCrcFail, f9pLastRtcmType);
  broadcastTelemetry(msg);
}

static void connectWiFi(bool force) {
  if (!ROUTER_MODE) return;
  const uint32_t now = millis();
  if (!force && wifiStartAllowedMs != 0 && now < wifiStartAllowedMs) return;
  if (!force && WiFi.status() == WL_CONNECTED) return;

  if (!force && lastWiFiAttemptMs != 0 &&
      now - lastWiFiAttemptMs < WIFI_RETRY_MS) {
    return;
  }
  lastWiFiAttemptMs = now;
  wifiReconnectCount++;

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setTxPower(ROVER_WIFI_TX_POWER);
  WiFi.setAutoReconnect(true);
  WiFi.config(ROVER_STA_IP, ROUTER_GATEWAY, ROUTER_SUBNET);
  if (force) {
    rtcmTcpClient.stop();
    WiFi.disconnect(false);
    delay(50);
  }
  Serial.printf("WiFi STA reconnect #%lu to %s, static IP %s, tx=15dBm\n",
                (unsigned long)wifiReconnectCount, ROUTER_WIFI_SSID,
                ROVER_STA_IP.toString().c_str());
  Serial.flush();
  WiFi.begin(ROUTER_WIFI_SSID, ROUTER_WIFI_PASS);
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
    startNetworkServices();
    wifiDisconnectedSinceMs = 0;
    wasWiFiConnected = true;
    return;
  }

  if (wifiDisconnectedSinceMs == 0) wifiDisconnectedSinceMs = now;
  wasWiFiConnected = false;
  rtcmUdpActive = false;

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

static void navStop(const char* reason) {
  navRunning = false;
  navPaused = false;
  navState = "IDLE";
  navReason = reason == nullptr ? "stop" : reason;
  navMotionSeeded = false;
  navMotionValid = false;
  navRecoveryUntilMs = 0;
  motorsStop(reason);
}

static bool navHasRtcm(uint32_t now) {
  const uint32_t f9pAge = rtcmF9pAgeMs(now);
  if (f9pAge != 0) return f9pAge <= NAV_MAX_RTCM_AGE_MS;
  const uint32_t transportAge = rtcmTransportAgeMs(now);
  if (transportAge != 0) return transportAge <= NAV_MAX_RTCM_AGE_MS;
  return gps.diff || gps.carrier == 2;
}

static NavQuality navQuality(uint32_t now) {
  if (!gps.valid || gps.fixType < 3 || gps.lastMs == 0 ||
      now - gps.lastMs > NAV_MAX_GPS_AGE_MS) {
    return NAV_Q_NONE;
  }

  const bool rtcmOk = navHasRtcm(now);
  if (gps.carrier == 2 && gps.hAccMm <= NAV_PRECISE_HACC_MM && rtcmOk) {
    return NAV_Q_PRECISE;
  }
  if ((gps.carrier == 2 || gps.carrier == 1 || gps.diff) &&
      gps.hAccMm <= NAV_USABLE_HACC_MM && rtcmOk) {
    return NAV_Q_USABLE;
  }
  if (gps.hAccMm <= NAV_DEGRADED_HACC_MM) {
    return NAV_Q_DEGRADED;
  }
  return NAV_Q_NONE;
}

static const char* navQualityName(NavQuality q) {
  switch (q) {
    case NAV_Q_PRECISE:
      return "precise";
    case NAV_Q_USABLE:
      return "usable";
    case NAV_Q_DEGRADED:
      return "degraded";
    default:
      return "none";
  }
}

static bool navCurrentHeading(uint32_t now, float desiredBearing,
                              float* heading, const char** source) {
  if (imuDetected && imuValid && lastImuMs != 0 &&
      now - lastImuMs <= NAV_MAX_IMU_AGE_MS) {
    const float yaw = navInvertYaw ? -imuYaw : imuYaw;
    navLastHeading = normalizeDeg(yaw + navHeadingOffsetDeg);
    *heading = navLastHeading;
    *source = "imu";
    return true;
  }

  if (gps.speedMps >= NAV_MIN_GPS_COURSE_SPEED_MPS) {
    navLastHeading = normalizeDeg(gps.heading);
    *heading = navLastHeading;
    *source = "gps-course";
    return true;
  }

  if (navMotionValid) {
    navLastHeading = navLastMotionBearing;
    *heading = navLastHeading;
    *source = "gps-motion";
    return true;
  }

  // Bootstrap: start slow along the desired bearing until GPS/IMU gives motion.
  navLastHeading = normalizeDeg(desiredBearing);
  *heading = navLastHeading;
  *source = "bootstrap";
  return true;
}

static void navResetGuidanceState() {
  navMotionSeeded = false;
  navMotionValid = false;
  navLastMotionBearing = 0.0f;
  navLastMotionError = 0.0f;
  navProgressCheckMs = 0;
  navProgressCheckDistanceM = 0.0f;
  navBestDistanceM = 9999.0f;
  navStallCount = 0;
  navBadMotionCount = 0;
  navRecoveryUntilMs = 0;
  navRecoveryDir = 1;
}

static NavWaypoint navCurrentLegStart() {
  if (navWpIndex == 0) {
    return navStartPointValid ? navStartPoint : navRoute[0];
  }
  return navRoute[navWpIndex - 1];
}

static NavLegMetrics navComputeLegMetrics(const NavWaypoint& start,
                                          const NavWaypoint& target) {
  NavLegMetrics m;
  float segX = 0.0f;
  float segY = 0.0f;
  float curX = 0.0f;
  float curY = 0.0f;
  localMeters(start.lat, start.lon, target.lat, target.lon, &segX, &segY);
  localMeters(start.lat, start.lon, gps.lat, gps.lon, &curX, &curY);

  m.lengthM = sqrtf(segX * segX + segY * segY);
  if (m.lengthM < 0.10f) {
    m.alongM = 0.0f;
    m.crossTrackM = 0.0f;
    m.remainingM = navLastDistanceM;
    m.aimBearing = navLastBearing;
    return m;
  }

  const float ux = segX / m.lengthM;
  const float uy = segY / m.lengthM;
  m.alongM = curX * ux + curY * uy;
  m.crossTrackM = ux * curY - uy * curX;
  m.remainingM = m.lengthM - m.alongM;

  const float lookahead = fminf(
      NAV_LOOKAHEAD_MAX_M,
      fmaxf(NAV_LOOKAHEAD_MIN_M, 0.75f + fabsf(m.crossTrackM) * 0.55f));
  float targetAlong = m.alongM + lookahead;
  if (targetAlong < lookahead) targetAlong = lookahead;
  if (targetAlong > m.lengthM) targetAlong = m.lengthM;

  const float aimX = ux * targetAlong;
  const float aimY = uy * targetAlong;
  const float dx = aimX - curX;
  const float dy = aimY - curY;
  if (sqrtf(dx * dx + dy * dy) < 0.20f) {
    m.aimBearing = navLastBearing;
  } else {
    m.aimBearing = normalizeDeg((float)(atan2(dx, dy) * 180.0 / PI));
  }
  return m;
}

static void navUpdateMotion(uint32_t now, float desiredBearing) {
  if (!navMotionSeeded) {
    navMotionSamplePoint.lat = gps.lat;
    navMotionSamplePoint.lon = gps.lon;
    navMotionSeeded = true;
    navMotionValid = false;
    navLastMotionMs = now;
    return;
  }

  const float moved = distanceMeters(navMotionSamplePoint.lat,
                                     navMotionSamplePoint.lon, gps.lat, gps.lon);
  if (moved >= NAV_MOTION_SAMPLE_M) {
    navLastMotionBearing = bearingDeg(navMotionSamplePoint.lat,
                                      navMotionSamplePoint.lon, gps.lat, gps.lon);
    navLastMotionError = headingErrorDeg(navLastMotionBearing, desiredBearing);
    navMotionValid = true;
    navLastMotionMs = now;
    navMotionSamplePoint.lat = gps.lat;
    navMotionSamplePoint.lon = gps.lon;
    if (fabsf(navLastMotionError) > NAV_BAD_MOTION_DEG &&
        navLastDistanceM > NAV_PASS_WP_M) {
      if (navBadMotionCount < 5) navBadMotionCount++;
    } else if (navBadMotionCount > 0) {
      navBadMotionCount--;
    }
  } else if (now - navLastMotionMs > 5000) {
    navMotionValid = false;
  }
}

static void navUpdateProgress(uint32_t now) {
  if (navLastDistanceM < navBestDistanceM) navBestDistanceM = navLastDistanceM;
  if (navProgressCheckMs == 0) {
    navProgressCheckMs = now;
    navProgressCheckDistanceM = navLastDistanceM;
    navBestDistanceM = navLastDistanceM;
    return;
  }
  if (now - navProgressCheckMs < NAV_PROGRESS_CHECK_MS) return;

  const float progress = navProgressCheckDistanceM - navBestDistanceM;
  if (navLastDistanceM > NAV_PASS_WP_M &&
      fabsf(navLastHeadingError) < 65.0f &&
      progress < NAV_MIN_PROGRESS_M) {
    if (navStallCount < 5) navStallCount++;
  } else {
    navStallCount = 0;
  }
  navProgressCheckMs = now;
  navProgressCheckDistanceM = navLastDistanceM;
  navBestDistanceM = navLastDistanceM;
}

static void navMaybeEnterRecovery(uint32_t now) {
  if (now < navRecoveryUntilMs) return;
  if (navBadMotionCount >= 2 || navStallCount >= 2) {
    navRecoveryDir = navLastHeadingError >= 0.0f ? 1 : -1;
    navRecoveryUntilMs = now + NAV_RECOVERY_MS;
    navReason = navBadMotionCount >= 2 ? "recover-motion" : "recover-stall";
    navBadMotionCount = 0;
    navStallCount = 0;
  }
}

static void navSetMotor(uint32_t now, float errorDeg, float distanceM,
                        float crossTrackM, NavQuality quality,
                        const char* headingSource) {
  const int16_t maxForward = clampI16(navForwardPercent, 0, 45);
  const int16_t maxTurn = clampI16(navTurnPercent, 0, 40);
  const int16_t safeTurn = maxTurn > 0 ? maxTurn : 12;
  const float absError = fabs(errorDeg);
  const bool bootstrapHeading =
      headingSource != nullptr && strcmp(headingSource, "bootstrap") == 0;

  if (now < navRecoveryUntilMs) {
    const int16_t turn =
        (int16_t)(safeTurn * navRecoveryDir) * (navInvertSteering ? -1 : 1);
    motorsSetTarget(turn, -turn);
    return;
  }

  if (!bootstrapHeading && absError >= NAV_PIVOT_ERROR_DEG) {
    const int16_t turn =
        (int16_t)(safeTurn * (errorDeg > 0.0f ? 1 : -1)) *
        (navInvertSteering ? -1 : 1);
    motorsSetTarget(turn, -turn);
    return;
  }

  float forwardScale = 1.0f;
  if (quality == NAV_Q_DEGRADED) {
    forwardScale *= 0.35f;
  } else if (quality == NAV_Q_USABLE) {
    forwardScale *= 0.70f;
  }
  if (bootstrapHeading) {
    forwardScale *= 0.35f;
  }
  if (distanceM < NAV_CLOSE_SLOW_M) {
    forwardScale *= 0.35f + distanceM / NAV_CLOSE_SLOW_M * 0.65f;
  }
  if (absError > 55.0f) {
    forwardScale *= 0.25f;
  } else if (absError > 35.0f) {
    forwardScale *= 0.55f;
  }
  const float absCross = fabsf(crossTrackM);
  if (absCross > NAV_CROSSTRACK_SLOW_M) {
    forwardScale *= fmaxf(0.35f, 1.0f - (absCross - NAV_CROSSTRACK_SLOW_M) * 0.25f);
  }
  if (navMotionValid && fabsf(navLastMotionError) > 75.0f) {
    forwardScale *= 0.55f;
  }

  int16_t baseMag = (int16_t)roundf((float)maxForward * forwardScale);
  if (maxForward > 0 && baseMag < 6) baseMag = 6;
  if (baseMag > maxForward) baseMag = maxForward;
  const int16_t base = baseMag * (navInvertForward ? -1 : 1);

  float steer = (float)maxTurn * fminf(absError / 45.0f, 1.0f) *
                (errorDeg > 0.0f ? 1.0f : -1.0f);
  steer += fmaxf(-(float)maxTurn * 0.35f,
                 fminf((float)maxTurn * 0.35f, crossTrackM * 5.0f));
  if (navInvertSteering) steer = -steer;

  motorsSetTarget(clampI16(base + (int16_t)roundf(steer), -100, 100),
                  clampI16(base - (int16_t)roundf(steer), -100, 100));
}

static void navBroadcast(bool force = false) {
  const uint32_t now = millis();
  if (!force && now - lastNavBroadcastMs < NAV_BROADCAST_MS) return;
  lastNavBroadcastMs = now;
  char msg[160];
  snprintf(msg, sizeof(msg), "NAV,%s,%u,%u,%.2f,%.1f,%.1f,%.2f,%.1f,%s",
           navState, navWpIndex, navRouteCount, navLastDistanceM,
           navLastHeadingError, navLastBearing, navLastCrossTrackM,
           navMotionValid ? navLastMotionError : 0.0f, navReason);
  broadcastTelemetry(msg);
}

static void navUpdate() {
  const uint32_t now = millis();
  navBroadcast(false);
  if (!navRunning || navPaused) return;
  if (now - lastNavLoopMs < NAV_LOOP_MS) return;
  lastNavLoopMs = now;

  if (!navRouteReady || navRouteCount == 0 || navWpIndex >= navRouteCount) {
    navStop("nav-route-empty");
    navState = "ERROR";
    navBroadcast(true);
    return;
  }
  if (!gps.valid || gps.fixType < 3 || gps.lastMs == 0 ||
      now - gps.lastMs > NAV_MAX_GPS_AGE_MS) {
    motorsStop("nav gps stale");
    navState = "WAIT_GPS";
    navReason = "gps";
    navBroadcast(true);
    return;
  }
  const NavQuality quality = navQuality(now);
  if (quality == NAV_Q_NONE) {
    motorsStop("nav gps quality");
    navState = "WAIT_GPS";
    navReason = "gps-quality";
    navBroadcast(true);
    return;
  }

  if (!navStartPointValid) {
    navStartPoint.lat = gps.lat;
    navStartPoint.lon = gps.lon;
    navStartPointValid = true;
  }

  const NavWaypoint& target = navRoute[navWpIndex];
  navLastDistanceM = distanceMeters(gps.lat, gps.lon, target.lat, target.lon);
  navLastBearing = bearingDeg(gps.lat, gps.lon, target.lat, target.lon);
  const NavWaypoint legStart = navCurrentLegStart();
  const NavLegMetrics leg = navComputeLegMetrics(legStart, target);
  navLastCrossTrackM = leg.crossTrackM;
  navLastAlongTrackM = leg.alongM;
  float heading = 0.0f;
  const char* headingSource = "none";
  navCurrentHeading(now, leg.aimBearing, &heading, &headingSource);
  navLastHeadingError = headingErrorDeg(heading, leg.aimBearing);

  if (navLastDistanceM <= NAV_ARRIVED_M ||
      (leg.lengthM > 0.10f && leg.alongM >= leg.lengthM - 0.10f &&
       navLastDistanceM <= NAV_PASS_WP_M)) {
    char wpMsg[32];
    snprintf(wpMsg, sizeof(wpMsg), "NAV_WP,%u", navWpIndex);
    broadcastTelemetry(wpMsg);
    navWpIndex++;
    navResetGuidanceState();
    if (navWpIndex >= navRouteCount) {
      navRunning = false;
      navState = "DONE";
      navReason = "done";
      motorsStop("nav done");
      navBroadcast(true);
      return;
    }
    navBroadcast(true);
    return;
  }

  navUpdateMotion(now, leg.aimBearing);
  navUpdateProgress(now);
  navMaybeEnterRecovery(now);

  navState = "RUNNING";
  if (now >= navRecoveryUntilMs) {
    static char reasonBuf[32];
    snprintf(reasonBuf, sizeof(reasonBuf), "track-%s-%s",
             navQualityName(quality), headingSource);
    navReason = reasonBuf;
  }
  navSetMotor(now, navLastHeadingError, navLastDistanceM, navLastCrossTrackM,
              quality, headingSource);
  navBroadcast(false);
}

static void motorsSetTarget(int left, int right) {
  startMotorTask();
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

static bool imuEnableReports() {
  const bool gameOk =
      bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, IMU_REPORT_INTERVAL_US);
  const bool rotOk = gameOk ? false : bno08x.enableReport(
                                             SH2_ROTATION_VECTOR,
                                             IMU_REPORT_INTERVAL_US);
  imuValid = gameOk || rotOk;
  imuYawSource = gameOk ? "game" : (rotOk ? "rot" : "none");
  return imuValid;
}

static bool parseMoveCommand(const String& msg, int* left, int* right) {
  if (!msg.startsWith("M,")) return false;
  const int comma = msg.indexOf(',', 2);
  if (comma < 0) return false;
  *left = msg.substring(2, comma).toInt();
  *right = msg.substring(comma + 1).toInt();
  return true;
}

static bool handleNavConfig(const String& msg) {
  if (!msg.startsWith("NAV_CFG,")) return false;
  int forward = navForwardPercent;
  int turn = navTurnPercent;
  int invForward = navInvertForward ? 1 : 0;
  int invSteering = navInvertSteering ? 1 : 0;
  int invYaw = navInvertYaw ? 1 : 0;
  float offset = navHeadingOffsetDeg;
  if (sscanf(msg.c_str(), "NAV_CFG,%d,%d,%d,%d,%f,%d",
             &forward, &turn, &invForward, &invSteering, &offset, &invYaw) !=
      6) {
    return false;
  }
  navForwardPercent = clampI16(forward, 0, 45);
  navTurnPercent = clampI16(turn, 0, 40);
  navInvertForward = invForward != 0;
  navInvertSteering = invSteering != 0;
  navHeadingOffsetDeg = normalizeDeg(offset);
  navInvertYaw = invYaw != 0;
  return true;
}

static bool handleRouteBegin(const String& msg) {
  if (!msg.startsWith("ROUTE_BEGIN,")) return false;
  const int requested = msg.substring(12).toInt();
  if (requested <= 0 || requested > NAV_MAX_WAYPOINTS) return false;
  navStop("route upload");
  navRouteExpected = (uint8_t)requested;
  navRouteCount = 0;
  navRouteReceivedCount = 0;
  navWpIndex = 0;
  navStartPointValid = false;
  navResetGuidanceState();
  for (uint8_t i = 0; i < NAV_MAX_WAYPOINTS; i++) {
    navRouteReceived[i] = false;
  }
  navRouteUploading = true;
  navRouteReady = false;
  navState = "ROUTE";
  return true;
}

static bool handleRouteWaypoint(const String& msg) {
  if (!msg.startsWith("ROUTE_WP,")) return false;
  if (!navRouteUploading) return false;
  const int c1 = msg.indexOf(',', 9);
  if (c1 < 0) return false;
  const int c2 = msg.indexOf(',', c1 + 1);
  if (c2 < 0) return false;
  const int index = msg.substring(9, c1).toInt();
  if (index < 0 || index >= navRouteExpected ||
      index >= NAV_MAX_WAYPOINTS) {
    return false;
  }
  navRoute[index].lat = msg.substring(c1 + 1, c2).toDouble();
  navRoute[index].lon = msg.substring(c2 + 1).toDouble();
  if (!navRouteReceived[index]) {
    navRouteReceived[index] = true;
    navRouteReceivedCount++;
  }
  if (index + 1 > navRouteCount) navRouteCount = (uint8_t)(index + 1);
  return true;
}

static bool handleRouteEnd() {
  if (!navRouteUploading) return false;
  navRouteUploading = false;
  navRouteReady = navRouteCount == navRouteExpected &&
                  navRouteReceivedCount == navRouteExpected &&
                  navRouteCount > 0;
  navState = navRouteReady ? "READY" : "ERROR";
  navReason = navRouteReady ? "ready" : "route";
  navBroadcast(true);
  return navRouteReady;
}

static void imuInit() {
  Wire.end();
  delay(20);
  Wire.begin(PIN_IMU_SDA, PIN_IMU_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(20);
  Serial.printf("IMU: I2C SDA=%d SCL=%d\n", PIN_IMU_SDA, PIN_IMU_SCL);

  uint8_t found = 0;
  bool found4A = false;
  bool found4B = false;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("IMU: I2C device found at 0x%02X\n", addr);
      if (addr == 0x4A) found4A = true;
      if (addr == 0x4B) found4B = true;
      found++;
    }
  }
  if (found == 0) {
    Serial.println("IMU: I2C scan found no devices");
  }

  bool bnoOk = false;
  if (found4B) bnoOk = bno08x.begin_I2C(0x4B, &Wire);
  if (!bnoOk && found4A) bnoOk = bno08x.begin_I2C(0x4A, &Wire);
  if (!bnoOk && !found4A && !found4B) {
    bnoOk = bno08x.begin_I2C(0x4B, &Wire) || bno08x.begin_I2C(0x4A, &Wire);
  }
  if (!bnoOk) {
    imuDetected = false;
    imuValid = false;
    Serial.println("IMU: BNO085 not found at 0x4A/0x4B");
    return;
  }
  imuDetected = true;
  if (!imuEnableReports()) {
    Serial.println("IMU: BNO085 report enable failed");
    return;
  }
  lastImuMs = millis();
  Serial.println("IMU: BNO085 found, rotation vectors enabled");
}

static void imuReset(const char* reason) {
  Serial.printf("IMU: reset requested reason=%s\n", reason);
  imuDetected = false;
  imuValid = false;
  lastImuMs = 0;
  lastImuRotationMs = 0;
  lastImuGameMs = 0;
  lastImuRecoverMs = 0;
  lastImuHardResetMs = millis();
  imuYawSource = "none";
  Wire.end();
  delay(80);
  imuInit();
}

static void imuStoreYaw(uint8_t sensorId, float yaw, uint32_t now) {
  if (sensorId == SH2_ROTATION_VECTOR) {
    imuRotationYaw = yaw;
    lastImuRotationMs = now;
  } else {
    imuGameYaw = yaw;
    lastImuGameMs = now;
  }

  if (lastImuRotationMs != 0 && now - lastImuRotationMs <= 250) {
    imuYaw = imuRotationYaw;
    lastImuMs = lastImuRotationMs;
    imuYawSource = "rot";
  } else if (lastImuGameMs != 0) {
    imuYaw = imuGameYaw;
    lastImuMs = lastImuGameMs;
    imuYawSource = "game";
  }
}

static void imuUpdate() {
  const uint32_t now = millis();
  if (!imuDetected) {
    if (now - lastImuRecoverMs >= IMU_NOT_FOUND_RETRY_MS) {
      lastImuRecoverMs = now;
      Serial.println("WARN IMU not detected, retrying BNO085 init");
      imuInit();
    }
    return;
  }
  if (bno08x.wasReset()) {
    Serial.println("IMU: BNO085 reset, re-enabling rotation vectors");
    imuValid = imuEnableReports();
    if (!imuValid) return;
  }

  while (bno08x.getSensorEvent(&imuValue)) {
    if (imuValue.sensorId != SH2_GAME_ROTATION_VECTOR &&
        imuValue.sensorId != SH2_ROTATION_VECTOR) {
      continue;
    }
    const float qw = imuValue.sensorId == SH2_GAME_ROTATION_VECTOR
                         ? imuValue.un.gameRotationVector.real
                         : imuValue.un.rotationVector.real;
    const float qx = imuValue.sensorId == SH2_GAME_ROTATION_VECTOR
                         ? imuValue.un.gameRotationVector.i
                         : imuValue.un.rotationVector.i;
    const float qy = imuValue.sensorId == SH2_GAME_ROTATION_VECTOR
                         ? imuValue.un.gameRotationVector.j
                         : imuValue.un.rotationVector.j;
    const float qz = imuValue.sensorId == SH2_GAME_ROTATION_VECTOR
                         ? imuValue.un.gameRotationVector.k
                         : imuValue.un.rotationVector.k;
    float yaw = atan2(2.0f * (qw * qz + qx * qy),
                      1.0f - 2.0f * (qy * qy + qz * qz)) *
                180.0f / PI;
    if (yaw < 0) yaw += 360.0f;
    imuValid = true;
    imuStoreYaw(imuValue.sensorId, yaw, millis());
  }

  if (lastImuMs != 0 && now >= lastImuMs &&
      now - lastImuMs > IMU_STALE_RESET_MS &&
      now - lastImuHardResetMs > IMU_STALE_RESET_MS) {
    lastImuHardResetMs = now;
    Serial.printf("WARN IMU hard reset age=%lums\n",
                  (unsigned long)(now - lastImuMs));
    imuReset("stale");
    return;
  }
  if (lastImuMs != 0 && now >= lastImuMs &&
      now - lastImuMs > IMU_STALE_REENABLE_MS &&
      now - lastImuRecoverMs > IMU_STALE_REENABLE_MS) {
    lastImuRecoverMs = now;
    Serial.printf("WARN IMU stale age=%lums, re-enabling rotation vectors\n",
                  (unsigned long)(now - lastImuMs));
    imuValid = imuEnableReports();
  }
}

static void broadcastImu() {
  const uint32_t now = millis();
  if (now - lastImuBroadcastMs < IMU_BROADCAST_MS) return;
  lastImuBroadcastMs = now;
  if (!imuFresh(now)) return;
  const uint32_t imuAgeMs = now - lastImuMs;
  char msg[64];
  snprintf(msg, sizeof(msg), "IMU,%.2f,%lu,1,%s", imuYaw,
           (unsigned long)imuAgeMs, imuYawSource);
  broadcastTelemetry(msg);
}

static void setupWeb() {
  if (ROUTER_MODE) {
    wifiStartAllowedMs = millis() + WIFI_STARTUP_DELAY_MS;
    Serial.printf("Router mode: deferred WiFi %lums, WebSocket ws://%s:%u/ws\n",
                  (unsigned long)WIFI_STARTUP_DELAY_MS,
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
      if (!navRunning) {
        motorsStop("websocket disconnect");
      } else {
        Serial.println("NAV: websocket disconnected, local navigation continues");
      }
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
      navStop("ws STOP");
      client->text("OK STOP");
    } else if (msg == "UDP_RESET") {
      restartRtcmUdp("ws UDP_RESET");
      client->text("OK UDP_RESET");
    } else if (msg == "IMU_RESET") {
      client->text("OK IMU_RESET");
      imuReset("ws IMU_RESET");
    } else if (msg == "IMU_STATUS") {
      char status[128];
      const uint32_t now = millis();
      const uint32_t imuAgeMs = lastImuMs == 0 ? 0 : now - lastImuMs;
      snprintf(status, sizeof(status),
               "IMU_STATUS,detected=%d,valid=%d,fresh=%d,age=%lu,yaw=%.2f,src=%s",
               imuDetected ? 1 : 0,
               imuValid ? 1 : 0,
               imuFresh(now) ? 1 : 0,
               (unsigned long)imuAgeMs,
               imuYaw,
               imuYawSource);
      client->text(status);
    } else if (msg == "STATUS") {
      char status[192];
      const uint32_t now = millis();
      const uint32_t rtcmAgeMs = effectiveRtcmAgeMs(now);
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
    } else if (handleNavConfig(msg)) {
      client->text("OK NAV_CFG");
    } else if (handleRouteBegin(msg)) {
      client->text("OK ROUTE_BEGIN");
    } else if (handleRouteWaypoint(msg)) {
      client->text("OK ROUTE_WP");
    } else if (msg == "ROUTE_END") {
      client->text(handleRouteEnd() ? "OK ROUTE_END" : "ERR,ROUTE_END");
    } else if (msg == "NAV_START") {
      if (navRouteReady && navRouteCount > 0) {
        navRunning = true;
        navPaused = false;
        navWpIndex = 0;
        navStartPoint.lat = gps.valid ? gps.lat : navRoute[0].lat;
        navStartPoint.lon = gps.valid ? gps.lon : navRoute[0].lon;
        navStartPointValid = gps.valid;
        navResetGuidanceState();
        navState = "RUNNING";
        navReason = "start";
        navBroadcast(true);
        client->text("OK NAV_START");
      } else {
        client->text("ERR,NAV_NO_ROUTE");
      }
    } else if (msg == "NAV_PAUSE") {
      if (navRunning) {
        navPaused = true;
        navState = "PAUSED";
        motorsStop("nav pause");
        navBroadcast(true);
      }
      client->text("OK NAV_PAUSE");
    } else if (msg == "NAV_RESUME") {
      if (navRouteReady && navRouteCount > 0) {
        navRunning = true;
        navPaused = false;
        if (!navStartPointValid && gps.valid) {
          navStartPoint.lat = gps.lat;
          navStartPoint.lon = gps.lon;
          navStartPointValid = true;
        }
        navResetGuidanceState();
        navState = "RUNNING";
        navBroadcast(true);
      }
      client->text("OK NAV_RESUME");
    } else if (msg == "NAV_STOP") {
      navStop("nav stop");
      navBroadcast(true);
      client->text("OK NAV_STOP");
    } else {
      int left = 0;
      int right = 0;
      if (parseMoveCommand(msg, &left, &right)) {
        navRunning = false;
        navPaused = false;
        navState = "IDLE";
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
}

static void setupOta() {
  if (otaCallbacksConfigured) return;
  ArduinoOTA.setHostname("rtk-rover");
  ArduinoOTA.onStart([]() {
    motorsStop("OTA start");
    Serial.println("OTA: start");
  });
  ArduinoOTA.onEnd([]() { Serial.println("OTA: end"); });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA: error %u\n", (unsigned)error);
  });
  otaCallbacksConfigured = true;
  Serial.println("OTA: callbacks configured");
}

static void startNetworkServices() {
  if (networkServicesStarted) return;
  server.begin();
  Serial.println("WEB: HTTP/WebSocket server started");
  ArduinoOTA.begin();
  Serial.println("OTA: enabled hostname=rtk-rover");
  networkServicesStarted = true;
}

static void printStatus() {
  const uint32_t now = millis();
  if (now - lastStatusMs < STATUS_MS) return;
  lastStatusMs = now;

  const uint32_t gpsAgeMs = gps.lastMs == 0 ? 0 : now - gps.lastMs;
  const uint32_t rtcmAgeMs = effectiveRtcmAgeMs(now);
  const uint32_t f9pRtcmAgeMs =
      f9pLastRtcmMs == 0 ? 0 : now - f9pLastRtcmMs;
  const uint32_t imuAgeMs = lastImuMs == 0 ? 0 : now - lastImuMs;
  Serial.printf("ROVER wifi=%s ip=%s wifiReconnect=%lu udpRestart=%lu tcpReconnect=%lu tcpBytes=%lu tcpReads=%lu port=%s baud=%lu src=%s parsed=%lu clients=%u fix=%u carrier=%s diff=%u sv=%u hAcc=%lumm gpsAge=%lums raw=%lu rtcmSrc=%s rtcm=%lubytes/%lupkts age=%lums from=%s size=%u f9pRtcm=%lu crcFail=%lu lastType=%u age=%lums imu=%s yaw=%.2f src=%s age=%lums nav=%s wp=%u/%u dist=%.2f err=%.1f xt=%.2f moveErr=%.1f reason=%s motorTarget=%d/%d motorCur=%d/%d motorCmd=%d/%d motorCount=%lu stopCount=%lu\n",
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
                     : (imuFresh(now) ? "ok" : "stale")),
                imuYaw,
                imuYawSource,
                (unsigned long)imuAgeMs,
                navState,
                navWpIndex,
                navRouteCount,
                navLastDistanceM,
                navLastHeadingError,
                navLastCrossTrackM,
                navMotionValid ? navLastMotionError : 0.0f,
                navReason,
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
  setCpuFrequencyMhz(80);
  delay(300);
  Serial.println();
  Serial.println("ESP32 ZED-F9P RTK ROVER");

  motorsInit();
  autoDetectGpsBaud();
  setupWeb();
  setupOta();
  if (!ROUTER_MODE) {
    startNetworkServices();
  }
  restartRtcmUdp("setup");
  delay(300);
  imuInit();
  startMotorTask();
}

void loop() {
  if (networkServicesStarted) {
    ArduinoOTA.handle();
  }
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

  navUpdate();
  broadcastGps();
  broadcastImu();
  checkRtcmWatchdog();
  checkF9pRtcmWatchdog();
  printStatus();
  ws.cleanupClients();
}
