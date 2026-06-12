/**
 * RTK Rover Autopilot v2.0 - Advanced Navigation System
 *
 * Key Features:
 * - Movement Monitor (progress tracking, stuck detection, heading validation)
 * - Advanced IMU Calibration (multi-sample, GPS verification, drift monitoring)
 * - GPS Heading Fusion (IMU + GPS velocity based heading)
 * - Improved Arrival Detection (distance + time + GPS verification)
 * - State Machine with Recovery (IDLE, CALIBRATING, MOVING, APPROACHING, RECOVERING, ERROR)
 *
 * Target Accuracy: < 5cm with RTK Fixed
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Preferences.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <esp_system.h>
#include <Adafruit_BNO08x.h>
#include "NavigationCore.h"

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
static constexpr uint32_t WIFI_RECONNECT_INITIAL_MS = 15000;
static constexpr uint32_t WIFI_RECONNECT_MAX_MS = 60000;

// GPS
static constexpr int GPS_RX = 4;
static constexpr int GPS_TX = 5;
static constexpr int GPS_ALT_RX = GPS_TX;
static constexpr int GPS_ALT_TX = GPS_RX;
static constexpr uint32_t GPS_BAUD = 38400;
static int g_activeGpsRx = GPS_RX;
static int g_activeGpsTx = GPS_TX;

// Motor
static constexpr int MOTOR_RX = 16;
static constexpr int MOTOR_TX = 17;
static constexpr uint32_t MOTOR_BAUD = 115200;

// Relays from the previous manual-control firmware.
static constexpr int PIN_RELAY_ATTACHMENT = 32;
static constexpr int PIN_RELAY_MOUNT = 33;

// IMU
static constexpr int IMU_SDA = 21;
static constexpr int IMU_SCL = 22;

// ============== QUALITY THRESHOLDS ==============

static constexpr uint32_t RTK_FIXED_AGE_MS = 500;
static constexpr uint32_t RTK_FLOAT_AGE_MS = 1000;
static constexpr uint32_t GPS_HOLD_AGE_MS = 2000;
static constexpr uint32_t GPS_DEAD_AGE_MS = 5000;
static constexpr uint32_t IMU_TIMEOUT_MS = 2000;
static constexpr uint32_t IMU_REPORT_INTERVAL_US = 20000;
static constexpr uint32_t IMU_REENABLE_MS = 2000;
static constexpr uint32_t RTK_CORRECTION_TIMEOUT_MS = 30000;

static constexpr uint32_t RTK_FIXED_HACC_MM = 50;
static constexpr uint32_t RTK_FLOAT_HACC_MM = 300;
static constexpr uint32_t DEGRADED_HACC_MM = 1500;

// ============== GPS POSITION FILTERING ==============

static constexpr uint8_t GPS_MEDIAN_WINDOW = 5;        // Median filter size
static constexpr float GPS_OUTLIER_THRESHOLD_M = 0.5f;  // Max jump from median to accept
static constexpr float GPS_HACC_WEIGHT_THRESHOLD_MM = 50.0f;  // hAcc threshold for weighted average
static constexpr float GPS_MAX_HACC_WEIGHT_MM = 300.0f; // hAcc for minimum weight

// ============== SPEED SETTINGS ==============

// The robot still cannot lock onto a heading target at 0.06 m/s. The
// problem is wheel inertia: at the previous speeds the EMA-filtered
// heading error still had enough authority to throw the chassis into
// an arc that misses the waypoint before the next 50 ms tick. We
// calibrated against manual drive: in MANUAL, M,15,15 gives the
// wheels about 5.8% of HOVER_MAX_CMD, and at that throttle the
// operator can comfortably steer without overshoot. We set
// MAX_SPEED so the autonomous controller asks for ~2.9% of
// HOVER_MAX_CMD at top speed — about 2x slower than the comfortable
// manual 15%. That gives the EMA heading filter enough time to
// converge before the wheels have built up any meaningful angular
// momentum, so the robot can actually reach its heading target.
//
// At 0.0035 m/s a 10 m segment is ~48 minutes. We trade time for
// stability here on purpose.
static constexpr float MAX_SPEED = 0.0035f;
static constexpr float FLOAT_SPEED = 0.0018f;
static constexpr float DEGRADED_SPEED = 0.0012f;
static constexpr float HOLD_SPEED = 0.0006f;

// ============== TIMING ==============

static constexpr uint32_t NAV_LOOP_MS = 50;
static constexpr uint32_t MOTOR_SEND_MS = 20;
static constexpr uint32_t STATUS_MS = 2000;
static constexpr uint32_t TELEMETRY_MS = 200;
static constexpr uint32_t MANUAL_CMD_TIMEOUT_MS = 400;

// ============== MOTOR CONTROL ==============

static constexpr int MAX_SPEED_PERCENT = 60;
static constexpr int16_t HOVER_MAX_CMD = 300;
static constexpr int INPUT_DIV = 2;
static constexpr uint32_t HOVER_SEND_MS = 20;
static constexpr uint32_t CMD_TIMEOUT_MS = 400;
static constexpr uint32_t RAMP_UPDATE_MS = 20;
// CRITICAL FIX: was 1, caused 1.4s delay from 0 to 70%. Now 5 = 280ms total ramp.
static constexpr int RAMP_STEP_UP_PER_TICK = 5;
static constexpr int RAMP_STEP_DOWN_PER_TICK = 8;  // Stop faster than accelerate
static constexpr int16_t SLEW_SPEED_PER_SEND = 4;
static constexpr int16_t SLEW_STEER_PER_SEND = 6;

// ============== NAVIGATION CONSTANTS ==============

// Target accuracy 2-5cm
static constexpr float ARRIVAL_DIST_M = 0.05f;           // 5cm - target accuracy
// CRITICAL FIX: was 2.0s, caused overshoot. Now 0.3s for responsive stopping.
static constexpr float ARRIVAL_CONFIRM_TIME_S = 0.3f;   // Must be within 5cm for 0.3 seconds
static constexpr float ARRIVAL_APPROACH_DIST_M = 0.3f;  // Start approach mode at this distance
static constexpr float INTERMEDIATE_ADVANCE_MAX_M = 0.60f;  // Segment handoff distance for long mowing rows

// Movement Monitor
static constexpr float PROGRESS_RATE_MIN_MPS = -0.05f;  // Must be approaching faster than this
// Stuck threshold must be BELOW HOLD_SPEED (0.0006 m/s) so HOLD-mode does
// not falsely trip the recovery loop and stop the robot at slow speed.
static constexpr float STUCK_SPEED_THRESHOLD_MPS = 0.0003f; // Below this = stuck
static constexpr float HEADING_ERROR_THRESHOLD_DEG = 15.0f; // Heading error limit
static constexpr float SPEED_ERROR_THRESHOLD_MPS = 0.1f;  // Commanded vs actual speed
static constexpr uint32_t MONITOR_HISTORY_SIZE = 50;     // 5 seconds at 10Hz
static constexpr uint32_t RECOVERY_TIMEOUT_MS = 5000;    // Time before recovery attempt

// IMU Calibration
static constexpr uint32_t IMU_CAL_SAMPLE_COUNT = 100;   // Number of samples for calibration
static constexpr uint32_t IMU_CAL_SAMPLE_INTERVAL_MS = 50;
static constexpr float IMU_CAL_STD_DEV_MAX_DEG = 3.0f;   // Max std dev for valid calibration
static constexpr float IMU_DRIFT_THRESHOLD_DEG = 5.0f;   // Trigger recalibration
static constexpr uint32_t IMU_DRIFT_CHECK_INTERVAL_MS = 30000;

// GPS Heading
static constexpr float GPS_HEADING_MIN_SPEED_MPS = 0.05f; // Min speed for GPS heading
static constexpr float GPS_HEADING_BLEND_FACTOR = 0.3f;  // Weight for GPS heading
// Low-pass filter on GPS-velocity-derived heading. Without this, raw GPS
// velocity noise (a few mm/s on a 0.18 m/s move) makes heading jump by 2-3°
// per NAV_LOOP tick and the controller oscillates. 0.20 keeps a ~150 ms
// time constant (NAV_LOOP=50 ms) — smooth enough to stop the wobble, fast
// enough to react to real turns.
static constexpr float HEADING_EMA_ALPHA = 0.20f;
// Dead zone: ignore heading errors below this. RTK position noise (1.4 cm)
// at 0.18 m/s produces ~4.5° of bearing jitter, so 3° cuts pure noise turns
// while still correcting real misalignment.
static constexpr float HEADING_DEAD_ZONE_DEG = 3.0f;

// Cross track
static constexpr float CROSS_TRACK_LIMIT_M = 1.0f;      // Max allowed cross track error
static constexpr float CROSS_TRACK_SLOW_M = 0.18f;
static constexpr float CROSS_TRACK_STOP_M = 0.45f;
static constexpr uint32_t CROSS_TRACK_STOP_CONFIRM_MS = 1200;
static constexpr float SEGMENT_ENTRY_SLOW_M = 0.60f;
static constexpr uint32_t SEGMENT_ENTRY_SLOW_MS = 2500;
static constexpr float ROBOT_RADIUS_M = 0.20f;  // Physical robot radius ~15cm, clearance for path planning
static constexpr float FORBIDDEN_STOP_CLEARANCE_M = 0.08f;  // RTK drit 14mm + handoff 0.6m margin: stop only inside or below 8cm
static constexpr float FORBIDDEN_SLOW_CLEARANCE_M = 0.10f;  // Slow only at the requested 10cm near-contour band

// ============== MOTOR STATE ==============

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

static float approxGpsDistanceM(double latA, double lonA, double latB, double lonB) {
  const double latRad = ((latA + latB) * 0.5) * PI / 180.0;
  const double mPerDegLat = 111132.92 - 559.82 * cos(2 * latRad) + 1.175 * cos(4 * latRad);
  const double mPerDegLon = 111412.84 * cos(latRad) - 93.5 * cos(3 * latRad);
  const double dx = (lonB - lonA) * mPerDegLon;
  const double dy = (latB - latA) * mPerDegLat;
  return (float)sqrt(dx * dx + dy * dy);
}

// ============== NAVIGATION QUALITY ==============

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
  STATE_CALIBRATING,
  STATE_MOVING,
  STATE_APPROACHING,
  STATE_ARRIVED,
  STATE_RECOVERING,
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
    case STATE_CALIBRATING: return "CALIBRATING";
    case STATE_MOVING: return "MOVING";
    case STATE_APPROACHING: return "APPROACHING";
    case STATE_ARRIVED: return "ARRIVED";
    case STATE_RECOVERING: return "RECOVERING";
    case STATE_ERROR: return "ERROR";
  }
  return "UNKNOWN";
}

// ============== GPS INPUT ==============

static HardwareSerial GpsSerial(1);
static HardwareSerial MotorSerial(2);

struct GpsRaw {
  double lat = 0, lon = 0;
  float hAcc = 999999;
  float vAcc = 999999;
  float speed = 0;
  float heading = 0;
  float velN = 0;  // North velocity m/s
  float velE = 0;  // East velocity m/s
  uint8_t fixType = 0;
  uint8_t carrier = 0;
  bool diff = false;
  bool valid = false;
  int8_t numSV = 0;
  uint32_t lastMs = 0;
};

static GpsRaw g_gps;
static uint32_t g_gpsRawBytes = 0;

static bool gpsPositionPlausible(double lat, double lon, float hAcc, uint8_t numSV) {
  if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) return false;
  if (numSV > 80) return false;
  if (hAcc <= 0.0f || hAcc > 20000.0f) return false;
  if (!g_gps.valid || g_gps.lastMs == 0) return true;

  const uint32_t ageMs = millis() - g_gps.lastMs;
  if (ageMs > 10000) return true;
  const float jumpM = approxGpsDistanceM(g_gps.lat, g_gps.lon, lat, lon);
  const float allowedJumpM = 3.0f + 0.003f * (float)ageMs +
      max(g_gps.speed, 0.1f) * ((float)ageMs * 0.001f) * 4.0f;
  return jumpM <= max(allowedJumpM, 8.0f);
}

// GPS Position Filter
struct GpsPositionFilter {
  LocalCoords samples[GPS_MEDIAN_WINDOW];
  float hAccHistory[GPS_MEDIAN_WINDOW];
  uint8_t head = 0;
  uint8_t count = 0;
  bool filterInitialized = false;
  LocalCoords filteredPos;
  float filteredHAcc = 999999.0f;
};

static GpsPositionFilter g_gpsFilter;
static uint32_t g_gpsUbxParsed = 0;
static uint32_t g_gpsNmeaParsed = 0;
static uint32_t g_gpsPvtWatchdog = 0;
static uint32_t g_f9pRtcmMsgs = 0;
static uint32_t g_f9pRtcmCrcFail = 0;
static uint32_t g_lastF9pRtcmMs = 0;
static uint32_t g_rtcmLastType = 0;
static uint32_t g_ubxAck = 0;
static uint32_t g_ubxNak = 0;

// GPS parser state
enum GpsParserState { GPS_SYNC1, GPS_SYNC2, GPS_CLASS, GPS_ID, GPS_LEN1, GPS_LEN2, GPS_PAYLOAD, GPS_CKA, GPS_CKB };
static GpsParserState gpsState = GPS_SYNC1;
static uint8_t gpsClass = 0, gpsId = 0;
static uint16_t gpsLen = 0, gpsCount = 0;
static uint8_t gpsPayload[128];
static uint8_t gpsCkA = 0, gpsCkB = 0;
static char nmeaLine[128];
static uint8_t nmeaLen = 0;

static void feedGpsByte(uint8_t b);

// UBX config keys
static constexpr uint32_t CFG_RATE_MEAS = 0x30210001;
static constexpr uint32_t CFG_MSGOUT_UBX_NAV_PVT_UART1 = 0x20910007;
static constexpr uint32_t CFG_MSGOUT_UBX_NAV_RELPOSNED_UART1 = 0x2099008E;
static constexpr uint32_t CFG_MSGOUT_UBX_RXM_RTCM_UART1 = 0x20910269;
static constexpr uint32_t CFG_MSGOUT_UBX_RXM_RTCM_UART2 = 0x2091026A;
static constexpr uint32_t CFG_MSGOUT_UBX_NAV_VELNED_UART1 = 0x20910027;
static constexpr uint32_t CFG_MSGOUT_NMEA_GGA_UART1 = 0x209100BA;
static constexpr uint32_t CFG_UART1INPROT_UBX = 0x10730001;
static constexpr uint32_t CFG_UART1INPROT_NMEA = 0x10730002;
static constexpr uint32_t CFG_UART1INPROT_RTCM3X = 0x10730004;
static constexpr uint32_t CFG_UART1OUTPROT_UBX = 0x10740001;
static constexpr uint32_t CFG_UART1OUTPROT_NMEA = 0x10740002;
static constexpr uint32_t CFG_UART2INPROT_UBX = 0x10750001;
static constexpr uint32_t CFG_UART2INPROT_NMEA = 0x10750002;
static constexpr uint32_t CFG_UART2INPROT_RTCM3X = 0x10750004;
static constexpr uint32_t CFG_UART2OUTPROT_UBX = 0x10760001;
static constexpr uint32_t CFG_UART2OUTPROT_NMEA = 0x10760002;

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

static void serviceGpsInputFor(uint32_t durationMs) {
  const uint32_t until = millis() + durationMs;
  do {
    while (GpsSerial.available()) {
      feedGpsByte(GpsSerial.read());
    }
    delay(1);
  } while ((int32_t)(millis() - until) < 0);
}

static void sendValset(uint32_t key, uint32_t value, uint8_t size) {
  uint8_t payload[16] = {0};
  payload[0] = 0; payload[1] = 0x01;
  putU32(payload + 4, key);
  for (uint8_t i = 0; i < size; i++) {
    payload[8 + i] = (uint8_t)(value >> (8 * i));
  }
  writeUbx(0x06, 0x8A, payload, 8 + size);
  serviceGpsInputFor(30);
  Serial.printf("GPS CFG key=0x%08lX val=%lu\n", (unsigned long)key, (unsigned long)value);
}

// ============== GPS POSITION FILTERING ==============

static void resetGpsFilter() {
  g_gpsFilter.head = 0;
  g_gpsFilter.count = 0;
  g_gpsFilter.filterInitialized = false;
  g_gpsFilter.filteredPos = {0, 0};
  g_gpsFilter.filteredHAcc = 999999.0f;
}

static LocalCoords sortAndGetMedian(LocalCoords* arr, uint8_t n) {
  // Simple selection sort to get median (works for small arrays)
  for (uint8_t i = 0; i < n - 1; i++) {
    for (uint8_t j = i + 1; j < n; j++) {
      float di = hypotf(arr[i].x, arr[i].y);
      float dj = hypotf(arr[j].x, arr[j].y);
      if (dj < di) {
        LocalCoords tmp = arr[i];
        arr[i] = arr[j];
        arr[j] = tmp;
      }
    }
  }
  return arr[n / 2];
}

static void updateGpsFilter(LocalCoords rawPos, float hAcc) {
  // Add new sample
  g_gpsFilter.samples[g_gpsFilter.head] = rawPos;
  g_gpsFilter.hAccHistory[g_gpsFilter.head] = hAcc;
  g_gpsFilter.head = (g_gpsFilter.head + 1) % GPS_MEDIAN_WINDOW;
  if (g_gpsFilter.count < GPS_MEDIAN_WINDOW) {
    g_gpsFilter.count++;
  }

  // Need at least 3 samples to filter
  if (g_gpsFilter.count < 3) {
    g_gpsFilter.filteredPos = rawPos;
    g_gpsFilter.filteredHAcc = hAcc;
    g_gpsFilter.filterInitialized = false;
    return;
  }

  // Calculate median position
  LocalCoords sortedSamples[GPS_MEDIAN_WINDOW];
  for (uint8_t i = 0; i < g_gpsFilter.count; i++) {
    sortedSamples[i] = g_gpsFilter.samples[i];
  }
  LocalCoords medianPos = sortAndGetMedian(sortedSamples, g_gpsFilter.count);

  // Check if current position is outlier (too far from median)
  float distFromMedian = hypotf(rawPos.x - medianPos.x, rawPos.y - medianPos.y);

  if (distFromMedian > GPS_OUTLIER_THRESHOLD_M && g_gpsFilter.filterInitialized) {
    // Outlier detected - use median instead
    g_gpsFilter.filteredPos = medianPos;
    g_gpsFilter.filteredHAcc = hAcc * 1.5f;  // Increase uncertainty
  } else {
    // Normal case - weighted average with median as base
    g_gpsFilter.filteredPos = rawPos;
    g_gpsFilter.filteredHAcc = hAcc;
  }

  g_gpsFilter.filterInitialized = true;
}

static void applyWeightedGpsUpdate(LocalCoords* pos, float hAcc) {
  // If hAcc is high, blend with previous filtered position
  if (hAcc > GPS_HACC_WEIGHT_THRESHOLD_MM && g_gpsFilter.filterInitialized) {
    // Weight inversely proportional to hAcc
    float weight = constrain(1.0f - (hAcc - GPS_HACC_WEIGHT_THRESHOLD_MM) /
                             (GPS_MAX_HACC_WEIGHT_MM - GPS_HACC_WEIGHT_THRESHOLD_MM), 0.0f, 1.0f);

    // Blend: filtered = weight * raw + (1-weight) * median
    pos->x = weight * pos->x + (1.0f - weight) * g_gpsFilter.filteredPos.x;
    pos->y = weight * pos->y + (1.0f - weight) * g_gpsFilter.filteredPos.y;
  }
}

static void sendCfgMsg(uint8_t msgClass, uint8_t msgId, uint8_t uartRate) {
  uint8_t payload[8] = {0};
  payload[0] = msgClass;
  payload[1] = msgId;
  payload[3] = uartRate;  // UART1
  payload[4] = uartRate;  // UART2
  writeUbx(0x06, 0x01, payload, sizeof(payload));
  serviceGpsInputFor(30);
}

static void resetGpsIoMessageConfig() {
  uint8_t payload[12] = {0};
  payload[0] = 0x03;  // clearMask: ioPort + msgConf
  payload[1] = 0x00;
  payload[2] = 0x00;
  payload[3] = 0x00;
  writeUbx(0x06, 0x09, payload, sizeof(payload));
  serviceGpsInputFor(150);
  while (GpsSerial.available()) feedGpsByte(GpsSerial.read());
  Serial.println("GPS: cleared I/O and message config");
}

static void sendCfgPrt(uint8_t portId, uint16_t inProtoMask, uint16_t outProtoMask) {
  uint8_t cfgPrt[28] = {0};
  cfgPrt[0] = 0xB5; cfgPrt[1] = 0x62;
  cfgPrt[2] = 0x06; cfgPrt[3] = 0x00;
  cfgPrt[4] = 0x14; cfgPrt[5] = 0x00;
  cfgPrt[6] = portId;
  cfgPrt[8] = 0x00; cfgPrt[9] = 0x00;  // txReady
  cfgPrt[10] = 0xC0; cfgPrt[11] = 0x08; cfgPrt[12] = 0x00; cfgPrt[13] = 0x00;  // mode: 8N1
  cfgPrt[14] = (uint8_t)GPS_BAUD;
  cfgPrt[15] = (uint8_t)(GPS_BAUD >> 8);
  cfgPrt[16] = (uint8_t)(GPS_BAUD >> 16);
  cfgPrt[17] = (uint8_t)(GPS_BAUD >> 24);
  cfgPrt[18] = (uint8_t)inProtoMask;
  cfgPrt[19] = (uint8_t)(inProtoMask >> 8);
  cfgPrt[20] = (uint8_t)outProtoMask;
  cfgPrt[21] = (uint8_t)(outProtoMask >> 8);
  cfgPrt[22] = 0x00; cfgPrt[23] = 0x00;  // flags
  cfgPrt[24] = 0x00; cfgPrt[25] = 0x00;  // reserved
  uint8_t ckA = 0, ckB = 0;
  for (int i = 2; i <= 25; i++) { ckA += cfgPrt[i]; ckB += ckA; }
  cfgPrt[26] = ckA; cfgPrt[27] = ckB;
  GpsSerial.write(cfgPrt, sizeof(cfgPrt));
  serviceGpsInputFor(50);
  Serial.printf("GPS CFG-PRT port=%u in=0x%04X out=0x%04X\n",
    portId, inProtoMask, outProtoMask);
}

static void configureGpsRover() {
  Serial.println("GPS: configuring rover...");

  resetGpsIoMessageConfig();
  delay(100);

  sendCfgPrt(0x01, 0x0021, 0x0001);  // UART1: UBX + RTCM3 in, UBX out
  delay(100);
  sendCfgPrt(0x02, 0x0021, 0x0001);  // UART2: same, covers boards wired to UART2
  delay(200);

  sendValset(CFG_UART1INPROT_UBX, 1, 1);
  sendValset(CFG_UART1INPROT_NMEA, 0, 1);
  sendValset(CFG_UART1INPROT_RTCM3X, 1, 1);
  sendValset(CFG_UART1OUTPROT_UBX, 1, 1);
  sendValset(CFG_UART1OUTPROT_NMEA, 0, 1);
  sendValset(CFG_UART2INPROT_UBX, 1, 1);
  sendValset(CFG_UART2INPROT_NMEA, 0, 1);
  sendValset(CFG_UART2INPROT_RTCM3X, 1, 1);
  sendValset(CFG_UART2OUTPROT_UBX, 1, 1);
  sendValset(CFG_UART2OUTPROT_NMEA, 0, 1);

  sendValset(CFG_RATE_MEAS, 100, 2);  // 10 Hz
  sendValset(CFG_MSGOUT_UBX_NAV_PVT_UART1, 1, 1);
  sendValset(CFG_MSGOUT_UBX_RXM_RTCM_UART1, 1, 1);
  sendValset(CFG_MSGOUT_UBX_RXM_RTCM_UART2, 1, 1);
  sendValset(CFG_MSGOUT_UBX_NAV_RELPOSNED_UART1, 1, 1);
  sendValset(CFG_MSGOUT_UBX_NAV_VELNED_UART1, 1, 1);  // VELNED for GPS heading
  sendValset(CFG_MSGOUT_NMEA_GGA_UART1, 1, 1);
  sendCfgMsg(0x01, 0x07, 1);  // NAV-PVT
  sendCfgMsg(0x02, 0x32, 1);  // RXM-RTCM
  sendCfgMsg(0x01, 0x3C, 1);  // NAV-RELPOSNED
  sendCfgMsg(0x01, 0x12, 1);  // NAV-VELNED
  sendCfgMsg(0xF0, 0x00, 1);  // NMEA GGA

  Serial.println("GPS: configuration sent");
}

static void maintainGpsOutput() {
  uint32_t now = millis();
  if (now < 8000) return;
  if (g_gps.lastMs != 0 && now - g_gps.lastMs <= 3000) return;
  if (now - g_gpsPvtWatchdog < 5000) return;
  g_gpsPvtWatchdog = now;

  Serial.println("GPS: NAV-PVT stale, re-requesting");
  sendCfgMsg(0x01, 0x07, 1);
  sendCfgMsg(0x01, 0x12, 1);
  sendValset(CFG_MSGOUT_UBX_NAV_VELNED_UART1, 1, 1);
  writeUbx(0x01, 0x07, nullptr, 0);
}

static void parseNavPvt(const uint8_t* p, uint16_t len) {
  if (len < 92) return;

  const double lat = (int32_t)getU32(p + 28) * 1e-7;
  const double lon = (int32_t)getU32(p + 24) * 1e-7;
  const float hAcc = (float)getU32(p + 40);
  const float vAcc = (float)getU32(p + 44);
  const uint8_t fixType = p[20];
  const uint8_t carrier = (p[21] >> 6) & 0x03;
  const uint8_t numSV = p[23];
  if (!gpsPositionPlausible(lat, lon, hAcc, numSV)) return;
  if (fixType > 5 || carrier > 2 || numSV > 80) return;
  if (vAcc <= 0.0f || vAcc > 1000000.0f) return;

  g_gps.lat = lat;
  g_gps.lon = lon;
  g_gps.hAcc = hAcc;
  g_gps.vAcc = vAcc;
  g_gps.speed = (int32_t)getU32(p + 60) * 0.001f;
  g_gps.heading = (int32_t)getU32(p + 64) * 1e-5f;
  if (g_gps.heading < 0) g_gps.heading += 360.0f;
  g_gps.fixType = fixType;
  g_gps.diff = (p[21] & 0x02) != 0;
  g_gps.carrier = carrier;
  g_gps.valid = (p[21] & 0x01) != 0;
  g_gps.numSV = numSV;
  g_gps.lastMs = millis();
  g_gpsUbxParsed++;
}

static void parseNavVelned(const uint8_t* p, uint16_t len) {
  if (len < 36) return;

  // VELNED: velN (mm/s) at offset 4, velE at offset 8
  int32_t velN = getU32(p + 4);
  int32_t velE = getU32(p + 8);
  g_gps.velN = velN * 0.001f;  // Convert to m/s
  g_gps.velE = velE * 0.001f;
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

static void parseUbxAck(uint8_t id, const uint8_t* p, uint16_t len) {
  if (len < 2) return;
  if (id == 0x01) {
    g_ubxAck++;
  } else if (id == 0x00) {
    g_ubxNak++;
    Serial.printf("GPS NAK cls=0x%02X id=0x%02X\n", p[0], p[1]);
  }
}

static void feedGpsByte(uint8_t b) {
  g_gpsRawBytes++;

  // NMEA parser - fallback
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
        if (count >= 10 && fields[0][0] == '$' && strlen(fields[0]) >= 6 &&
            strncmp(fields[0] + 3, "GGA", 3) == 0 &&
            fields[2][0] && fields[3][0] && fields[4][0] && fields[5][0]) {
          int deg = atoi(fields[2]) / 100;
          double min = atof(fields[2]) - deg * 100;
          double lat = deg + min / 60.0;
          if (fields[3][0] == 'S') lat = -lat;
          deg = atoi(fields[4]) / 100;
          min = atof(fields[4]) - deg * 100;
          double lon = deg + min / 60.0;
          if (fields[5][0] == 'W') lon = -lon;
          int quality = atoi(fields[6]);
          int numSV = atoi(fields[7]);
          g_gps.fixType = quality > 0 ? 3 : 0;
          g_gps.carrier = (quality == 4) ? 2 : (quality == 5 ? 1 : 0);
          g_gps.diff = quality >= 2;
          g_gps.valid = quality > 0;
          float hdop = atof(fields[8]);
          const float hAcc = (hdop > 0.0f && hdop < 99.0f) ? hdop * 1000.0f : g_gps.hAcc;
          if (numSV < 0 || !gpsPositionPlausible(lat, lon, hAcc, (uint8_t)numSV)) {
            nmeaLen = 0;
            return;
          }
          g_gps.lat = lat;
          g_gps.lon = lon;
          g_gps.numSV = numSV;
          if (hdop > 0.0f && hdop < 99.0f) {
            g_gps.hAcc = hAcc;
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
      } else if (b == gpsCkB && gpsClass == 0x01 && gpsId == 0x12) {
        parseNavVelned(gpsPayload, gpsLen);
      } else if (b == gpsCkB && gpsClass == 0x02 && gpsId == 0x32) {
        parseRxmRtcm(gpsPayload, gpsLen);
      } else if (b == gpsCkB && gpsClass == 0x05) {
        parseUbxAck(gpsId, gpsPayload, gpsLen);
      }
      break;
  }
}

// ============== RTCM ==============

static constexpr size_t RTCM_PACKET_BUF_SIZE = 1200;
static constexpr uint16_t RTCM_FRAME_BUF_SIZE = 1200;
static WiFiUDP rtcmUdp;
static uint32_t g_rtcmBytes = 0;
static uint32_t g_rtcmMsgs = 0;
static uint32_t g_rtcmErrors = 0;
static uint32_t g_rtcmWriteErrors = 0;
static uint32_t g_rtcmOversize = 0;
static uint32_t g_rtcmFramesToF9p = 0;
static uint32_t g_rtcmCrcFail = 0;
static uint32_t g_rtcmFrameOverflow = 0;
static uint32_t g_lastRtcmMs = 0;
static bool g_rtcmFresh = false;
static uint8_t g_rtcmFrameBuf[RTCM_FRAME_BUF_SIZE];
static uint16_t g_rtcmFrameLen = 0;
static uint16_t g_rtcmExpectedLen = 0;

static uint16_t rtcmMessageType(const uint8_t* frame, uint16_t len) {
  if (len < 5) return 0;
  return ((uint16_t)frame[3] << 4) | (frame[4] >> 4);
}

static uint16_t rtcmPayloadLength(const uint8_t* frame) {
  return ((uint16_t)(frame[1] & 0x03) << 8) | frame[2];
}

static uint32_t rtcmCrc24q(const uint8_t* data, uint16_t len) {
  uint32_t crc = 0;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= (uint32_t)data[i] << 16;
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc <<= 1;
      if (crc & 0x1000000) crc ^= 0x1864CFB;
    }
  }
  return crc & 0xFFFFFF;
}

static bool rtcmCrcOk(const uint8_t* frame, uint16_t len) {
  if (len < 6) return false;
  const uint32_t expected = ((uint32_t)frame[len - 3] << 16) |
                            ((uint32_t)frame[len - 2] << 8) |
                            frame[len - 1];
  return rtcmCrc24q(frame, len - 3) == expected;
}

static void resetRtcmFrameAssembler() {
  g_rtcmFrameLen = 0;
  g_rtcmExpectedLen = 0;
}

static bool resyncRtcmFrameAssembler() {
  for (uint16_t i = 1; i + 2 < g_rtcmFrameLen; i++) {
    if (g_rtcmFrameBuf[i] != 0xD3) continue;
    if ((g_rtcmFrameBuf[i + 1] & 0xFC) != 0) continue;
    const uint16_t payloadLen = ((uint16_t)(g_rtcmFrameBuf[i + 1] & 0x03) << 8) | g_rtcmFrameBuf[i + 2];
    const uint16_t expectedLen = payloadLen + 6;
    if (payloadLen == 0 || expectedLen > RTCM_FRAME_BUF_SIZE) continue;

    const uint16_t remaining = g_rtcmFrameLen - i;
    memmove(g_rtcmFrameBuf, g_rtcmFrameBuf + i, remaining);
    g_rtcmFrameLen = remaining;
    g_rtcmExpectedLen = expectedLen;
    return true;
  }

  resetRtcmFrameAssembler();
  return false;
}

static void writeRtcmFrameToF9p(const uint8_t* frame, uint16_t len) {
  const size_t written = GpsSerial.write(frame, len);
  if (written != (size_t)len) {
    g_rtcmWriteErrors++;
    return;
  }
  g_rtcmFramesToF9p++;
  g_rtcmLastType = rtcmMessageType(frame, len);
}

static void feedRtcmByte(uint8_t b) {
  if (g_rtcmFrameLen == 0) {
    if (b != 0xD3) return;
    g_rtcmFrameBuf[g_rtcmFrameLen++] = b;
    g_rtcmExpectedLen = 0;
    return;
  }

  if (g_rtcmFrameLen == 1 && (b & 0xFC) != 0) {
    resetRtcmFrameAssembler();
    return;
  }

  if (g_rtcmFrameLen >= RTCM_FRAME_BUF_SIZE) {
    g_rtcmFrameOverflow++;
    resetRtcmFrameAssembler();
    return;
  }

  g_rtcmFrameBuf[g_rtcmFrameLen++] = b;

  if (g_rtcmFrameLen == 3) {
    const uint16_t payloadLen = rtcmPayloadLength(g_rtcmFrameBuf);
    g_rtcmExpectedLen = payloadLen + 6;
    if (payloadLen == 0 || g_rtcmExpectedLen > RTCM_FRAME_BUF_SIZE) {
      g_rtcmFrameOverflow++;
      resetRtcmFrameAssembler();
    }
    return;
  }

  if (g_rtcmExpectedLen > 0 && g_rtcmFrameLen >= g_rtcmExpectedLen) {
    if (rtcmCrcOk(g_rtcmFrameBuf, g_rtcmExpectedLen)) {
      writeRtcmFrameToF9p(g_rtcmFrameBuf, g_rtcmExpectedLen);
      resetRtcmFrameAssembler();
    } else {
      g_rtcmCrcFail++;
      if (!resyncRtcmFrameAssembler()) return;
      if (g_rtcmExpectedLen > 0 && g_rtcmFrameLen >= g_rtcmExpectedLen &&
          rtcmCrcOk(g_rtcmFrameBuf, g_rtcmExpectedLen)) {
        writeRtcmFrameToF9p(g_rtcmFrameBuf, g_rtcmExpectedLen);
        resetRtcmFrameAssembler();
      }
    }
  }
}

static void relayRtcm() {
  for (uint8_t packets = 0; packets < 64; packets++) {
    int pktSize = rtcmUdp.parsePacket();
    if (pktSize <= 0) break;

    uint8_t buf[RTCM_PACKET_BUF_SIZE];
    if ((size_t)pktSize > sizeof(buf)) {
      rtcmUdp.read(buf, sizeof(buf));
      while (rtcmUdp.available()) rtcmUdp.read();
      g_rtcmOversize++;
      g_rtcmErrors++;
      return;
    }
    int len = rtcmUdp.read(buf, sizeof(buf));
    if (len > 0) {
      if (len != pktSize) {
        g_rtcmErrors++;
        return;
      }
      for (int i = 0; i < len; i++) feedRtcmByte(buf[i]);
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
static float g_imuYawRate = 0;
static uint32_t g_lastImuMs = 0;
static bool g_imuFresh = false;
static bool g_imuOk = false;
static float g_imuCalibrationOffset = 0;
// This robot's BNO085 mounting reports decreasing yaw during a physical
// clockwise/right turn. Navigation uses compass heading convention where
// clockwise/right turns increase heading, so invert yaw by default.
static bool g_invertYaw = true;
static uint32_t g_imuFirstReadMs = 0;
static uint32_t g_lastImuReenableMs = 0;

static bool enableImuReport() {
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, IMU_REPORT_INTERVAL_US)) {
    Serial.println("IMU: ERROR - can't enable game rotation vector!");
    return false;
  }
  Serial.println("IMU: game rotation vector enabled");
  return true;
}

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

  if (!enableImuReport()) {
    return false;
  }
  g_imuOk = true;
  g_lastImuReenableMs = millis();
  return true;
}

static void updateImu() {
  if (!g_imuOk) return;

  static float prevYaw = 0;
  static uint32_t prevYawMs = 0;

  sh2_SensorValue_t value;
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

      // Calculate yaw rate (deg/s)
      uint32_t now = millis();
      if (prevYawMs > 0) {
        float dt = (now - prevYawMs) / 1000.0f;
        if (dt > 0 && dt < 1.0f) {
          float delta = rawYaw - prevYaw;
          if (delta > 180) delta -= 360;
          if (delta < -180) delta += 360;
          g_imuYawRate = delta / dt;
        }
      }
      prevYaw = rawYaw;
      prevYawMs = now;

      g_imuRawYaw = normalizeAngle360(rawYaw);
      g_imuYaw = normalizeAngle360(g_imuRawYaw + g_imuCalibrationOffset);
      g_lastImuMs = now;
      g_imuFresh = true;

      if (g_imuFirstReadMs == 0) {
        g_imuFirstReadMs = now;
      }
    }
  }
  if (g_lastImuMs > 0 && millis() - g_lastImuMs > IMU_TIMEOUT_MS) {
    g_imuFresh = false;
    const uint32_t now = millis();
    if (now - g_lastImuReenableMs >= IMU_REENABLE_MS) {
      g_lastImuReenableMs = now;
      Serial.println("IMU: stale, re-enabling game rotation vector");
      enableImuReport();
    }
  }
}

// ============== IMU CALIBRATION SYSTEM ==============

// Estimator struct (defined here to use in calibration)
struct Estimator {
  LocalCoords pos;
  LocalCoords vel;
  float heading = 0;
  float headingSource = 0;  // 0=imu, 1=gps, 2=blended
  float speed = 0;
  uint32_t lastUpdateMs = 0;
  bool rtkFixed = false;
  NavQuality quality = QUAL_NAV_ERROR;
  uint32_t qualityAgeMs = 0;
};
static Estimator g_est;

enum CalState {
  CAL_IDLE,
  CAL_MEASURING,
  CAL_VERIFYING,
  CAL_COMPLETE,
  CAL_FAILED
};

static const char* calStateString(CalState s) {
  switch (s) {
    case CAL_IDLE: return "IDLE";
    case CAL_MEASURING: return "MEASURING";
    case CAL_VERIFYING: return "VERIFYING";
    case CAL_COMPLETE: return "COMPLETE";
    case CAL_FAILED: return "FAILED";
  }
  return "UNKNOWN";
}

struct ImuCalibration {
  CalState state = CAL_IDLE;
  float offset = 0;
  float stdDev = 0;
  uint32_t startTimeMs = 0;
  uint32_t quality = 0;  // 0-100
  bool verified = false;
  uint32_t lastVerifyMs = 0;
};

static ImuCalibration g_cal;
static float g_imuSamples[IMU_CAL_SAMPLE_COUNT];
static uint16_t g_imuSampleCount = 0;
static uint32_t g_imuLastSampleMs = 0;

static bool imuReadyForCalibration() {
  return g_imuFresh && g_gps.valid && g_gps.carrier == 2 && g_gps.hAcc < RTK_FIXED_HACC_MM;
}

static void startImuCalibration() {
  if (!imuReadyForCalibration()) {
    Serial.println("CAL: Cannot start - prerequisites not met");
    return;
  }

  g_cal.state = CAL_MEASURING;
  g_cal.startTimeMs = millis();
  g_imuSampleCount = 0;
  g_imuLastSampleMs = 0;
  Serial.println("CAL: Starting IMU calibration...");
}

static void updateCalibration() {
  uint32_t now = millis();

  switch (g_cal.state) {
    case CAL_IDLE:
      // Check if auto-recalibration needed
      if (now - g_cal.lastVerifyMs > IMU_DRIFT_CHECK_INTERVAL_MS) {
        if (imuReadyForCalibration() && g_gps.speed > GPS_HEADING_MIN_SPEED_MPS) {
          // Check drift
          float gpsHeading = atan2f(g_gps.velE, g_gps.velN) * 180.0f / PI;
          if (gpsHeading < 0) gpsHeading += 360.0f;
          float imuHeading = g_imuYaw;
          float drift = normalizeAngle(gpsHeading - imuHeading);
          if (fabsf(drift) > IMU_DRIFT_THRESHOLD_DEG) {
            Serial.printf("CAL: Drift detected (%.1f deg), starting recalibration\n", drift);
            startImuCalibration();
          }
          g_cal.lastVerifyMs = now;
        }
      }
      break;

    case CAL_MEASURING: {
      // Collect IMU samples
      if (g_imuFresh && now - g_imuLastSampleMs >= IMU_CAL_SAMPLE_INTERVAL_MS) {
        if (g_imuSampleCount < IMU_CAL_SAMPLE_COUNT) {
          g_imuSamples[g_imuSampleCount++] = g_imuRawYaw;
          g_imuLastSampleMs = now;
        }

        // Check if collection complete
        if (g_imuSampleCount >= IMU_CAL_SAMPLE_COUNT) {
          // Calculate mean and std dev
          float sum = 0;
          for (uint16_t i = 0; i < g_imuSampleCount; i++) {
            sum += g_imuSamples[i];
          }
          float mean = sum / g_imuSampleCount;

          float varSum = 0;
          for (uint16_t i = 0; i < g_imuSampleCount; i++) {
            float diff = g_imuSamples[i] - mean;
            if (diff > 180) diff -= 360;
            if (diff < -180) diff += 360;
            varSum += diff * diff;
          }
          g_cal.stdDev = sqrtf(varSum / g_imuSampleCount);

          Serial.printf("CAL: Collected %u samples, mean=%.2f, stddev=%.2f deg\n",
            g_imuSampleCount, mean, g_cal.stdDev);

          if (g_cal.stdDev > IMU_CAL_STD_DEV_MAX_DEG) {
            Serial.println("CAL: FAILED - IMU too noisy");
            g_cal.state = CAL_FAILED;
          } else {
            // Move to verification - need to measure GPS heading
            Serial.println("CAL: Measurement complete, waiting for movement verification...");
            g_cal.state = CAL_VERIFYING;
            g_cal.offset = mean;  // Initial offset = raw mean
          }
        }
      }

      // Timeout after 15 seconds
      if (now - g_cal.startTimeMs > 15000) {
        Serial.println("CAL: FAILED - timeout");
        g_cal.state = CAL_FAILED;
      }
      break;
    }

    case CAL_VERIFYING: {
      // Need robot to move for GPS heading verification
      if (g_gps.speed > GPS_HEADING_MIN_SPEED_MPS) {
        // Calculate GPS heading from velocity
        float gpsHeading = atan2f(g_gps.velE, g_gps.velN) * 180.0f / PI;
        if (gpsHeading < 0) gpsHeading += 360.0f;

        // IMU heading = raw yaw + offset
        float imuHeading = g_imuRawYaw + g_cal.offset;
        imuHeading = normalizeAngle360(imuHeading);

        // Calculate error
        float error = normalizeAngle(gpsHeading - imuHeading);
        Serial.printf("CAL: Verify - GPS=%.1f IMU=%.1f offset=%.1f error=%.1f\n",
          gpsHeading, imuHeading, g_cal.offset, error);

        // Check if error is acceptable
        if (fabsf(error) <= 5.0f) {
          // Validate offset is reasonable before saving
          if (fabsf(g_cal.offset) > 90.0f) {
            Serial.printf("CAL: WARNING - final offset %.1f is suspiciously large, remeasuring\n", g_cal.offset);
            g_cal.state = CAL_MEASURING;
            g_cal.startTimeMs = now;
            g_imuSampleCount = 0;
          } else {
            // Calibration OK - save
            g_imuCalibrationOffset = g_cal.offset;
            g_cal.state = CAL_COMPLETE;
            g_cal.verified = true;
            g_cal.quality = 100;
            g_est.heading = normalizeAngle360(g_imuRawYaw + g_imuCalibrationOffset);
            saveNavPrefs();
            Serial.printf("CAL: SUCCESS! Saved offset=%.2f\n", g_imuCalibrationOffset);
          }
        } else if (fabsf(error) <= 15.0f) {
          // Small correction needed
          float newOffset = normalizeAngle(g_cal.offset + error);
          // Validate new offset
          if (fabsf(newOffset) > 90.0f) {
            Serial.printf("CAL: Correction would produce large offset %.1f, remeasuring\n", newOffset);
            g_cal.state = CAL_MEASURING;
            g_cal.startTimeMs = now;
            g_imuSampleCount = 0;
          } else {
            g_cal.offset = newOffset;
            g_cal.verified = true;
            Serial.printf("CAL: Adjusted offset to %.2f\n", g_cal.offset);
          }
        } else {
          // Large error - remeasure
          Serial.println("CAL: Large error, remeasuring...");
          g_cal.state = CAL_MEASURING;
          g_cal.startTimeMs = now;
          g_imuSampleCount = 0;
        }
      }

      // Timeout after 30 seconds of no movement
      if (now - g_cal.startTimeMs > 30000) {
        Serial.println("CAL: FAILED - no movement for verification");
        g_cal.state = CAL_FAILED;
      }
      break;
    }

    case CAL_COMPLETE:
    case CAL_FAILED:
      // Return to idle after delay
      if (now - g_cal.startTimeMs > 3000) {
        g_cal.state = CAL_IDLE;
      }
      break;
  }
}

// ============== ESTIMATOR STATE ==============

static uint32_t g_lastGoodRtkMs = 0;
static bool g_haveRtkFix = false;
// Set true once the GPS-velocity heading EMA filter has at least one sample.
// Reset on START so the bearing-to-first-WP seed doesn't get blended with
// the previous run's heading.
static bool g_headingFilterSeeded = false;

static void updateEstimator() {
  uint32_t now = millis();
  uint32_t gpsAge = g_gps.lastMs ? now - g_gps.lastMs : 99999;
  uint32_t f9pRtcmAge = g_lastF9pRtcmMs ? now - g_lastF9pRtcmMs : 99999;
  uint32_t rtcmTransportAge = g_lastRtcmMs ? now - g_lastRtcmMs : 99999;
  bool correctionsFresh = (f9pRtcmAge <= RTK_CORRECTION_TIMEOUT_MS) ||
                          (rtcmTransportAge <= RTK_CORRECTION_TIMEOUT_MS && g_gps.diff);

  // Determine quality
  NavQuality qual = QUAL_NAV_ERROR;
  if (g_gps.valid && g_gps.fixType >= 3) {
    if (g_gps.carrier == 2 && g_gps.hAcc <= RTK_FIXED_HACC_MM && gpsAge <= RTK_FIXED_AGE_MS && correctionsFresh) {
      qual = QUAL_RTK_FIXED_GOOD;
    } else if (g_gps.carrier >= 1 && g_gps.hAcc <= RTK_FLOAT_HACC_MM && gpsAge <= RTK_FLOAT_AGE_MS && correctionsFresh) {
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
    qual = QUAL_NAV_ERROR;
  }

  g_est.quality = qual;
  g_est.qualityAgeMs = gpsAge;

  // Update position estimate
  if (qual == QUAL_RTK_FIXED_GOOD || qual == QUAL_RTK_FLOAT_OK || qual == QUAL_GPS_DEGRADED) {
    LocalCoords local = toLocal(g_gps.lat, g_gps.lon);
    if (!g_origin.valid) {
      setOrigin(g_gps.lat, g_gps.lon);
      resetGpsFilter();
      local = {0, 0};
    }

    // Apply GPS position filtering (median filter + outlier rejection)
    updateGpsFilter(local, g_gps.hAcc);

    // Apply weighted average for high hAcc
    if (g_gps.hAcc > GPS_HACC_WEIGHT_THRESHOLD_MM) {
      applyWeightedGpsUpdate(&local, g_gps.hAcc);
    }

    const float f9pVelMag = hypotf(g_gps.velE, g_gps.velN);
    const float f9pSpeed = max(g_gps.speed, f9pVelMag);
    // Hysteresis on the velocity threshold: previously 0.02 m/s, but at
    // robot speeds near the gate (0.18 m/s) GPS-velocity noise (~5-10 mm/s)
    // makes f9pVelMag hop across 0.02 every other tick, toggling between
    // raw velE/velN and the heading-based fallback. That toggling fed the
    // heading estimator directly and made the heading wobble. Use a tighter
    // gate so velocity either engages or stays off, not flickers.
    constexpr float VEL_ENGAGE_MPS = 0.05f;
    if (f9pVelMag > VEL_ENGAGE_MPS) {
      g_est.vel.x = g_gps.velE;
      g_est.vel.y = g_gps.velN;
    } else if (f9pSpeed > VEL_ENGAGE_MPS) {
      const float headingRad = degToRad(g_gps.heading);
      g_est.vel.x = f9pSpeed * sinf(headingRad);
      g_est.vel.y = f9pSpeed * cosf(headingRad);
    } else {
      g_est.vel.x = 0.0f;
      g_est.vel.y = 0.0f;
    }
    g_est.speed = f9pSpeed;
    g_est.pos = local;
    g_est.lastUpdateMs = now;
    const bool rtkUsable = (qual == QUAL_RTK_FIXED_GOOD || qual == QUAL_RTK_FLOAT_OK);
    if (rtkUsable) {
      g_lastGoodRtkMs = now;
      g_haveRtkFix = true;
    }
    g_est.rtkFixed = (g_gps.carrier == 2);

    // Update heading from GPS velocity — always, even when IMU is present.
    // IMU can have calibration errors that cause oscillation. GPS heading is
    // stable and sufficient for navigation when RTK is fixed/float.
    // Heading is low-pass filtered (EMA, alpha=HEADING_EMA_ALPHA) so that
    // GPS velocity noise (a few mm/s at 0.18 m/s gives ~2-3° per tick) does
    // not feed the turn controller and cause the robot to wiggle in place.
    if (g_est.speed > GPS_HEADING_MIN_SPEED_MPS) {
      float gpsHeading = atan2f(g_est.vel.x, g_est.vel.y) * 180.0f / PI;
      if (gpsHeading < 0) gpsHeading += 360.0f;
      if (!g_headingFilterSeeded) {
        g_est.heading = gpsHeading;
        g_headingFilterSeeded = true;
      } else {
        float diff = gpsHeading - g_est.heading;
        if (diff > 180.0f) diff -= 360.0f;
        if (diff < -180.0f) diff += 360.0f;
        g_est.heading = normalizeAngle360(g_est.heading + HEADING_EMA_ALPHA * diff);
      }
      g_est.headingSource = 1;  // GPS
    }
    // When stationary, keep last known heading (no IMU fallback)
  } else if (qual == QUAL_GPS_HOLD_SHORT && g_est.lastUpdateMs > 0) {
    // Dead reckoning - use last known heading
    float dt = (now - g_est.lastUpdateMs) / 1000.0f;
    if (dt > 0 && dt < 0.5f && g_est.speed > 0.01f) {
      float headingRad = degToRad(g_est.heading);
      g_est.pos.x += g_est.speed * sin(headingRad) * dt * 0.8f;
      g_est.pos.y += g_est.speed * cos(headingRad) * dt * 0.8f;
    }
    g_est.lastUpdateMs = now;
  } else {
    g_est.speed = 0;
    g_est.vel = {0, 0};
  }
}

// ============== MOVEMENT MONITOR ==============

struct MovementMonitor {
  float distHistory[MONITOR_HISTORY_SIZE];
  uint32_t timeHistory[MONITOR_HISTORY_SIZE];
  uint16_t head = 0;
  uint16_t count = 0;

  // Analysis results
  float progressRate = 0;       // m/s (negative = approaching)
  bool isApproaching = false;
  bool isStuck = false;
  bool isGoingWrong = false;
  float actualSpeed = 0;
  float actualHeading = 0;
  float headingError = 0;
  float commandedSpeed = 0;
  float commandedHeading = 0;
  float crossTrackError = 0;
  uint32_t lastUpdateMs = 0;
};

static MovementMonitor g_mon;

static void resetMovementMonitor() {
  g_mon.head = 0;
  g_mon.count = 0;
  g_mon.progressRate = 0;
  g_mon.isApproaching = false;
  g_mon.isStuck = false;
  g_mon.isGoingWrong = false;
  g_mon.actualSpeed = 0;
  g_mon.actualHeading = 0;
  g_mon.headingError = 0;
  g_mon.lastUpdateMs = 0;
}

static void updateMovementMonitor(LocalCoords target, float cmdSpeed, float cmdHeading) {
  uint32_t now = millis();

  // Calculate current distance to target
  float dist = hypotf(target.x - g_est.pos.x, target.y - g_est.pos.y);

  // Add to history
  g_mon.distHistory[g_mon.head] = dist;
  g_mon.timeHistory[g_mon.head] = now;
  g_mon.head = (g_mon.head + 1) % MONITOR_HISTORY_SIZE;
  if (g_mon.count < MONITOR_HISTORY_SIZE) g_mon.count++;

  // Update commanded values
  g_mon.commandedSpeed = cmdSpeed;
  g_mon.commandedHeading = cmdHeading;

  // Update actual values from GPS
  g_mon.actualSpeed = g_est.speed;
  if (g_est.speed > GPS_HEADING_MIN_SPEED_MPS) {
    g_mon.actualHeading = atan2f(g_est.vel.x, g_est.vel.y) * 180.0f / PI;
    if (g_mon.actualHeading < 0) g_mon.actualHeading += 360.0f;
  }

  // Calculate heading error
  if (g_est.speed > 0.1f) {
    g_mon.headingError = normalizeAngle(g_mon.commandedHeading - g_mon.actualHeading);
  }

  // Calculate progress rate
  if (g_mon.count >= 10) {
    uint16_t oldestIdx = (g_mon.head + MONITOR_HISTORY_SIZE - g_mon.count) % MONITOR_HISTORY_SIZE;
    uint16_t newestIdx = (g_mon.head + MONITOR_HISTORY_SIZE - 1) % MONITOR_HISTORY_SIZE;

    float oldestDist = g_mon.distHistory[oldestIdx];
    float newestDist = g_mon.distHistory[newestIdx];
    float timeDiff = (g_mon.timeHistory[newestIdx] - g_mon.timeHistory[oldestIdx]) / 1000.0f;

    if (timeDiff > 1.0f) {
      g_mon.progressRate = (newestDist - oldestDist) / timeDiff;
      g_mon.isApproaching = g_mon.progressRate < -0.02f;
      g_mon.isGoingWrong = g_mon.progressRate > 0.02f;
      g_mon.isStuck = g_est.speed < STUCK_SPEED_THRESHOLD_MPS &&
                      g_mon.commandedSpeed > STUCK_SPEED_THRESHOLD_MPS;
    }
  }

  // Cross track error
  g_mon.crossTrackError = 0;  // Calculated in nav update

  g_mon.lastUpdateMs = now;
}

static const char* getMovementStatus() {
  if (g_mon.isStuck) return "STUCK";
  if (g_mon.isGoingWrong) return "WRONG_DIR";
  if (g_mon.isApproaching) return "APPROACHING";
  if (g_mon.actualSpeed < 0.05f && g_mon.commandedSpeed > 0.1f) return "STARTING";
  return "OK";
}

// ============== ARRIVAL DETECTION ==============

struct ArrivalDetector {
  float arrivalTimer = 0;
  float distanceAtArrival = 999;
  uint8_t gpsConfirmCount = 0;
  LocalCoords lastPositions[5];
  uint8_t posIndex = 0;
  bool arrived = false;
};

static ArrivalDetector g_arrival;

static void resetArrivalDetector() {
  g_arrival.arrivalTimer = 0;
  g_arrival.distanceAtArrival = 999;
  g_arrival.gpsConfirmCount = 0;
  g_arrival.posIndex = 0;
  g_arrival.arrived = false;
  resetGpsFilter();
}

static bool checkArrival(LocalCoords target, float commandedSpeed, float headingError) {
  uint32_t now = millis();

  float dist = hypotf(target.x - g_est.pos.x, target.y - g_est.pos.y);
  float speed = g_est.speed;

  // Only save position for verification when we're actually close to target
  // This prevents mixing positions from different contexts
  if (dist < ARRIVAL_DIST_M * 2.0f) {
    g_arrival.lastPositions[g_arrival.posIndex] = g_est.pos;
    g_arrival.posIndex = (g_arrival.posIndex + 1) % 5;
  }

  // Check if we meet arrival criteria
  bool withinThreshold = dist < ARRIVAL_DIST_M;
  bool stopped = speed < 0.03f;
  if (withinThreshold && stopped) {
    g_arrival.arrivalTimer += NAV_LOOP_MS / 1000.0f;
    g_arrival.distanceAtArrival = dist;

    // GPS verification: check last 5 positions are consistent
    if (g_arrival.gpsConfirmCount < 5) {
      g_arrival.gpsConfirmCount++;
    }

    // Check position consistency
    bool consistent = true;
    for (uint8_t i = 0; i < g_arrival.gpsConfirmCount - 1; i++) {
      uint8_t idx1 = (g_arrival.posIndex + 5 - g_arrival.gpsConfirmCount + i) % 5;
      uint8_t idx2 = (g_arrival.posIndex + 5 - g_arrival.gpsConfirmCount + i + 1) % 5;
      float posDiff = hypotf(
        g_arrival.lastPositions[idx1].x - g_arrival.lastPositions[idx2].x,
        g_arrival.lastPositions[idx1].y - g_arrival.lastPositions[idx2].y
      );
      if (posDiff > 0.1f) consistent = false;
    }

    if (g_arrival.arrivalTimer >= ARRIVAL_CONFIRM_TIME_S && consistent) {
      g_arrival.arrived = true;
      return true;
    }
  } else {
    // Reset if we moved away
    if (dist > ARRIVAL_DIST_M + 0.1f) {
      g_arrival.arrivalTimer = 0;
      g_arrival.gpsConfirmCount = 0;
    }
  }

  return false;
}

// ============== RECOVERY SYSTEM ==============

struct RecoverySystem {
  uint32_t recoveryStartMs = 0;
  uint32_t recoveryAttempts = 0;
  LocalCoords lastGoodPosition;
  float lastGoodHeading = 0;
  bool recoveryActive = false;
};

static RecoverySystem g_recovery;

static void triggerRecovery(const char* reason) {
  uint32_t now = millis();

  Serial.printf("RECOVERY: Triggered - %s\n", reason);
  Serial.printf("  Position: (%.3f, %.3f)\n", g_est.pos.x, g_est.pos.y);
  Serial.printf("  Heading: %.1f\n", g_est.heading);
  Serial.printf("  Speed: %.3f m/s, Progress: %.3f m/s\n", g_mon.actualSpeed, g_mon.progressRate);

  g_recovery.lastGoodPosition = g_est.pos;
  g_recovery.lastGoodHeading = g_est.heading;
  g_recovery.recoveryStartMs = now;
  g_recovery.recoveryActive = true;
  g_recovery.recoveryAttempts++;

  // Stop motors during recovery
  g_targetLeft = 0;
  g_targetRight = 0;
  g_curLeft = 0;
  g_curRight = 0;
}

static void updateRecovery() {
  if (!g_recovery.recoveryActive) return;

  uint32_t now = millis();
  uint32_t elapsed = now - g_recovery.recoveryStartMs;

  // Check if we should exit recovery. Heading is GPS-only now; do not require
  // IMU freshness here, or the recovery can never end when IMU is stale.
  bool gpsGood = g_est.quality == QUAL_RTK_FIXED_GOOD || g_est.quality == QUAL_RTK_FLOAT_OK;

  if (gpsGood && elapsed > 2000) {
    // Verify position is consistent
    float posDrift = hypotf(g_est.pos.x - g_recovery.lastGoodPosition.x,
                            g_est.pos.y - g_recovery.lastGoodPosition.y);

    if (posDrift < 0.5f) {
      Serial.printf("RECOVERY: Success after %lu ms, attempts=%lu\n",
        (unsigned long)elapsed, (unsigned long)g_recovery.recoveryAttempts);
      g_recovery.recoveryActive = false;
      return;
    }
  }

  // Timeout recovery after 10 seconds
  if (elapsed > 10000) {
    Serial.println("RECOVERY: Timeout - manual intervention required");
    g_recovery.recoveryActive = false;
  }
}

// ============== ROUTE ==============

static constexpr uint8_t MAX_WAYPOINTS = 254;
static constexpr uint8_t MAX_AREA_POINTS = 32;
static constexpr uint8_t MAX_FORBIDDEN_POLYGONS = 8;
static constexpr uint8_t MAX_FORBIDDEN_POINTS = 24;

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
static LocalCoords g_forbidden[MAX_FORBIDDEN_POLYGONS][MAX_FORBIDDEN_POINTS];
static bool g_forbiddenReceived[MAX_FORBIDDEN_POLYGONS][MAX_FORBIDDEN_POINTS];
static uint8_t g_forbiddenPointCount[MAX_FORBIDDEN_POLYGONS];
static uint8_t g_forbiddenCount = 0;
static bool g_forbiddenReady = true;
static float g_forbiddenClearanceM = NAN;
static uint32_t g_segmentStartMs = 0;
static uint8_t g_segmentStartIndex = 0;
static uint32_t g_offRouteSinceMs = 0;

static NavState g_navState = STATE_IDLE;
static const char* g_navReason = "idle";

// Pure pursuit
static constexpr float LOOKAHEAD_MIN_M = 0.22f;
static constexpr float LOOKAHEAD_MAX_M = 0.60f;
static constexpr float LOOKAHEAD_SPEED_GAIN = 0.55f;
// Runtime tunable gains (changed via SIM_SETPARAMS or WebSocket)
static float g_kHeading = 0.85f;
static float g_kCrossTrack = 30.0f;

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

// Extended telemetry
static float g_crossTrackError = 0;
static float g_headingError = 0;
static float g_distToRouteEnd = 0;
static float g_distToNextWaypoint = 0;
static navcore::NavigationCore g_navCore;

// USB hardware-in-the-loop mode. The ESP32 still runs rover.cpp and
// NavigationCore, while the PC supplies simulated sensors over Serial.
static bool g_simulationMode = false;
static bool g_simVerbose = false;
static char g_simSerialLine[160];
static uint8_t g_simSerialLen = 0;
static uint32_t g_simTick = 0;

static navcore::NavQuality toCoreQuality(NavQuality quality) {
  switch (quality) {
    case QUAL_RTK_FIXED_GOOD: return navcore::NavQuality::Fixed;
    case QUAL_RTK_FLOAT_OK: return navcore::NavQuality::FloatOk;
    case QUAL_GPS_DEGRADED: return navcore::NavQuality::Degraded;
    case QUAL_GPS_HOLD_SHORT: return navcore::NavQuality::GpsHold;
    default: return navcore::NavQuality::None;
  }
}

static navcore::NavConfig makeNavCoreConfig() {
  navcore::NavConfig config;
  config.maxSpeedMps = MAX_SPEED;
  config.floatSpeedMps = FLOAT_SPEED;
  config.degradedSpeedMps = DEGRADED_SPEED;
  config.holdSpeedMps = HOLD_SPEED;
  config.maxSpeedPercent = MAX_SPEED_PERCENT;
  config.arrivalDistanceM = ARRIVAL_DIST_M;
  config.arrivalApproachDistanceM = ARRIVAL_APPROACH_DIST_M;
  config.enableArrivalAdvance = false;
  config.lookaheadMinM = LOOKAHEAD_MIN_M;
  config.lookaheadMaxM = LOOKAHEAD_MAX_M;
  config.lookaheadSpeedGain = LOOKAHEAD_SPEED_GAIN;
  config.headingGain = g_kHeading;
  config.crossTrackGain = g_kCrossTrack;
  // alignFirst OFF: heading is GPS-velocity-derived and only updates while moving.
  // If we hold forward at 0 until heading aligns, we never start moving and heading
  // never updates — deadlock. Always apply turn correction; pure pursuit will arc
  // toward the target if heading is off.
  config.alignFirst = false;
  config.alignFirstThresholdDeg = 180.0f;
  config.forwardScale = g_navForwardScale;
  config.turnScale = g_navTurnScale;
  config.invertForward = g_invertForward;
  config.invertSteering = g_invertSteering;
  return config;
}

static void syncNavCoreRoute() {
  g_navCore.clearRoute();
  for (uint8_t i = 0; i < g_routeCount; i++) {
    g_navCore.setWaypoint(i, g_route[i].pos.x, g_route[i].pos.y, g_routeReceived[i]);
  }
  g_navCore.setWaypointCount(g_routeCount);
  g_navCore.setCurrentWaypointIndex(g_routeIndex);
}

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

static void clearForbiddenZones() {
  g_forbiddenCount = 0;
  g_forbiddenReady = true;
  g_forbiddenClearanceM = NAN;
  memset(g_forbiddenReceived, 0, sizeof(g_forbiddenReceived));
  memset(g_forbiddenPointCount, 0, sizeof(g_forbiddenPointCount));
}

static bool forbiddenReady() {
  if (g_forbiddenCount == 0) return true;
  for (uint8_t poly = 0; poly < g_forbiddenCount; poly++) {
    const uint8_t count = g_forbiddenPointCount[poly];
    if (count < 3) return false;
    for (uint8_t i = 0; i < count; i++) {
      if (!g_forbiddenReceived[poly][i]) return false;
    }
  }
  return true;
}

static float pointSegmentDistance(LocalCoords p, LocalCoords a, LocalCoords b) {
  const float dx = b.x - a.x;
  const float dy = b.y - a.y;
  const float len2 = dx * dx + dy * dy;
  if (len2 < 1e-6f) return hypotf(p.x - a.x, p.y - a.y);
  float t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2;
  t = constrain(t, 0.0f, 1.0f);
  const float px = a.x + t * dx;
  const float py = a.y + t * dy;
  return hypotf(p.x - px, p.y - py);
}

static bool pointInPolygon(LocalCoords p, const LocalCoords* poly, uint8_t count) {
  bool inside = false;
  for (uint8_t i = 0, j = count - 1; i < count; j = i++) {
    const LocalCoords pi = poly[i];
    const LocalCoords pj = poly[j];
    const bool crosses = ((pi.y > p.y) != (pj.y > p.y)) &&
      (p.x < (pj.x - pi.x) * (p.y - pi.y) / ((pj.y - pi.y) + 1e-6f) + pi.x);
    if (crosses) inside = !inside;
  }
  return inside;
}

static float polygonDistance(LocalCoords p, const LocalCoords* poly, uint8_t count) {
  float best = 1e6f;
  for (uint8_t i = 0; i < count; i++) {
    const LocalCoords a = poly[i];
    const LocalCoords b = poly[(i + 1) % count];
    best = min(best, pointSegmentDistance(p, a, b));
  }
  return best;
}

static float forbiddenClearance(LocalCoords p, bool* insideAny) {
  if (insideAny != nullptr) *insideAny = false;
  if (!g_forbiddenReady || g_forbiddenCount == 0) return NAN;

  float best = 1e6f;
  for (uint8_t poly = 0; poly < g_forbiddenCount; poly++) {
    const uint8_t count = g_forbiddenPointCount[poly];
    if (count < 3) continue;
    if (pointInPolygon(p, g_forbidden[poly], count)) {
      if (insideAny != nullptr) *insideAny = true;
      return 0.0f;
    }
    best = min(best, polygonDistance(p, g_forbidden[poly], count));
  }
  return best;
}

static void markSegmentStart() {
  g_segmentStartMs = millis();
  g_segmentStartIndex = g_routeIndex;
}

static float currentSegmentProgressM() {
  if (g_routeIndex == 0 || g_routeIndex >= g_routeCount) return 1e6f;

  const LocalCoords a = g_route[g_routeIndex - 1].pos;
  const LocalCoords b = g_route[g_routeIndex].pos;
  const float dx = b.x - a.x;
  const float dy = b.y - a.y;
  const float len = hypotf(dx, dy);
  if (len < 1e-6f) return 1e6f;
  const float progress = ((g_est.pos.x - a.x) * dx + (g_est.pos.y - a.y) * dy) / len;
  return constrain(progress, 0.0f, len);
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

static bool appendAreaSnakeLines(bool horizontal, float minX, float maxX, float minY, float maxY, float step) {
  const float width = maxX - minX;
  const float height = maxY - minY;
  const float span = horizontal ? height : width;
  const float first = (span >= step) ? ((horizontal ? minY : minX) + step * 0.5f)
                                    : ((horizontal ? minY + height * 0.5f : minX + width * 0.5f));
  const float last = (span >= step) ? ((horizontal ? maxY : maxX) - step * 0.5f)
                                   : first;
  bool reverse = false;
  bool added = false;
  float intersections[MAX_AREA_POINTS];

  for (float line = first; line <= last + 0.001f; line += step) {
    const uint8_t n = horizontal
      ? scanHorizontalIntersections(line, intersections, MAX_AREA_POINTS)
      : scanVerticalIntersections(line, intersections, MAX_AREA_POINTS);

    for (uint8_t i = 0; i + 1 < n; i += 2) {
      LocalCoords a = horizontal ? LocalCoords{intersections[i], line}
                                 : LocalCoords{line, intersections[i]};
      LocalCoords b = horizontal ? LocalCoords{intersections[i + 1], line}
                                 : LocalCoords{line, intersections[i + 1]};
      const bool ok = reverse ? (addRoutePoint(b) && addRoutePoint(a))
                              : (addRoutePoint(a) && addRoutePoint(b));
      if (!ok) return false;
      reverse = !reverse;
      added = true;
    }

    if (span < step) break;
  }

  return added;
}

static bool buildRouteFromArea() {
  if (!areaReady() || !g_origin.valid) return false;

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

  const bool primaryHorizontal = width >= height;
  bool ok = appendAreaSnakeLines(primaryHorizontal, minX, maxX, minY, maxY, step);

  if (!ok || g_routeCount < 2) {
    g_routeCount = 0;
    g_routeIndex = 0;
    memset(g_routeReceived, 0, sizeof(g_routeReceived));
    addRoutePoint(g_est.pos);
    ok = appendAreaSnakeLines(!primaryHorizontal, minX, maxX, minY, maxY, step);
  }

  if (!ok || g_routeCount < 2) {
    g_routeCount = 0;
    memset(g_routeReceived, 0, sizeof(g_routeReceived));
    return false;
  }

  g_navState = STATE_IDLE;
  g_navReason = "route_planned";
  Serial.printf("PLAN: area %u pts -> route %u pts\n", g_areaCount, g_routeCount);
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

  // Validate loaded offset
  if (fabsf(g_imuCalibrationOffset) > 180.0f) {
    Serial.printf("NAV PREFS: WARNING - saved IMU offset %.1f exceeds bounds, resetting\n", g_imuCalibrationOffset);
    g_imuCalibrationOffset = 0.0f;
    g_cal.verified = false;
  } else if (fabsf(g_imuCalibrationOffset) > 90.0f) {
    Serial.printf("NAV PREFS: WARNING - saved IMU offset %.1f is large, recalibration recommended\n", g_imuCalibrationOffset);
  }

  g_invertYaw = true;
  g_invertForward = g_prefs.getBool("invFwd", false);
  g_invertSteering = g_prefs.getBool("invSteer", false);
  g_navForwardScale = g_prefs.getFloat("fwdScale", 1.0f);
  g_navTurnScale = g_prefs.getFloat("turnScale", 1.0f);
}

static void calibrateImuToHeading(float desiredHeading) {
  desiredHeading = normalizeAngle360(desiredHeading);
  g_imuCalibrationOffset = normalizeAngle(desiredHeading - g_imuRawYaw);

  // Validate offset is within reasonable bounds
  if (fabsf(g_imuCalibrationOffset) > 180.0f) {
    Serial.printf("CAL_IMU: WARNING - offset %.1f exceeds bounds, clamping\n", g_imuCalibrationOffset);
    g_imuCalibrationOffset = normalizeAngle(g_imuCalibrationOffset);
  }

  // Warn if offset is suspiciously large (possible bad calibration)
  if (fabsf(g_imuCalibrationOffset) > 90.0f) {
    Serial.printf("CAL_IMU: WARNING - large offset %.1f degrees, verify IMU mounting\n", g_imuCalibrationOffset);
  }

  g_imuYaw = normalizeAngle360(g_imuRawYaw + g_imuCalibrationOffset);
  g_est.heading = g_imuYaw;
  g_cal.offset = g_imuCalibrationOffset;
  g_cal.verified = true;
  saveNavPrefs();
  Serial.printf("CAL_IMU: offset=%.2f heading=%.1f\n", g_imuCalibrationOffset, g_est.heading);
}

// ============== WEBSOCKET ==============

static AsyncWebServer server(WS_PORT);
static AsyncWebSocket ws("/ws");
static bool g_wifiConnected = false;
static bool g_serverStarted = false;
static uint32_t g_lastWifiReconnectMs = 0;
static uint32_t g_wifiReconnectIntervalMs = WIFI_RECONNECT_INITIAL_MS;

static const char* resetReasonString(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON: return "poweron";
    case ESP_RST_EXT: return "external";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "interrupt_wdt";
    case ESP_RST_TASK_WDT: return "task_wdt";
    case ESP_RST_WDT: return "other_wdt";
    case ESP_RST_DEEPSLEEP: return "deepsleep";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_SDIO: return "sdio";
    default: return "unknown";
  }
}

static void setAttachmentRelay(bool on) {
  digitalWrite(PIN_RELAY_ATTACHMENT, on ? HIGH : LOW);
  Serial.printf("ATTACHMENT %s\n", on ? "ON" : "OFF");
}

static void setMountRelay(bool on) {
  digitalWrite(PIN_RELAY_MOUNT, on ? HIGH : LOW);
  Serial.printf("MOUNT %s\n", on ? "ON" : "OFF");
}

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
    resetMovementMonitor();
    client->text("OK FREE");
    return;
  }

  if (cmd.startsWith("M,")) {
    int comma1 = cmd.indexOf(',', 2);
    if (comma1 > 2) {
      int left = cmd.substring(2, comma1).toInt();
      int right = cmd.substring(comma1 + 1).toInt();
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
    resetMovementMonitor();
    client->text("OK");
    return;
  }

  if (cmd == "ATTACHMENT_ON") {
    setAttachmentRelay(true);
    client->text("OK");
    return;
  }
  if (cmd == "ATTACHMENT_OFF") {
    setAttachmentRelay(false);
    client->text("OK");
    return;
  }
  if (cmd == "MOUNT_ON") {
    setMountRelay(true);
    client->text("OK");
    return;
  }
  if (cmd == "MOUNT_OFF") {
    setMountRelay(false);
    client->text("OK");
    return;
  }

  if (cmd == "CAL_IMU") {
    // Manual calibration command
    if (imuReadyForCalibration()) {
      startImuCalibration();
      client->text("OK,CAL_STARTED");
    } else {
      client->text("ERR,CAL_NOT_READY");
    }
    return;
  }

  if (cmd == "START" || cmd == "NAV_START") {
    if (routeReady() && g_routeIndex < g_routeCount) {
      // Seed heading from bearing to first target so pure-pursuit starts
      // heading in the right direction. GPS velocity heading is only updated
      // while moving, so without this seed the robot would try to drive at
      // a wrong initial heading and arc around.
      if (g_routeCount > 0 && g_est.lastUpdateMs > 0) {
        const LocalCoords& t = g_route[g_routeIndex].pos;
        float bearing = atan2f(t.x - g_est.pos.x, t.y - g_est.pos.y) * 180.0f / PI;
        if (bearing < 0) bearing += 360.0f;
        g_est.heading = bearing;
        // Reset the GPS-velocity heading EMA filter so the first GPS heading
        // sample after start is taken verbatim and not blended with the old
        // (pre-start) heading. Otherwise the controller would see a few
        // intermediate values between bearing and the actual motion heading.
        g_headingFilterSeeded = false;
        g_imuCalibrationOffset = normalizeAngle(bearing - g_imuRawYaw);
        Serial.printf("NAV: heading seeded to %.1f (toward first WP)\n", bearing);
      }
      g_navState = STATE_MOVING;
      g_navReason = "started";
      g_manualActive = false;
      markSegmentStart();
      g_offRouteSinceMs = 0;
      resetMovementMonitor();
      resetArrivalDetector();
      client->text("OK");
      Serial.printf("NAV: Started, route %u/%u\n", g_routeIndex, g_routeCount);
    } else {
      client->text("ERR,NO_ROUTE");
    }
    return;
  }

  if (cmd.startsWith("FORBID_BEGIN,")) {
    const int count = cmd.substring(13).toInt();
    if (count >= 0 && count <= MAX_FORBIDDEN_POLYGONS) {
      clearForbiddenZones();
      g_forbiddenCount = (uint8_t)count;
      g_forbiddenReady = (count == 0);
      client->text("OK");
      Serial.printf("WS: forbidden begin %d polygons\n", count);
      return;
    }
    client->text("ERR,FORBID_BEGIN");
    return;
  }

  if (cmd.startsWith("FORBID_PT,")) {
    int first = 10;
    int p1 = cmd.indexOf(',', first);
    int p2 = cmd.indexOf(',', p1 + 1);
    int p3 = cmd.indexOf(',', p2 + 1);
    if (p1 > first && p2 > p1 && p3 > p2) {
      const int poly = cmd.substring(first, p1).toInt();
      const int idx = cmd.substring(p1 + 1, p2).toInt();
      const float x = cmd.substring(p2 + 1, p3).toFloat();
      const float y = cmd.substring(p3 + 1).toFloat();
      if (poly >= 0 && poly < g_forbiddenCount &&
          idx >= 0 && idx < MAX_FORBIDDEN_POINTS &&
          isfinite(x) && isfinite(y)) {
        g_forbidden[poly][idx] = {x, y};
        g_forbiddenReceived[poly][idx] = true;
        if ((uint8_t)(idx + 1) > g_forbiddenPointCount[poly]) {
          g_forbiddenPointCount[poly] = (uint8_t)(idx + 1);
        }
        client->text("OK");
        return;
      }
    }
    client->text("ERR,FORBID_PT");
    return;
  }

  if (cmd == "FORBID_END") {
    g_forbiddenReady = forbiddenReady();
    if (g_forbiddenReady) {
      client->text("OK");
      Serial.printf("WS: forbidden ready %u polygons\n", g_forbiddenCount);
    } else {
      client->text("ERR,FORBID_INCOMPLETE");
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
        Serial.printf("WS: route begin %u points\n", count);
        return;
      }
    }
    client->text("ERR,INVALID");
    return;
  }

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
      if (count >= 3 && count <= MAX_AREA_POINTS && originLat != 0 && originLon != 0) {
        g_areaCount = (uint8_t)count;
        g_areaLineStep = lineStep;
        g_areaReady = false;
        memset(g_areaReceived, 0, sizeof(g_areaReceived));
        g_routeCount = 0;
        memset(g_routeReceived, 0, sizeof(g_routeReceived));
        g_navState = STATE_IDLE;
        setOrigin(originLat, originLon);
        client->text("OK");
        return;
      }
    }
    client->text("ERR,AREA_BEGIN");
    return;
  }

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

  if (cmd.startsWith("GO_TO,")) {
    int comma = cmd.indexOf(',', 6);
    if (comma > 6) {
      double lat = cmd.substring(6, comma).toDouble();
      double lon = cmd.substring(comma + 1).toDouble();
      if (lat != 0 && lon != 0) {
        if (!g_origin.valid && g_gps.valid) {
          setOrigin(g_gps.lat, g_gps.lon);
        }
        if (!g_origin.valid) {
          client->text("ERR,NO_ORIGIN");
          return;
        }
        LocalCoords target = toLocal(lat, lon);
        g_routeCount = 1;
        g_routeIndex = 0;
        memset(g_routeReceived, 0, sizeof(g_routeReceived));
        g_routeReceived[0] = true;
        g_route[0].pos = target;
        g_navState = STATE_MOVING;
        g_navReason = "go_to";
        g_manualActive = false;
        resetMovementMonitor();
        resetArrivalDetector();
        client->text("OK");
        Serial.printf("GO_TO: (%.2f, %.2f) m\n", target.x, target.y);
        return;
      }
    }
    client->text("ERR,GO_TO");
    return;
  }

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

  if (cmd.startsWith("CAL_IMU,")) {
    float desiredHeading = cmd.substring(8).toFloat();
    if (imuReadyForCalibration() || g_imuFresh) {
      calibrateImuToHeading(desiredHeading);
      char resp[64];
      snprintf(resp, sizeof(resp), "CAL_OK,heading=%.1f,offset=%.2f",
        g_est.heading, g_imuCalibrationOffset);
      client->text(resp);
    } else {
      client->text("ERR,CAL_NOT_READY");
    }
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
      g_est.heading = normalizeAngle360(g_imuRawYaw + g_imuCalibrationOffset);
      saveNavPrefs();
      client->text("OK");
      Serial.printf("NAV_CFG: fwd=%.2f turn=%.2f offset=%.2f\n",
        g_navForwardScale, g_navTurnScale, g_imuCalibrationOffset);
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

  char msg[512];
  const uint32_t gpsAge = g_gps.lastMs ? now - g_gps.lastMs : 99999;
  const uint32_t rtcmTransportAge = g_lastRtcmMs ? now - g_lastRtcmMs : 99999;
  const uint32_t rtcmF9pAge = g_lastF9pRtcmMs ? now - g_lastF9pRtcmMs : 99999;
  const uint32_t imuAge = g_lastImuMs ? now - g_lastImuMs : 99999;
  const char* carrier = g_gps.carrier == 2 ? "fixed" : (g_gps.carrier == 1 ? "float" : "none");

  // Only send telemetry if WiFi is connected
  if (!g_wifiConnected) return;

  // Main telemetry
  snprintf(msg, sizeof(msg),
    "TEL,%.8f,%.8f,%.2f,%.1f,%u,%s,%u,%u,%.0f,%.0f,%.3f,%.2f,%lu,%lu,%lu,%.1f,%lu,%u,%lu,%lu,%s,%lu,%lu,%lu,%s,%.3f,%.3f,%.3f,%s,%s",
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
    (unsigned long)g_rtcmLastType,
    g_est.headingSource == 0 ? "IMU" : (g_est.headingSource == 1 ? "GPS" : "BLEND"),
    g_crossTrackError,
    g_distToNextWaypoint,
    g_distToRouteEnd,
    stateString(g_navState),
    getMovementStatus()
  );
  ws.textAll(msg);

  // GPS Debug
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
    "RTCM,%lu,%lu,%lu,%lu,%s,%lu,%lu,%lu,%lu,%lu,%lu,%lu",
    (unsigned long)g_rtcmBytes,
    (unsigned long)rtcmTransportAge,
    (unsigned long)rtcmTransportAge,
    (unsigned long)rtcmF9pAge,
    g_rtcmFresh ? "udp" : "none",
    (unsigned long)g_f9pRtcmMsgs,
    (unsigned long)g_f9pRtcmCrcFail,
    (unsigned long)g_rtcmLastType,
    (unsigned long)g_rtcmFramesToF9p,
    (unsigned long)g_rtcmCrcFail,
    (unsigned long)g_rtcmWriteErrors,
    (unsigned long)g_rtcmFrameOverflow
  );
  ws.textAll(msg);

  // IMU status
  snprintf(msg, sizeof(msg),
    "IMU,%.1f,%lu,%u,%.1f,%.2f,%s",
    g_imuYaw,
    (unsigned long)imuAge,
    g_imuFresh ? 1 : 0,
    g_imuYawRate,
    g_imuCalibrationOffset,
    calStateString(g_cal.state)
  );
  ws.textAll(msg);

  // Navigation
  snprintf(msg, sizeof(msg),
    "NAV,%s,%u,%u,%.2f,%.1f,%.3f,%.3f,%.1f,%s",
    stateString(g_navState),
    g_routeIndex,
    g_routeCount,
    g_distToRouteEnd,
    g_headingError,
    g_est.speed,
    g_mon.progressRate,
    g_crossTrackError,
    getMovementStatus()
  );
  ws.textAll(msg);

  // Motor
  snprintf(msg, sizeof(msg),
    "MOTOR,%d,%d,%u,%d,%d,%d,%d",
    g_curLeft, g_curRight,
    g_haveMotorFeedback ? 1 : 0,
    g_motorSpeedL, g_motorSpeedR,
    g_motorBatVoltage, g_motorBoardTemp
  );
  ws.textAll(msg);

  // Calibration
  snprintf(msg, sizeof(msg),
    "CAL,%s,%.2f,%.2f,%u,%u",
    g_cal.state == CAL_COMPLETE ? "OK" : (g_cal.state == CAL_MEASURING ? "MEAS" :
        (g_cal.state == CAL_VERIFYING ? "VERIFY" : "IDLE")),
    g_cal.offset,
    g_cal.stdDev,
    g_cal.quality,
    g_cal.verified ? 1 : 0
  );
  ws.textAll(msg);
}

// ============== NAVIGATION UPDATE ==============

static void navUpdate() {
  uint32_t now = millis();

  // Handle recovery state
  if (g_recovery.recoveryActive) {
    updateRecovery();
    if (!g_recovery.recoveryActive) {
      g_navState = STATE_MOVING;
    }
    return;
  }

  // Handle calibration
  if (g_cal.state == CAL_MEASURING || g_cal.state == CAL_VERIFYING) {
    updateCalibration();
  }

  if (g_navState != STATE_MOVING && g_navState != STATE_APPROACHING) {
    if (g_manualActive) return;
    return;
  }

  if (g_routeIndex >= g_routeCount) {
    g_navState = STATE_ARRIVED;
    g_navReason = "route_complete";
    g_targetLeft = g_targetRight = 0;
    char navMsg[64];
    snprintf(navMsg, sizeof(navMsg), "NAV,ARRIVED,%u,%u,0.00", g_routeIndex, g_routeCount);
    ws.textAll(navMsg);
    resetMovementMonitor();
    return;
  }

  // Check quality. Autonomous point-to-point navigation needs RTK precision.
  // A short hold is allowed only after a recent RTK fix, so a small bump or
  // brief correction dropout does not immediately abort the run.
  const bool preciseFix =
    g_est.quality == QUAL_RTK_FIXED_GOOD ||
    g_est.quality == QUAL_RTK_FLOAT_OK;
  const bool shortRtkHold =
    g_est.quality == QUAL_GPS_HOLD_SHORT &&
    g_haveRtkFix &&
    (now - g_lastGoodRtkMs) <= GPS_HOLD_AGE_MS;
  if (!g_simulationMode && !preciseFix && !shortRtkHold) {
    g_targetLeft = g_targetRight = 0;
    g_navReason = "no_rtk_fix";
    triggerRecovery("no_rtk_fix");
    return;
  }

  bool insideForbidden = false;
  g_forbiddenClearanceM = forbiddenClearance(g_est.pos, &insideForbidden);
  if (g_forbiddenReady && g_forbiddenCount > 0 &&
      (insideForbidden || g_forbiddenClearanceM <= FORBIDDEN_STOP_CLEARANCE_M)) {
    g_targetLeft = g_targetRight = 0;
    g_navState = STATE_ERROR;
    g_navReason = "forbidden_stop";
    char navMsg[96];
    snprintf(navMsg, sizeof(navMsg), "NAV,FORBIDDEN_STOP,%u,%u,%.2f",
      g_routeIndex, g_routeCount, g_forbiddenClearanceM);
    ws.textAll(navMsg);
    Serial.printf("NAV: Forbidden stop, clearance=%.2f m\n", g_forbiddenClearanceM);
    resetMovementMonitor();
    return;
  }

  // Get current waypoint
  LocalCoords target = g_route[g_routeIndex].pos;
  float distToTarget = hypotf(target.x - g_est.pos.x, target.y - g_est.pos.y);
  g_distToNextWaypoint = distToTarget;

  // Calculate remaining route distance
  g_distToRouteEnd = distToTarget;
  for (uint8_t i = g_routeIndex + 1; i < g_routeCount; i++) {
    float segLen = hypotf(g_route[i].pos.x - g_route[i-1].pos.x,
                          g_route[i].pos.y - g_route[i-1].pos.y);
    g_distToRouteEnd += segLen;
  }

  // Intermediate waypoints are segment handoffs, not final parking points.
  // Advance them without requiring a full stop; keep strict confirmation only
  // for the final waypoint.
  const bool finalWaypoint = (g_routeIndex + 1) >= g_routeCount;
  float intermediateAdvanceM = ARRIVAL_DIST_M * 3.0f;
  if (g_routeIndex > 0) {
    const LocalCoords prev = g_route[g_routeIndex - 1].pos;
    const float segLen = hypotf(target.x - prev.x, target.y - prev.y);
    // Intermediate points are handoff points. Long mowing rows switch early;
    // short connectors keep a tighter radius so turns do not get skipped.
    intermediateAdvanceM = constrain(
      segLen * 0.08f,
      ARRIVAL_DIST_M * 3.0f,
      INTERMEDIATE_ADVANCE_MAX_M
    );
  }

  const bool intermediateReached =
    !finalWaypoint && distToTarget < intermediateAdvanceM;
  // For the final waypoint: do not require the robot to drop below HOLD_SPEED
  // (HOLD_SPEED=0.025 > arrivalSpeedMps=0.03? close, but checkArrival demands
  // 0.03 m/s and RTK drit 14mm makes the 5cm radius oscillate). Use a wider
  // final-approach radius and accept arrival when we are inside it.
  const bool finalArrived = finalWaypoint && distToTarget < 0.40f;

  if (intermediateReached || finalArrived || checkArrival(target, 0, g_headingError)) {
    g_routeIndex++;
    char navMsg[64];
    snprintf(navMsg, sizeof(navMsg), "NAV,WP_REACHED,%u,%u,%.2f",
      g_routeIndex, g_routeCount, distToTarget);
    ws.textAll(navMsg);
    Serial.printf("NAV: Waypoint %u reached (dist=%.2f)\n", g_routeIndex - 1, distToTarget);
    resetMovementMonitor();
    resetArrivalDetector();

    if (g_routeIndex >= g_routeCount) {
      g_navState = STATE_ARRIVED;
      g_navReason = "route_complete";
      g_targetLeft = g_targetRight = 0;
    } else {
      g_navState = STATE_MOVING;
      g_navReason = "next_waypoint";
      markSegmentStart();
      g_offRouteSinceMs = 0;
    }
    return;
  }

  // Update state based on distance
  if (distToTarget < ARRIVAL_APPROACH_DIST_M) {
    g_navState = STATE_APPROACHING;
  }

  syncNavCoreRoute();
  g_navCore.setConfig(makeNavCoreConfig());

  navcore::NavInput navInput;
  navInput.position = navcore::LocalCoords(g_est.pos.x, g_est.pos.y);
  navInput.headingDeg = g_est.heading;
  navInput.speedMps = g_est.speed;
  navInput.quality = toCoreQuality(g_est.quality);
  navInput.goingWrong = g_mon.isGoingWrong;

  navcore::NavOutput navOutput = g_navCore.update(navInput, NAV_LOOP_MS / 1000.0f);
  // Apply heading dead zone: small bearing jitter (RTK drit + GPS-velocity
  // noise) sits inside ±HEADING_DEAD_ZONE_DEG and would otherwise produce
  // an oscillating turnCmd that wiggles the robot in place.
  float rawHeadingError = navOutput.headingErrorDeg;
  if (!g_simulationMode && fabsf(rawHeadingError) < HEADING_DEAD_ZONE_DEG) {
    navOutput.headingErrorDeg = 0.0f;
  }
  g_headingError = navOutput.headingErrorDeg;
  g_crossTrackError = navOutput.crossTrackErrorM;
  float desiredHeading = navOutput.desiredHeadingDeg;
  float speed = navOutput.commandSpeedMps;

  if (navOutput.state == navcore::NavState::Approaching) {
    g_navState = STATE_APPROACHING;
  } else {
    g_navState = STATE_MOVING;
  }

  if (navOutput.alignOnly) {
    resetMovementMonitor();
  }

  // Update movement monitor
  updateMovementMonitor(target, speed, desiredHeading);

  // Check for stuck condition - but allow for active turning
  // If heading error is large, robot is legitimately turning in place
  const bool isActivelyTurning = fabsf(g_headingError) > 60.0f || navOutput.alignOnly;
  if (!g_simulationMode && g_mon.isStuck && speed > STUCK_SPEED_THRESHOLD_MPS && g_est.speed < STUCK_SPEED_THRESHOLD_MPS && !isActivelyTurning) {
    Serial.println("NAV: Stuck detected - triggering recovery");
    triggerRecovery("stuck");
    return;
  }

  // Check for wrong direction
  if (g_mon.isGoingWrong && g_mon.actualSpeed > 0.1f) {
    Serial.printf("NAV: Going wrong direction! progress=%.3f m/s\n", g_mon.progressRate);
    navInput.goingWrong = true;
    navOutput = g_navCore.update(navInput, NAV_LOOP_MS / 1000.0f);
    speed = navOutput.commandSpeedMps;
    g_headingError = navOutput.headingErrorDeg;
    g_crossTrackError = navOutput.crossTrackErrorM;
  }

  if (!navOutput.alignOnly && fabsf(g_crossTrackError) > CROSS_TRACK_STOP_M) {
    if (g_offRouteSinceMs == 0) g_offRouteSinceMs = now;
    if (!g_simulationMode && (now - g_offRouteSinceMs) >= CROSS_TRACK_STOP_CONFIRM_MS) {
      g_targetLeft = g_targetRight = 0;
      g_navState = STATE_ERROR;
      g_navReason = "off_route_stop";
      char navMsg[96];
      snprintf(navMsg, sizeof(navMsg), "NAV,OFF_ROUTE_STOP,%u,%u,%.2f",
        g_routeIndex, g_routeCount, g_crossTrackError);
      ws.textAll(navMsg);
      Serial.printf("NAV: Off-route stop, cross-track=%.2f m\n", g_crossTrackError);
      resetMovementMonitor();
      return;
    }
  } else {
    g_offRouteSinceMs = 0;
  }

  float safetyScale = 1.0f;
  if (g_forbiddenReady && g_forbiddenCount > 0 &&
      g_forbiddenClearanceM <= FORBIDDEN_SLOW_CLEARANCE_M) {
    safetyScale = min(safetyScale, 0.45f);
  }
  if (!navOutput.alignOnly && fabsf(g_crossTrackError) > CROSS_TRACK_SLOW_M) {
    safetyScale = min(safetyScale, 0.55f);
  }
  if (g_routeIndex != g_segmentStartIndex) {
    markSegmentStart();
  }
  const bool segmentEntrySlow =
    (now - g_segmentStartMs) < SEGMENT_ENTRY_SLOW_MS ||
    currentSegmentProgressM() < SEGMENT_ENTRY_SLOW_M;
  if (!navOutput.alignOnly && segmentEntrySlow) {
    safetyScale = min(safetyScale, 0.55f);
  }

  const int16_t safeLeft = (int16_t)lroundf(navOutput.leftCmd * safetyScale);
  const int16_t safeRight = (int16_t)lroundf(navOutput.rightCmd * safetyScale);
  g_targetLeft = constrain(safeLeft, -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);
  g_targetRight = constrain(safeRight, -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);
  g_lastCmdMs = now;
  g_isFailSafeStopping = false;
  g_navReason = qualityString(g_est.quality);

  // Debug output
  static uint32_t lastNavDebug = 0;
  if (now - lastNavDebug > 2000) {
    lastNavDebug = now;
    Serial.printf("NAV: pos=(%.2f,%.2f) target=(%.2f,%.2f) dist=%.2f heading=%.1f err=%.1f xtk=%.2f progress=%.3f speed=%d L=%d R=%d\n",
      g_est.pos.x, g_est.pos.y,
      target.x, target.y,
      distToTarget,
      g_est.heading,
      g_headingError,
      g_crossTrackError,
      g_mon.progressRate,
      (int)speed * 100,
      g_targetLeft, g_targetRight);
  }
}

// ============== MOTOR CONTROL ==============

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

// ============== USB HITL SIMULATION ==============

static NavQuality simQualityFromToken(String token) {
  token.trim();
  token.toUpperCase();
  if (token == "FIXED" || token == "RTK_FIXED" || token == "4") return QUAL_RTK_FIXED_GOOD;
  if (token == "FLOAT" || token == "RTK_FLOAT" || token == "5") return QUAL_RTK_FLOAT_OK;
  if (token == "DEGRADED" || token == "DGPS" || token == "2") return QUAL_GPS_DEGRADED;
  if (token == "HOLD" || token == "GPS_HOLD") return QUAL_GPS_HOLD_SHORT;
  return QUAL_NAV_ERROR;
}

static void startSimulationMode(float targetX, float targetY) {
  const uint32_t now = millis();

  g_simulationMode = true;
  g_simTick = 0;
  g_simSerialLen = 0;

  g_routeCount = 1;
  g_routeIndex = 0;
  memset(g_routeReceived, 0, sizeof(g_routeReceived));
  g_route[0].pos = {targetX, targetY};
  g_routeReceived[0] = true;

  g_navState = STATE_MOVING;
  g_navReason = "simulation";
  g_manualActive = false;
  g_recovery.recoveryActive = false;
  g_targetLeft = g_targetRight = 0;
  g_curLeft = g_curRight = 0;
  g_cmdSpeed = g_cmdSteer = 0;
  g_lastCmdMs = now;
  g_lastRampMs = now;
  g_isFailSafeStopping = false;

  g_est.pos = {0, 0};
  g_est.vel = {0, 0};
  g_est.heading = 0;
  g_est.headingSource = 0;
  g_est.speed = 0;
  g_est.lastUpdateMs = now;
  g_est.quality = QUAL_RTK_FIXED_GOOD;
  g_est.qualityAgeMs = 0;
  g_est.rtkFixed = true;

  g_gps.valid = true;
  g_gps.fixType = 3;
  g_gps.carrier = 2;
  g_gps.diff = true;
  g_gps.hAcc = 15.0f;
  g_gps.vAcc = 25.0f;
  g_gps.speed = 0;
  g_gps.velN = 0;
  g_gps.velE = 0;
  g_gps.lastMs = now;
  g_lastGoodRtkMs = now;
  g_lastF9pRtcmMs = now;
  g_haveRtkFix = true;

  g_imuRawYaw = 0;
  g_imuYaw = 0;
  g_imuYawRate = 0;
  g_lastImuMs = now;
  g_imuFresh = true;

  resetGpsFilter();
  resetMovementMonitor();
  resetArrivalDetector();
  syncNavCoreRoute();
  g_navCore.setConfig(makeNavCoreConfig());
  g_navCore.reset();

  Serial.printf("SIM_OK,%.3f,%.3f\n", targetX, targetY);
}

static void restartSimulationWithCurrentRoute(float ackX, float ackY) {
  const uint32_t now = millis();

  g_simulationMode = true;
  g_simTick = 0;
  g_simSerialLen = 0;
  g_routeIndex = 0;  // Start from first waypoint

  g_navState = STATE_MOVING;
  g_navReason = "simulation_route";
  g_manualActive = false;
  g_recovery.recoveryActive = false;
  g_targetLeft = g_targetRight = 0;
  g_curLeft = g_curRight = 0;
  g_cmdSpeed = g_cmdSteer = 0;
  g_lastCmdMs = now;
  g_lastRampMs = now;
  g_isFailSafeStopping = false;

  g_est.pos = {0, 0};
  g_est.vel = {0, 0};
  g_est.heading = 0;
  g_est.headingSource = 0;
  g_est.speed = 0;
  g_est.lastUpdateMs = now;
  g_est.quality = QUAL_RTK_FIXED_GOOD;
  g_est.qualityAgeMs = 0;
  g_est.rtkFixed = true;

  g_gps.valid = true;
  g_gps.fixType = 3;
  g_gps.carrier = 2;
  g_gps.diff = true;
  g_gps.hAcc = 15.0f;
  g_gps.vAcc = 25.0f;
  g_gps.speed = 0;
  g_gps.velN = 0;
  g_gps.velE = 0;
  g_gps.lastMs = now;
  g_lastGoodRtkMs = now;
  g_lastF9pRtcmMs = now;
  g_haveRtkFix = true;

  g_imuRawYaw = 0;
  g_imuYaw = 0;
  g_imuYawRate = 0;
  g_lastImuMs = now;
  g_imuFresh = true;

  resetGpsFilter();
  resetMovementMonitor();
  resetArrivalDetector();
  syncNavCoreRoute();
  g_navCore.setConfig(makeNavCoreConfig());
  g_navCore.reset();

  Serial.printf("SIM_ROUTE_OK,%u,%u,%.3f,%.3f\n", g_routeCount, g_routeIndex, ackX, ackY);
}

static bool startSimulationRoute(String spec) {
  spec.trim();
  if (spec.length() == 0) return false;

  g_routeCount = 0;
  g_routeIndex = 0;
  memset(g_routeReceived, 0, sizeof(g_routeReceived));

  float lastX = 0.0f;
  float lastY = 0.0f;
  int tokenStart = 0;
  while (tokenStart < spec.length() && g_routeCount < MAX_WAYPOINTS) {
    int tokenEnd = spec.indexOf(';', tokenStart);
    if (tokenEnd < 0) tokenEnd = spec.length();

    String token = spec.substring(tokenStart, tokenEnd);
    token.trim();
    const int comma = token.indexOf(',');
    if (comma <= 0) return false;

    const float x = token.substring(0, comma).toFloat();
    const float y = token.substring(comma + 1).toFloat();
    g_route[g_routeCount].pos = {x, y};
    g_routeReceived[g_routeCount] = true;
    g_routeCount++;
    lastX = x;
    lastY = y;

    tokenStart = tokenEnd + 1;
  }

  if (g_routeCount < 2) return false;
  restartSimulationWithCurrentRoute(lastX, lastY);
  return true;
}

static bool beginSimulationRouteUpload(uint8_t count) {
  if (count < 2 || count > MAX_WAYPOINTS) return false;
  g_routeCount = count;
  g_routeIndex = 0;
  memset(g_routeReceived, 0, sizeof(g_routeReceived));
  g_simulationMode = false;
  Serial.printf("SIM_ROUTE_BEGIN_OK,%u\n", g_routeCount);
  return true;
}

static bool setSimulationRouteWaypoint(String spec) {
  spec.trim();
  const int comma1 = spec.indexOf(',');
  const int comma2 = spec.indexOf(',', comma1 + 1);
  if (comma1 <= 0 || comma2 <= comma1) return false;

  const int idx = spec.substring(0, comma1).toInt();
  if (idx < 0 || idx >= g_routeCount) return false;

  const float x = spec.substring(comma1 + 1, comma2).toFloat();
  const float y = spec.substring(comma2 + 1).toFloat();
  g_route[idx].pos = {x, y};
  g_routeReceived[idx] = true;
  return true;
}

static bool finishSimulationRouteUpload() {
  if (!routeReady()) return false;
  const LocalCoords last = g_route[g_routeCount - 1].pos;
  restartSimulationWithCurrentRoute(last.x, last.y);
  return true;
}

static void applySimulationGps(float x, float y, float headingDeg, float speedMps, NavQuality quality) {
  const uint32_t now = millis();
  const float headingRad = degToRad(headingDeg);

  // Apply GPS filtering to reduce noise (same as in updateEstimator)
  LocalCoords rawPos = {x, y};
  updateGpsFilter(rawPos, quality == QUAL_RTK_FIXED_GOOD ? 15.0f : 250.0f);

  g_est.pos = g_gpsFilter.filteredPos;
  g_est.vel = {speedMps * sinf(headingRad), speedMps * cosf(headingRad)};
  g_est.heading = normalizeAngle360(headingDeg);
  g_est.headingSource = 0;
  g_est.speed = fabsf(speedMps);
  g_est.lastUpdateMs = now;
  g_est.quality = quality;
  g_est.qualityAgeMs = 0;
  g_est.rtkFixed = quality == QUAL_RTK_FIXED_GOOD;

  g_gps.valid = quality != QUAL_NAV_ERROR;
  g_gps.fixType = g_gps.valid ? 3 : 0;
  g_gps.carrier = quality == QUAL_RTK_FIXED_GOOD ? 2 : (quality == QUAL_RTK_FLOAT_OK ? 1 : 0);
  g_gps.diff = quality == QUAL_RTK_FIXED_GOOD || quality == QUAL_RTK_FLOAT_OK;
  g_gps.hAcc = quality == QUAL_RTK_FIXED_GOOD ? 15.0f : 250.0f;
  g_gps.vAcc = 25.0f;
  g_gps.speed = fabsf(speedMps);
  g_gps.heading = normalizeAngle360(headingDeg);
  g_gps.velE = g_est.vel.x;
  g_gps.velN = g_est.vel.y;
  g_gps.lastMs = now;

  g_imuRawYaw = normalizeAngle360(headingDeg);
  g_imuYaw = g_imuRawYaw;
  g_lastImuMs = now;
  g_imuFresh = true;

  if (quality == QUAL_RTK_FIXED_GOOD || quality == QUAL_RTK_FLOAT_OK) {
    g_lastGoodRtkMs = now;
    g_lastF9pRtcmMs = now;
    g_haveRtkFix = true;
  }
}

static void runSimulationNavigationTick() {
  navUpdate();
  // HITL can run faster than wall-clock time. The normal motor ramp is based
  // on millis(), so using it here creates false "stuck" detections when the
  // PC advances simulated physics faster than real time. TrackPhysics already
  // models command delay and motor inertia, so expose the navigation target
  // command directly in simulation mode.
  g_curLeft = g_targetLeft;
  g_curRight = g_targetRight;
  if (g_simVerbose) {
    Serial.printf("SIM_DBG,%u,%u,%d,%.3f,%.1f,%.3f,%d,%d,%d,%d\n",
      g_routeIndex,
      g_routeCount,
      (int)g_navState,
      g_distToNextWaypoint,
      g_headingError,
      g_crossTrackError,
      (int)g_targetLeft,
      (int)g_targetRight,
      (int)g_curLeft,
      (int)g_curRight);
  }
  Serial.printf("MOTORS,%d,%d\n", (int)g_curLeft, (int)g_curRight);
  g_simTick++;
}

static void handleSimulationLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line.startsWith("SIM_START")) {
    float targetX = 5.0f;
    float targetY = 0.0f;
    const int comma1 = line.indexOf(',');
    if (comma1 > 0) {
      const int comma2 = line.indexOf(',', comma1 + 1);
      if (comma2 > comma1) {
        targetX = line.substring(comma1 + 1, comma2).toFloat();
        targetY = line.substring(comma2 + 1).toFloat();
      }
    }
    startSimulationMode(targetX, targetY);
    return;
  }

  if (line.startsWith("SIM_ROUTE,")) {
    if (!startSimulationRoute(line.substring(10))) {
      Serial.println("SIM_ERR,ROUTE_FORMAT");
    }
    return;
  }

  if (line.startsWith("SIM_ROUTE_BEGIN,")) {
    const int count = line.substring(16).toInt();
    if (!beginSimulationRouteUpload((uint8_t)count)) {
      Serial.println("SIM_ERR,ROUTE_BEGIN");
    }
    return;
  }

  if (line.startsWith("SIM_ROUTE_WP,")) {
    if (!setSimulationRouteWaypoint(line.substring(13))) {
      Serial.println("SIM_ERR,ROUTE_WP");
    }
    return;
  }

  if (line == "SIM_ROUTE_END") {
    if (!finishSimulationRouteUpload()) {
      Serial.println("SIM_ERR,ROUTE_INCOMPLETE");
    }
    return;
  }

  // SIM_FORBID,count - clear old forbidden and set polygon count
  if (line.startsWith("SIM_FORBID,")) {
    const int count = line.substring(11).toInt();
    if (count >= 0 && count <= MAX_FORBIDDEN_POLYGONS) {
      clearForbiddenZones();
      g_forbiddenCount = (uint8_t)count;
      g_forbiddenReady = (count == 0);
      Serial.printf("SIM_FORBID_OK,%u\n", count);
      return;
    }
    Serial.println("SIM_ERR,FORBID_COUNT");
    return;
  }

  // SIM_FORBID_PT,polyIdx,ptIdx,x,y
  if (line.startsWith("SIM_FORBID_PT,")) {
    const char* p = line.c_str() + 14;  // skip "SIM_FORBID_PT,"
    int comma1 = -1, comma2 = -1, comma3 = -1;
    for (int i = 0; p[i]; i++) {
      if (p[i] == ',') {
        if (comma1 < 0) comma1 = i;
        else if (comma2 < 0) comma2 = i;
        else if (comma3 < 0) comma3 = i;
      }
    }
    if (comma1 > 0 && comma2 > comma1 && comma3 > comma2) {
      char buf[32];
      strncpy(buf, p, comma1); buf[comma1] = '\0';
      const int poly = atoi(buf);
      strncpy(buf, p + comma1 + 1, comma2 - comma1 - 1); buf[comma2 - comma1 - 1] = '\0';
      const int idx = atoi(buf);
      strncpy(buf, p + comma2 + 1, comma3 - comma2 - 1); buf[comma3 - comma2 - 1] = '\0';
      const float x = atof(buf);
      strncpy(buf, p + comma3 + 1, 31);  // y is last field
      const float y = atof(buf);
      if (poly >= 0 && poly < MAX_FORBIDDEN_POLYGONS &&
          idx >= 0 && idx < MAX_FORBIDDEN_POINTS &&
          isfinite(x) && isfinite(y)) {
        g_forbidden[poly][idx] = {x, y};
        g_forbiddenReceived[poly][idx] = true;
        if ((uint8_t)(idx + 1) > g_forbiddenPointCount[poly]) {
          g_forbiddenPointCount[poly] = (uint8_t)(idx + 1);
        }
        Serial.println("SIM_FORBID_PT_OK");
        return;
      }
    }
    Serial.println("SIM_ERR,FORBID_PT");
    return;
  }

  // SIM_FORBID_END - mark forbidden zones ready
  if (line == "SIM_FORBID_END") {
    g_forbiddenReady = forbiddenReady();
    Serial.printf("SIM_FORBID_END_OK,%u\n", g_forbiddenReady ? 1 : 0);
    return;
  }

  if (line == "SIM_STOP") {
    g_simulationMode = false;
    g_navState = STATE_IDLE;
    g_navReason = "simulation_stopped";
    g_targetLeft = g_targetRight = 0;
    g_curLeft = g_curRight = 0;
    Serial.println("SIM_STOPPED");
    return;
  }

  if (line.startsWith("SIM_VERBOSE,")) {
    g_simVerbose = line.substring(12).toInt() != 0;
    Serial.printf("SIM_VERBOSE,%u\n", g_simVerbose ? 1 : 0);
    return;
  }

  // SIM_SETPARAMS,k_heading,k_crosstrack - set navigation gains at runtime
  if (line.startsWith("SIM_SETPARAMS,")) {
    const int comma1 = line.indexOf(',', 14);
    if (comma1 > 14) {
      const float kh = line.substring(14, comma1).toFloat();
      const float kx = line.substring(comma1 + 1).toFloat();
      if (kh > 0 && kh <= 2.0 && kx > 0 && kx <= 50.0) {
        g_kHeading = kh;
        g_kCrossTrack = kx;
        g_navCore.config().headingGain = kh;
        g_navCore.config().crossTrackGain = kx;
        Serial.printf("SIM_PARAMS,k_heading=%.2f,k_crosstrack=%.1f\n", kh, kx);
        return;
      }
    }
    Serial.println("SIM_ERR,INVALID_PARAMS");
    return;
  }

  if (!g_simulationMode) return;

  if (line.startsWith("GPS,")) {
    const int p1 = line.indexOf(',', 4);
    const int p2 = line.indexOf(',', p1 + 1);
    const int p3 = line.indexOf(',', p2 + 1);
    const int p4 = line.indexOf(',', p3 + 1);
    if (p1 < 0 || p2 < 0 || p3 < 0 || p4 < 0) {
      Serial.println("SIM_ERR,GPS_FORMAT");
      return;
    }

    const float x = line.substring(4, p1).toFloat();
    const float y = line.substring(p1 + 1, p2).toFloat();
    const float heading = line.substring(p2 + 1, p3).toFloat();
    const float speed = line.substring(p3 + 1, p4).toFloat();
    const NavQuality quality = simQualityFromToken(line.substring(p4 + 1));

    applySimulationGps(x, y, heading, speed, quality);
    runSimulationNavigationTick();
    return;
  }

  Serial.println("SIM_ERR,UNKNOWN");
}

static void handleSimulationSerialInput() {
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      g_simSerialLine[g_simSerialLen] = '\0';
      handleSimulationLine(String(g_simSerialLine));
      g_simSerialLen = 0;
    } else if (g_simSerialLen + 1 < sizeof(g_simSerialLine)) {
      g_simSerialLine[g_simSerialLen++] = c;
    } else {
      g_simSerialLen = 0;
      Serial.println("SIM_ERR,LINE_TOO_LONG");
    }
  }
}

// ============== STATUS ==============

static void printStatus() {
  static uint32_t lastStatus = 0;
  uint32_t now = millis();
  if (now - lastStatus < STATUS_MS) return;
  lastStatus = now;

  Serial.println();
  Serial.println("========== ROVER STATUS v2.0 ==========");
  Serial.printf("Uptime: %lus\n", (unsigned long)(now / 1000));

  Serial.println("-- State --");
  Serial.printf("  State: %s, Reason: %s\n", stateString(g_navState), g_navReason);
  Serial.printf("  Recovery: %s (%lu attempts)\n",
    g_recovery.recoveryActive ? "ACTIVE" : "idle", (unsigned long)g_recovery.recoveryAttempts);

  Serial.println("-- WiFi --");
  Serial.printf("  %s\n", g_wifiConnected ? WiFi.localIP().toString().c_str() : "DISCONNECTED");
  if (g_wifiConnected) Serial.printf("  RSSI: %d dBm\n", WiFi.RSSI());

  Serial.println("-- GPS --");
  Serial.printf("  Fix: %u, Carrier: %u (%s)\n",
    g_gps.fixType, g_gps.carrier,
    g_gps.carrier == 2 ? "FIXED" : (g_gps.carrier == 1 ? "FLOAT" : "NONE"));
  Serial.printf("  Pos: %.8f, %.8f\n", g_gps.lat, g_gps.lon);
  Serial.printf("  hAcc: %.0f mm, vAcc: %.0f mm\n", g_gps.hAcc, g_gps.vAcc);
  Serial.printf("  Vel: N=%.2f E=%.2f m/s\n", g_gps.velN, g_gps.velE);

  Serial.println("-- IMU --");
  Serial.printf("  Raw yaw: %.1f deg, Heading: %.1f deg\n", g_imuRawYaw, g_imuYaw);
  Serial.printf("  Offset: %.2f deg, Yaw rate: %.1f deg/s\n", g_imuCalibrationOffset, g_imuYawRate);
  Serial.printf("  Fresh: %u, Source: %s\n", g_imuFresh ? 1 : 0,
    g_est.headingSource == 0 ? "IMU" : (g_est.headingSource == 1 ? "GPS" : "BLEND"));

  Serial.println("-- Calibration --");
  Serial.printf("  State: %s, StdDev: %.2f deg\n",
    g_cal.state == CAL_COMPLETE ? "OK" : (g_cal.state == CAL_MEASURING ? "MEASURING" :
        (g_cal.state == CAL_VERIFYING ? "VERIFYING" : "IDLE")), g_cal.stdDev);
  Serial.printf("  Verified: %u, Quality: %u%%\n", g_cal.verified ? 1 : 0, g_cal.quality);

  Serial.println("-- Movement Monitor --");
  Serial.printf("  Status: %s\n", getMovementStatus());
  Serial.printf("  Progress: %.3f m/s, Actual speed: %.3f m/s\n", g_mon.progressRate, g_mon.actualSpeed);
  Serial.printf("  Heading error: %.1f deg\n", g_mon.headingError);

  Serial.println("-- Position --");
  Serial.printf("  Local: (%.3f, %.3f) m\n", g_est.pos.x, g_est.pos.y);
  Serial.printf("  Heading: %.1f deg, Speed: %.3f m/s\n", g_est.heading, g_est.speed);
  Serial.printf("  Quality: %s\n", qualityString(g_est.quality));

  Serial.println("-- Navigation --");
  Serial.printf("  Route: %u/%u\n", g_routeIndex, g_routeCount);
  Serial.printf("  Dist to wp: %.2f m, to end: %.2f m\n", g_distToNextWaypoint, g_distToRouteEnd);
  Serial.printf("  Heading err: %.1f deg, Cross-track: %.2f m\n", g_headingError, g_crossTrackError);
  Serial.printf("  Motor: L=%d R=%d (target), L=%d R=%d (current)\n",
    g_targetLeft, g_targetRight, g_curLeft, g_curRight);

  Serial.println("-- RTCM --");
  Serial.printf("  Fresh: %u, Bytes: %lu, Msgs: %lu\n",
    g_rtcmFresh ? 1 : 0, (unsigned long)g_rtcmBytes, (unsigned long)g_rtcmMsgs);
  Serial.printf("  Frames to F9P: %lu, asmCrcFail: %lu, writeErr: %lu, overflow: %lu\n",
    (unsigned long)g_rtcmFramesToF9p,
    (unsigned long)g_rtcmCrcFail,
    (unsigned long)g_rtcmWriteErrors,
    (unsigned long)g_rtcmFrameOverflow);
  Serial.printf("  F9P decoded: %lu, crcFail: %lu\n",
    (unsigned long)g_f9pRtcmMsgs, (unsigned long)g_f9pRtcmCrcFail);
  Serial.printf("  UBX ACK: %lu, NAK: %lu\n",
    (unsigned long)g_ubxAck, (unsigned long)g_ubxNak);

  Serial.println("-- Motor Feedback --");
  Serial.printf("  Fresh: %u, Speed: L=%d R=%d\n", g_haveMotorFeedback ? 1 : 0, g_motorSpeedL, g_motorSpeedR);
  Serial.printf("  Bat: %d mV, Temp: %d C\n", g_motorBatVoltage, g_motorBoardTemp);

  Serial.println("-- Origin --");
  if (g_origin.valid) {
    Serial.printf("  Lat: %.8f, Lon: %.8f\n", g_origin.lat, g_origin.lon);
  } else {
    Serial.println("  Not set");
  }

  Serial.println();
  Serial.printf("Free heap: %lu bytes\n", (unsigned long)ESP.getFreeHeap());
  Serial.printf("======================================\n");
}

// ============== MAIN ==============

struct GpsUartCandidate {
  int rx;
  int tx;
  const char* label;
};

static void writeUbxPoll(uint8_t cls, uint8_t id) {
  uint8_t ckA = 0, ckB = 0;
  auto addCk = [&](uint8_t b) { ckA += b; ckB += ckA; };
  GpsSerial.write(0xB5);
  GpsSerial.write(0x62);
  GpsSerial.write(cls); addCk(cls);
  GpsSerial.write(id); addCk(id);
  GpsSerial.write((uint8_t)0); addCk(0);
  GpsSerial.write((uint8_t)0); addCk(0);
  GpsSerial.write(ckA);
  GpsSerial.write(ckB);
  GpsSerial.flush();
}

static uint32_t scoreGpsUart(uint32_t passiveMs, bool activeProbe) {
  uint32_t bytes = 0;
  uint32_t score = 0;
  int prev = -1;
  uint32_t start = millis();
  bool probeSent = false;

  while (millis() - start < passiveMs) {
    if (activeProbe && !probeSent && millis() - start > passiveMs / 3) {
      writeUbxPoll(0x01, 0x07);  // UBX-NAV-PVT poll
      probeSent = true;
    }
    while (GpsSerial.available()) {
      uint8_t b = GpsSerial.read();
      bytes++;
      if (b == '$') score += 8;
      if (b == 0xD3) score += 3;
      if (prev == 0xB5 && b == 0x62) score += 40;
      if (b >= 32 && b <= 126) score++;
      prev = b;
    }
    delay(1);
  }
  return score + min(bytes, (uint32_t)200);
}

static void selectGpsUart() {
  const GpsUartCandidate candidates[] = {
    {GPS_RX, GPS_TX, "normal"},
    {GPS_ALT_RX, GPS_ALT_TX, "swapped"},
  };
  uint32_t bestScore = 0;
  uint8_t bestIndex = 0;

  Serial.println("GPS UART autodetect: checking normal and swapped TX/RX");
  for (uint8_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++) {
    GpsSerial.end();
    delay(50);
    GpsSerial.begin(GPS_BAUD, SERIAL_8N1, candidates[i].rx, candidates[i].tx);
    delay(200);
    while (GpsSerial.available()) GpsSerial.read();
    uint32_t score = scoreGpsUart(1200, true);
    Serial.printf("GPS UART candidate %s RX=%d TX=%d score=%lu\n",
      candidates[i].label, candidates[i].rx, candidates[i].tx, (unsigned long)score);
    if (score > bestScore) {
      bestScore = score;
      bestIndex = i;
    }
  }

  g_activeGpsRx = candidates[bestIndex].rx;
  g_activeGpsTx = candidates[bestIndex].tx;
  GpsSerial.end();
  delay(50);
  GpsSerial.begin(GPS_BAUD, SERIAL_8N1, g_activeGpsRx, g_activeGpsTx);
  delay(200);
  Serial.printf("GPS UART selected: %s RX=%d TX=%d score=%lu\n",
    candidates[bestIndex].label, g_activeGpsRx, g_activeGpsTx, (unsigned long)bestScore);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("========================================");
  Serial.println("   RTK ROVER AUTOPILOT v2.0");
  Serial.println("   Advanced Navigation System");
  Serial.println("========================================");
  Serial.printf("GPS UART default: RX=%d TX=%d alt RX=%d TX=%d %lu baud\n",
    GPS_RX, GPS_TX, GPS_ALT_RX, GPS_ALT_TX, (unsigned long)GPS_BAUD);
  Serial.printf("Motor UART: RX=%d TX=%d %lu baud\n", MOTOR_RX, MOTOR_TX, (unsigned long)MOTOR_BAUD);
  Serial.printf("IMU I2C: SDA=%d SCL=%d\n", IMU_SDA, IMU_SCL);
  Serial.printf("WebSocket port: %u\n", WS_PORT);
  Serial.printf("Reset reason: %s (%d)\n",
    resetReasonString(esp_reset_reason()), (int)esp_reset_reason());

  g_prefs.begin("rover-nav", false);
  loadNavPrefs();
  Serial.printf("NAV prefs: imuOffset=%.2f invYaw=%u invFwd=%u invSteer=%u fwd=%.2f turn=%.2f\n",
    g_imuCalibrationOffset, g_invertYaw ? 1 : 0, g_invertForward ? 1 : 0,
    g_invertSteering ? 1 : 0, g_navForwardScale, g_navTurnScale);

  pinMode(PIN_RELAY_ATTACHMENT, OUTPUT);
  pinMode(PIN_RELAY_MOUNT, OUTPUT);
  setAttachmentRelay(false);
  setMountRelay(false);

  // GPS
  selectGpsUart();
  Serial.println("GPS UART initialized");
  configureGpsRover();

  // IMU
  if (initImu()) {
    Serial.println("IMU: OK");
    g_est.heading = g_imuYaw;
  } else {
    Serial.println("IMU: FAILED - continuing without IMU");
  }

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.config(ROVER_IP, GATEWAY, SUBNET);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.println("WiFi: connecting...");
}

void loop() {
  uint32_t now = millis();

  handleSimulationSerialInput();
  if (g_simulationMode) {
    yield();
    return;
  }

  // WiFi
  if (WiFi.status() == WL_CONNECTED) {
    if (!g_wifiConnected) {
      g_wifiConnected = true;
      g_wifiReconnectIntervalMs = WIFI_RECONNECT_INITIAL_MS;
      Serial.println();
      Serial.println("!!! WiFi CONNECTED !!!");
      Serial.printf("   IP: %s\n", WiFi.localIP().toString().c_str());

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
    if (now - g_lastWifiReconnectMs >= g_wifiReconnectIntervalMs) {
      g_lastWifiReconnectMs = now;
      Serial.printf("WiFi: reconnecting, next retry in %lu ms...\n",
        (unsigned long)min(g_wifiReconnectIntervalMs * 2, WIFI_RECONNECT_MAX_MS));
      if (!WiFi.reconnect()) {
        WiFi.config(ROVER_IP, GATEWAY, SUBNET);
        WiFi.begin(WIFI_SSID, WIFI_PASS);
      }
      g_wifiReconnectIntervalMs = min(g_wifiReconnectIntervalMs * 2, WIFI_RECONNECT_MAX_MS);
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

  // Failsafe: if no fresh motor command arrived within CMD_TIMEOUT_MS, we
  // are blind to the operator (WiFi/WebSocket stall, app crash, USB
  // unplugged). The previous code only logged the event but left
  // g_targetLeft/Right at the last values, so the ramp kept moving the
  // robot toward a wall until something else cleared the targets. This
  // sets targets to zero and the ramp glides to a stop.
  if (now - g_lastCmdMs > CMD_TIMEOUT_MS) {
    if (!g_isFailSafeStopping &&
        (g_targetLeft != 0 || g_targetRight != 0 || g_curLeft != 0 || g_curRight != 0)) {
      g_isFailSafeStopping = true;
      Serial.printf("FAILSAFE: smooth stop, last cmd %lu ms ago\n",
        (unsigned long)(now - g_lastCmdMs));
    }
    if (g_isFailSafeStopping) {
      g_targetLeft = 0;
      g_targetRight = 0;
    }
  } else {
    g_isFailSafeStopping = false;
  }

  // Motor control
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

  yield();
}
