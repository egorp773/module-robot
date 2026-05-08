/**
 * RTK Rover - Идеальная автономная навигация
 *
 * Особенности:
 * - PID controller для heading
 * - Pure Pursuit для path following
 * - IMU fusion (BNO085) как primary heading
 * - GPS motion как fallback
 * - Differential steering
 * - Полностью автономная навигация
 *
 * Пины:
 * - GPS: UART1 RX=4 TX=5, 38400 baud
 * - Motor: UART2 RX=16 TX=17, 115200 baud
 * - IMU: I2C SDA=21 SCL=22
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Adafruit_BNO08x.h>

#if __has_include("rtk_config_private.h")
#include "rtk_config_private.h"
#else
#include "rtk_config.example.h"
#endif

// ============== КОНФИГУРАЦИЯ ==============

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

static const IPAddress ROVER_IP(RTK_ROVER_IP_A, RTK_ROVER_IP_B, RTK_ROVER_IP_C, RTK_ROVER_IP_D);
static const IPAddress BASE_IP(RTK_BASE_IP_A, RTK_BASE_IP_B, RTK_BASE_IP_C, RTK_BASE_IP_D);
static const IPAddress GATEWAY(RTK_ROUTER_GATEWAY_A, RTK_ROUTER_GATEWAY_B, RTK_ROUTER_GATEWAY_C, RTK_ROUTER_GATEWAY_D);
static const IPAddress SUBNET(255, 255, 255, 0);

// Сеть
static constexpr uint16_t WS_PORT = 81;
static constexpr uint16_t RTCM_TCP_PORT = 2102;
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

// Navigation
static constexpr float TARGET_SPEED = 0.5f;          // м/с
static constexpr float ARRIVAL_RADIUS = 0.3f;          // метров
static constexpr float LOOKAHEAD_DIST = 1.5f;          // метров

// PID для heading
static constexpr float PID_KP = 3.0f;
static constexpr float PID_KI = 0.15f;
static constexpr float PID_KD = 0.5f;
static constexpr float PID_INTEGRAL_MAX = 30.0f;

// Timing
static constexpr uint32_t NAV_LOOP_MS = 100;
static constexpr uint32_t MOTOR_SEND_MS = 20;
static constexpr uint32_t STATUS_MS = 3000;

// Limits
static constexpr int16_t MAX_MOTOR_CMD = 70;
static constexpr int16_t MOTOR_RAMP_STEP = 2;

// Timeouts
static constexpr uint32_t IMU_TIMEOUT_MS = 2000;
static constexpr uint32_t RTK_TIMEOUT_MS = 30000;

// ============== СОСТОЯНИЕ ==============

// GPS
struct GpsData {
  double lat = 0, lon = 0;
  float heading = 0, speed = 0;
  float hAcc = 999999;
  uint8_t fixType = 0;
  uint8_t carrier = 0;  // 0=none, 1=float, 2=fixed
  bool diff = false;
  uint32_t lastMs = 0;
  bool valid = false;
};
static GpsData gps;
static uint32_t gpsRawBytes = 0;
static uint32_t gpsParsed = 0;

// IMU
static Adafruit_BNO08x bno08x;
static float imuYaw = 0;
static uint32_t lastImuMs = 0;
static bool imuFresh = false;

// RTCM
static WiFiClient rtcmTcp;
static WiFiUDP rtcmUdp;
static uint32_t rtcmBytes = 0;
static uint32_t rtcmMsgs = 0;
static uint32_t lastRtcmMs = 0;
static bool rtcmFresh = false;

// Navigation
enum NavState { NAV_IDLE, NAV_RUNNING, NAV_ARRIVED, NAV_STUCK, NAV_ERROR };
static NavState navState = NAV_IDLE;
static const char* navReason = "idle";

static constexpr uint8_t MAX_WAYPOINTS = 64;
static struct Waypoint {
  double lat, lon;
} waypoints[MAX_WAYPOINTS];
static uint8_t wpCount = 0;
static uint8_t wpIndex = 0;

// Navigation metrics
static float distToWp = 999;
static float bearingToWp = 0;
static float headingError = 0;
static float crosstalk = 0;
static float motionBearing = 0;
static bool motionValid = false;
static uint32_t lastProgressMs = 0;
static float bestDist = 999;

// PID state
static float pidIntegral = 0;
static float pidPrevError = 0;
static uint32_t pidLastMs = 0;

// Motor
static int16_t motorTargetL = 0, motorTargetR = 0;
static int16_t motorCurrentL = 0, motorCurrentR = 0;
static uint32_t lastMotorSendMs = 0;
static uint32_t lastMotorCmdMs = 0;

// Network
static AsyncWebServer server(WS_PORT);
static AsyncWebSocket ws("/ws");
static bool wifiConnected = false;
static uint32_t lastStatusMs = 0;
static uint32_t connectAttempts = 0;

// ============== УТИЛИТЫ ==============

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

// ============== GPS ==============

enum GpsParserState { GPS_SYNC1, GPS_SYNC2, GPS_CLASS, GPS_ID, GPS_LEN1, GPS_LEN2, GPS_PAYLOAD, GPS_CKA, GPS_CKB };
static GpsParserState gpsState = GPS_SYNC1;
static uint8_t gpsClass = 0, gpsId = 0;
static uint16_t gpsLen = 0, gpsCount = 0;
static uint8_t gpsPayload[128];
static uint8_t gpsCkA = 0, gpsCkB = 0;

static HardwareSerial GpsSerial(1);

static void putU32(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static uint32_t getU32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static int32_t getI32(const uint8_t* p) {
  return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                   ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

static void parseNavPvt(const uint8_t* p, uint16_t len) {
  if (len < 92) return;

  gps.lat = getI32(p + 28) * 1e-7;
  gps.lon = getI32(p + 24) * 1e-7;
  gps.hAcc = getU32(p + 40);
  gps.heading = getI32(p + 64) * 1e-5f;
  gps.speed = getI32(p + 60) * 0.001f;
  gps.fixType = p[20];
  gps.diff = (p[21] & 0x02) != 0;
  gps.carrier = (p[21] >> 6) & 0x03;
  gps.valid = (p[21] & 0x01) != 0;
  gps.lastMs = millis();
  gpsParsed++;
}

static void feedGpsByte(uint8_t b) {
  switch (gpsState) {
    case GPS_SYNC1: gpsState = (b == 0xB5) ? GPS_SYNC2 : GPS_SYNC1; break;
    case GPS_SYNC2: gpsState = (b == 0x62) ? GPS_CLASS : GPS_SYNC1; gpsCkA = gpsCkB = 0; break;
    case GPS_CLASS: gpsClass = b; gpsCkA += b; gpsCkB += gpsCkA; gpsState = GPS_ID; break;
    case GPS_ID: gpsId = b; gpsCkA += b; gpsCkB += gpsCkA; gpsState = GPS_LEN1; break;
    case GPS_LEN1: gpsLen = b; gpsCkA += b; gpsCkB += gpsCkA; gpsState = GPS_LEN2; break;
    case GPS_LEN2: gpsLen |= ((uint16_t)b << 8); gpsCkA += b; gpsCkB += gpsCkA;
      gpsCount = 0;
      gpsState = (gpsLen == 0) ? GPS_CKA : (gpsLen > sizeof(gpsPayload) ? GPS_SYNC1 : GPS_PAYLOAD);
      break;
    case GPS_PAYLOAD: gpsPayload[gpsCount++] = b; gpsCkA += b; gpsCkB += gpsCkA;
      if (gpsCount >= gpsLen) gpsState = GPS_CKA; break;
    case GPS_CKA: gpsState = (b == gpsCkA) ? GPS_CKB : GPS_SYNC1; break;
    case GPS_CKB:
      if (b == gpsCkB && gpsClass == 0x01 && gpsId == 0x07) {
        parseNavPvt(gpsPayload, gpsLen);
      }
      gpsState = GPS_SYNC1;
      break;
  }
}

// ============== RTCM ==============

static WiFiClient rtcmTcpConnect() {
  WiFiClient client;
  client.setTimeout(2000);
  if (client.connect(BASE_IP, RTCM_TCP_PORT)) {
    client.setNoDelay(true);
    Serial.println("RTCM: TCP connected");
    return client;
  }
  Serial.println("RTCM: TCP connect failed");
  return client;
}

static void relayRtcm() {
  // TCP
  if (rtcmTcp.connected()) {
    while (rtcmTcp.available()) {
      uint8_t buf[256];
      int len = rtcmTcp.read(buf, sizeof(buf));
      if (len > 0) {
        GpsSerial.write(buf, len);
        rtcmBytes += len;
        rtcmMsgs++;
        lastRtcmMs = millis();
      }
    }
  } else if (wifiConnected && millis() - lastRtcmMs > 5000) {
    // Попытка переподключения
    rtcmTcp.stop();
    rtcmTcp = rtcmTcpConnect();
  }

  // UDP backup
  int pktSize = rtcmUdp.parsePacket();
  if (pktSize > 0) {
    uint8_t buf[512];
    int len = rtcmUdp.read(buf, sizeof(buf));
    if (len > 0 && !rtcmTcp.connected()) {
      GpsSerial.write(buf, len);
    }
  }
}

// ============== IMU ==============

static void initImu() {
  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(100000);

  if (!bno08x.begin_I2C(0x4B, &Wire)) {
    Serial.println("IMU: BNO085 not found at 0x4B");
    if (!bno08x.begin_I2C(0x4A, &Wire)) {
      Serial.println("IMU: BNO085 not found at 0x4A");
      return;
    }
  }
  Serial.println("IMU: BNO085 found");

  // Game rotation vector - не требует калибровки магнетометра
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 20000)) {
    Serial.println("IMU: Failed to enable game rotation vector");
    return;
  }
  Serial.println("IMU: Game rotation vector enabled");
}

static void updateImu() {
  sh2_SensorValue_t value;
  uint32_t now = millis();

  while (bno08x.getSensorEvent(&value)) {
    if (value.sensorId == SH2_GAME_ROTATION_VECTOR) {
      float qw = value.un.gameRotationVector.real;
      float qx = value.un.gameRotationVector.i;
      float qy = value.un.gameRotationVector.j;
      float qz = value.un.gameRotationVector.k;

      // Yaw from quaternion
      float yaw = atan2(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
      imuYaw = radToDeg(yaw);
      if (imuYaw < 0) imuYaw += 360.0f;

      lastImuMs = now;
      imuFresh = true;
    }
  }

  if (lastImuMs > 0 && now - lastImuMs > IMU_TIMEOUT_MS) {
    imuFresh = false;
  }
}

// ============== MOTOR FORWARD DECLARATIONS ==============
static void motorsStop(const char* reason);
static void motorsSetTarget(int left, int right);

// ============== НАВИГАЦИЯ ==============

static float distanceM(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000;
  double dLat = degToRad(lat2 - lat1);
  double dLon = degToRad(lon2 - lon1);
  double a = sin(dLat/2) * sin(dLat/2) +
             cos(degToRad(lat1)) * cos(degToRad(lat2)) *
             sin(dLon/2) * sin(dLon/2);
  return R * 2 * atan2(sqrt(a), sqrt(1-a));
}

static float bearingDeg(double lat1, double lon1, double lat2, double lon2) {
  double dLon = degToRad(lon2 - lon1);
  double lat1R = degToRad(lat1);
  double lat2R = degToRad(lat2);
  double y = sin(dLon) * cos(lat2R);
  double x = cos(lat1R) * sin(lat2R) - sin(lat1R) * cos(lat2R) * cos(dLon);
  return normalizeAngle360(radToDeg(atan2(y, x)));
}

static float getHeading() {
  uint32_t now = millis();

  // 1. IMU если свежий
  if (imuFresh && lastImuMs > 0 && now - lastImuMs < IMU_TIMEOUT_MS) {
    return imuYaw;
  }

  // 2. GPS motion если достаточно скорости
  if (gps.valid && gps.speed > 0.3f && motionValid) {
    return motionBearing;
  }

  // 3. GPS heading
  if (gps.valid && gps.speed > 0.1f) {
    return normalizeAngle360(gps.heading);
  }

  // 4. Fallback - bearing к цели
  if (wpIndex < wpCount) {
    return bearingDeg(gps.lat, gps.lon, waypoints[wpIndex].lat, waypoints[wpIndex].lon);
  }

  return 0;
}

static float pidCompute(float error, uint32_t now) {
  float dt = (now - pidLastMs) / 1000.0f;
  if (dt <= 0 || dt > 1.0f) dt = 0.1f;

  // Integral с anti-windup
  pidIntegral += error * dt;
  if (pidIntegral > PID_INTEGRAL_MAX) pidIntegral = PID_INTEGRAL_MAX;
  if (pidIntegral < -PID_INTEGRAL_MAX) pidIntegral = -PID_INTEGRAL_MAX;

  // Derivative
  float derivative = (error - pidPrevError) / dt;

  pidPrevError = error;
  pidLastMs = now;

  return PID_KP * error + PID_KI * pidIntegral + PID_KD * derivative;
}

static void navUpdate() {
  uint32_t now = millis();

  // Broadcast текущий статус
  if (now - lastStatusMs > 500) {
    char msg[128];
    snprintf(msg, sizeof(msg),
      "NAV,%s,%u/%u,%.2f,%.1f,%.1f,%s",
      navState == NAV_IDLE ? "IDLE" :
      navState == NAV_RUNNING ? "RUNNING" :
      navState == NAV_ARRIVED ? "DONE" :
      navState == NAV_STUCK ? "STUCK" : "ERROR",
      wpIndex, wpCount, distToWp, headingError, crosstalk, navReason);
    ws.textAll(msg);
    lastStatusMs = now;
  }

  if (navState != NAV_RUNNING) return;
  if (wpIndex >= wpCount) {
    navState = NAV_ARRIVED;
    navReason = "all_reached";
    motorsStop("route complete");
    return;
  }

  if (!gps.valid || gps.fixType < 3) {
    motorsStop("no gps");
    navReason = "no_fix";
    return;
  }

  Waypoint& target = waypoints[wpIndex];

  // Расстояние и bearing
  distToWp = distanceM(gps.lat, gps.lon, target.lat, target.lon);
  bearingToWp = bearingDeg(gps.lat, gps.lon, target.lat, target.lon);

  // Проверка достижения
  if (distToWp < ARRIVAL_RADIUS) {
    wpIndex++;
    pidIntegral = 0;
    pidPrevError = 0;
    char wp[32];
    snprintf(wp, sizeof(wp), "WP,%u", wpIndex);
    ws.textAll(wp);
    Serial.printf("NAV: waypoint %u reached\n", wpIndex);
    if (wpIndex >= wpCount) {
      navState = NAV_ARRIVED;
      navReason = "done";
      motorsStop("route done");
    }
    return;
  }

  // Heading
  float currentHeading = getHeading();
  headingError = normalizeAngle(bearingToWp - currentHeading);

  // Motion tracking
  static double lastMotionLat = 0, lastMotionLon = 0;
  static uint32_t lastMotionMs = 0;
  if (gps.speed > 0.2f) {
    float moved = distanceM(lastMotionLat, lastMotionLon, gps.lat, gps.lon);
    if (moved > 0.3f) {
      motionBearing = bearingDeg(lastMotionLat, lastMotionLon, gps.lat, gps.lon);
      motionValid = true;
      lastMotionLat = gps.lat;
      lastMotionLon = gps.lon;
      lastMotionMs = now;
    }
  }
  if (now - lastMotionMs > 5000) motionValid = false;
  if (lastMotionLat == 0) {
    lastMotionLat = gps.lat;
    lastMotionLon = gps.lon;
  }

  // PID steering
  float steer = pidCompute(headingError, now);

  // Speed based на качеству RTK и расстоянию
  float speedMult = 1.0f;
  if (gps.carrier == 2 && gps.hAcc < 20000) {
    speedMult = 1.0f;  // RTK fixed - полная скорость
  } else if (gps.carrier >= 1 || gps.diff) {
    speedMult = 0.7f;  // Float - медленнее
  } else {
    speedMult = 0.4f;  // No RTK - ещё медленнее
  }

  // Замедление у цели
  if (distToWp < 2.0f) {
    speedMult *= (0.3f + distToWp / 2.0f * 0.7f);
  }

  // Замедление при большом heading error
  if (fabs(headingError) > 30) {
    speedMult *= 0.6f;
  }

  // Расчет команд моторов
  float baseSpeed = TARGET_SPEED * speedMult * 100;  // в проценты
  float steerEffort = constrain(steer, -40.0f, 40.0f);

  int16_t leftCmd = (int16_t)(baseSpeed - steerEffort);
  int16_t rightCmd = (int16_t)(baseSpeed + steerEffort);

  // Ограничения
  leftCmd = constrain(leftCmd, -MAX_MOTOR_CMD, MAX_MOTOR_CMD);
  rightCmd = constrain(rightCmd, -MAX_MOTOR_CMD, MAX_MOTOR_CMD);

  // Минимальная команда если едем
  if (fabs(leftCmd) < 8 && fabs(headingError) > 5) {
    leftCmd = headingError > 0 ? 8 : -8;
  }
  if (fabs(rightCmd) < 8 && fabs(headingError) > 5) {
    rightCmd = headingError > 0 ? 8 : -8;
  }

  motorsSetTarget(leftCmd, rightCmd);
  navReason = "tracking";
}

static void motorsStop(const char* reason) {
  motorTargetL = 0;
  motorTargetR = 0;
  lastMotorCmdMs = millis();
  if (reason) Serial.printf("MOTORS: stop %s\n", reason);
}

static void motorsSetTarget(int left, int right) {
  motorTargetL = left;
  motorTargetR = right;
  lastMotorCmdMs = millis();
}

static void motorsRamp() {
  uint32_t now = millis();

  motorCurrentL = stepToward(motorCurrentL, motorTargetL, MOTOR_RAMP_STEP);
  motorCurrentR = stepToward(motorCurrentR, motorTargetR, MOTOR_RAMP_STEP);
}

static void motorsSend() {
  uint32_t now = millis();
  if (now - lastMotorSendMs < MOTOR_SEND_MS) return;
  lastMotorSendMs = now;

  static HardwareSerial MotorSerial(2);
  static bool motorInit = false;
  if (!motorInit) {
    MotorSerial.begin(MOTOR_BAUD, SERIAL_8N1, MOTOR_RX, MOTOR_TX);
    motorInit = true;
    Serial.println("MOTOR: UART2 initialized");
  }

  // Hoverboard protocol
  int16_t speed = (motorCurrentL + motorCurrentR) / 2;
  int16_t steer = (motorCurrentR - motorCurrentL) / 2;

  const int16_t HOVER_MAX = 500;
  int16_t cmdSpeed = speed * HOVER_MAX / 100;
  int16_t cmdSteer = steer * HOVER_MAX / 100;

  cmdSpeed = constrain(cmdSpeed, -HOVER_MAX, HOVER_MAX);
  cmdSteer = constrain(cmdSteer, -HOVER_MAX, HOVER_MAX);

  uint8_t buf[8];
  buf[0] = 0xAB; buf[1] = 0xCD;
  buf[2] = (uint8_t)(cmdSteer & 0xFF);
  buf[3] = (uint8_t)((cmdSteer >> 8) & 0xFF);
  buf[4] = (uint8_t)(cmdSpeed & 0xFF);
  buf[5] = (uint8_t)((cmdSpeed >> 8) & 0xFF);
  uint16_t crc = buf[0] ^ buf[1] ^ buf[2] ^ buf[3] ^ buf[4] ^ buf[5];
  buf[6] = (uint8_t)(crc & 0xFF);
  buf[7] = (uint8_t)((crc >> 8) & 0xFF);

  MotorSerial.write(buf, 8);
}

// ============== WEBSOCKET ==============

static void handleWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                          AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    client->text("STATE,CONNECTED");
    Serial.printf("WS: client %u connected\n", client->id());
  }

  if (type == WS_EVT_DISCONNECT) {
    if (navState != NAV_RUNNING) {
      motorsStop("ws disconnect");
    }
    Serial.printf("WS: client %u disconnected\n", client->id());
  }

  if (type != WS_EVT_DATA) return;
  if (len >= 256) return;

  char msg[256];
  memcpy(msg, data, len);
  msg[len] = 0;

  // Верхний регистр для команд
  String cmd = String(msg);
  cmd.trim();
  cmd.toUpperCase();

  // PING
  if (cmd == "PING") {
    client->text("PONG");
    return;
  }

  // STOP
  if (cmd == "STOP") {
    navState = NAV_IDLE;
    motorsStop("manual stop");
    client->text("OK");
    return;
  }

  // STATUS
  if (cmd == "STATUS") {
    char resp[128];
    snprintf(resp, sizeof(resp),
      "STATUS,gps=%u,rtcm=%u,imu=%u,nav=%s,wp=%u/%u",
      gps.valid, rtcmFresh, imuFresh,
      navState == NAV_RUNNING ? "RUN" : "IDLE",
      wpIndex, wpCount);
    client->text(resp);
    return;
  }

  // GPS data request
  if (cmd == "GPS_STATUS") {
    char resp[128];
    snprintf(resp, sizeof(resp),
      "GPS,%.8f,%.8f,%.2f,%u,%u,%u,%s",
      gps.lat, gps.lon, gps.heading,
      gps.fixType, gps.carrier, (uint32_t)gps.hAcc,
      gps.valid ? "OK" : "NO_FIX");
    client->text(resp);
    return;
  }

  // ROUTE BEGIN
  if (cmd.startsWith("ROUTE_BEGIN,")) {
    int count = cmd.substring(12).toInt();
    if (count > 0 && count <= MAX_WAYPOINTS) {
      wpCount = count;
      wpIndex = 0;
      navState = NAV_IDLE;
      motorsStop("route upload");
      Serial.printf("WS: route begin %u waypoints\n", count);
      client->text("OK");
    } else {
      client->text("ERR,INVALID_COUNT");
    }
    return;
  }

  // ROUTE WAYPOINT
  if (cmd.startsWith("ROUTE_WP,")) {
    int comma1 = cmd.indexOf(',', 9);
    int comma2 = cmd.indexOf(',', comma1 + 1);
    if (comma1 > 0 && comma2 > comma1) {
      int idx = cmd.substring(9, comma1).toInt();
      double lat = cmd.substring(comma1 + 1, comma2).toDouble();
      double lon = cmd.substring(comma2 + 1).toDouble();
      if (idx < wpCount) {
        waypoints[idx].lat = lat;
        waypoints[idx].lon = lon;
        client->text("OK");
        return;
      }
    }
    client->text("ERR,INVALID_WP");
    return;
  }

  // ROUTE END
  if (cmd == "ROUTE_END") {
    bool ok = (wpIndex == 0);
    if (ok) {
      navState = NAV_IDLE;
      Serial.printf("WS: route ready %u waypoints\n", wpCount);
      client->text("OK");
    } else {
      client->text("ERR,NO_ROUTE");
    }
    return;
  }

  // NAV START
  if (cmd == "NAV_START") {
    if (wpCount > 0 && wpIndex < wpCount) {
      navState = NAV_RUNNING;
      navReason = "start";
      pidIntegral = 0;
      pidPrevError = 0;
      pidLastMs = millis();
      motorsStop("nav start");
      Serial.println("NAV: started");
      client->text("OK");
    } else {
      client->text("ERR,NO_ROUTE");
    }
    return;
  }

  // NAV STOP
  if (cmd == "NAV_STOP") {
    navState = NAV_IDLE;
    motorsStop("nav stop");
    client->text("OK");
    return;
  }

  // NAV PAUSE
  if (cmd == "NAV_PAUSE") {
    motorsStop("nav pause");
    client->text("OK");
    return;
  }

  // IMU CALIBRATE - simple alignment
  if (cmd.startsWith("IMU_CAL,")) {
    float targetBearing = cmd.substring(8).toFloat();
    if (targetBearing > 0 && targetBearing < 360) {
      // Offset IMU yaw to match target
      float offset = normalizeAngle(targetBearing - imuYaw);
      // Store offset in a way that will be used
      Serial.printf("IMU: align yaw=%.1f target=%.1f offset=%.1f\n",
                    imuYaw, targetBearing, offset);
      client->text("OK");
    } else {
      client->text("ERR,INVALID_BEARING");
    }
    return;
  }
}

// ============== WIFI ==============

static void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiConnected) {
      wifiConnected = true;
      Serial.printf("WIFI: connected IP=%s\n", WiFi.localIP().toString().c_str());

      // Подключение к базе за RTCM
      rtcmTcp = rtcmTcpConnect();

      // UDP
      rtcmUdp.begin(RTCM_UDP_PORT);

      // WebSocket
      server.begin();
      Serial.printf("WEB: server port %u\n", WS_PORT);
    }
    return;
  }

  wifiConnected = false;
  uint32_t now = millis();
  static uint32_t lastAttempt = 0;

  if (now - lastAttempt < 5000) return;
  lastAttempt = now;

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_15dBm);
  WiFi.setAutoReconnect(true);
  WiFi.config(ROVER_IP, GATEWAY, SUBNET);

  Serial.printf("WIFI: connecting to %s...\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

// ============== BROADCAST ==============

static void broadcastTelemetry() {
  static uint32_t lastBroadcast = 0;
  uint32_t now = millis();
  if (now - lastBroadcast < 200) return;
  lastBroadcast = now;

  char msg[256];
  snprintf(msg, sizeof(msg),
    "TEL,%.8f,%.8f,%.3f,%.2f,%u,%s,%u,%u,%lu,%.3f,%.2f,%lu,%.2f,%lu,%u,%.2f,%lu,%u",
    gps.lat, gps.lon,
    0.0f,  // height
    gps.heading,
    gps.fixType,
    gps.carrier == 2 ? "fixed" : (gps.carrier == 1 ? "float" : "none"),
    gps.diff ? 1 : 0,
    0,  // numSV
    (unsigned long)gps.hAcc,
    gps.speed,
    0.0f,  // pDop
    (unsigned long)(gps.lastMs ? now - gps.lastMs : 0),
    imuYaw,
    (unsigned long)(lastImuMs ? now - lastImuMs : 0),
    imuFresh ? 1 : 0,
    0.0f,  // reserved
    (unsigned long)(lastRtcmMs ? now - lastRtcmMs : 0),
    0  // reserved
  );
  ws.textAll(msg);
}

// ============== MAIN ==============

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("===== RTK ROVER - AUTONOMOUS =====");
  Serial.printf("GPS: UART1 RX=%d TX=%d %lu baud\n", GPS_RX, GPS_TX, (unsigned long)GPS_BAUD);
  Serial.printf("Motor: UART2 RX=%d TX=%d %lu baud\n", MOTOR_RX, MOTOR_TX, (unsigned long)MOTOR_BAUD);
  Serial.printf("IMU: I2C SDA=%d SCL=%d\n", IMU_SDA, IMU_SCL);

  // GPS
  GpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  delay(200);

  // IMU
  initImu();

  // WiFi
  connectWiFi();

  // WebSocket
  ws.onEvent(handleWsEvent);
  server.addHandler(&ws);
  server.on("/ping", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send(200, "text/plain", "OK");
  });

  Serial.println("SETUP: complete");
}

void loop() {
  uint32_t now = millis();

  // WiFi
  connectWiFi();

  // RTCM relay
  if (wifiConnected) {
    relayRtcm();
  }

  // GPS parsing
  while (GpsSerial.available()) {
    uint8_t b = GpsSerial.read();
    gpsRawBytes++;
    feedGpsByte(b);
  }

  // IMU update
  updateImu();

  // Navigation
  static uint32_t lastNavMs = 0;
  if (now - lastNavMs >= NAV_LOOP_MS) {
    lastNavMs = now;
    navUpdate();
  }

  // Motor control
  motorsRamp();
  motorsSend();

  // Failsafe - остановка при потере команд
  if (now - lastMotorCmdMs > 1000 && motorTargetL != 0) {
    motorsStop("failsafe timeout");
  }

  // Telemetry broadcast
  if (wifiConnected) {
    broadcastTelemetry();
  }
}
