#include "nav.h"
#include "config.h"
#include "gps.h"
#include "imu.h"
#include <math.h>

NavState g_navState = NAV_IDLE;
int g_navWpIndex = 0;
int g_navWpTotal = 0;
float g_navDistToWp = 0.0f;
Waypoint g_waypoints[MAX_WAYPOINTS];

static uint32_t g_lastNavMs = 0;
static uint32_t g_lastGoodGpsMs = 0;

// PID state
static float g_pidIntegral = 0.0f;
static float g_pidLastError = 0.0f;

static inline float normalizeAngle(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

static inline float distance(double lat1, double lon1, double lat2, double lon2) {
  // Haversine formula
  const float R = 6371000.0f; // Earth radius in meters
  float dLat = (lat2 - lat1) * PI / 180.0f;
  float dLon = (lon2 - lon1) * PI / 180.0f;
  float a = sin(dLat / 2.0f) * sin(dLat / 2.0f) +
            cos(lat1 * PI / 180.0f) * cos(lat2 * PI / 180.0f) *
            sin(dLon / 2.0f) * sin(dLon / 2.0f);
  float c = 2.0f * atan2(sqrt(a), sqrt(1.0f - a));
  return R * c;
}

static inline float bearing(double lat1, double lon1, double lat2, double lon2) {
  float dLon = (lon2 - lon1) * PI / 180.0f;
  float y = sin(dLon) * cos(lat2 * PI / 180.0f);
  float x = cos(lat1 * PI / 180.0f) * sin(lat2 * PI / 180.0f) -
            sin(lat1 * PI / 180.0f) * cos(lat2 * PI / 180.0f) * cos(dLon);
  float brng = atan2(y, x) * 180.0f / PI;
  if (brng < 0) brng += 360.0f;
  return brng;
}

void nav_init() {
  g_navState = NAV_IDLE;
  g_navWpIndex = 0;
  g_navWpTotal = 0;
  g_targetLeft = 0;
  g_targetRight = 0;
  g_pidIntegral = 0.0f;
  g_pidLastError = 0.0f;
  Serial.println("NAV: Initialized");
}

void nav_clear_route() {
  g_navWpTotal = 0;
  g_navWpIndex = 0;
  Serial.println("NAV: Route cleared");
}

bool nav_add_waypoint(double lat, double lon) {
  if (g_navWpTotal >= MAX_WAYPOINTS) {
    Serial.println("NAV: Waypoint buffer full");
    return false;
  }
  g_waypoints[g_navWpTotal].lat = lat;
  g_waypoints[g_navWpTotal].lon = lon;
  g_navWpTotal++;
  return true;
}

void nav_start() {
  if (g_navWpTotal == 0) {
    Serial.println("NAV: No waypoints loaded");
    g_navState = NAV_ERROR;
    return;
  }

  g_navState = NAV_RUNNING;
  g_navWpIndex = 0;
  g_pidIntegral = 0.0f;
  g_pidLastError = 0.0f;
  g_lastNavMs = millis();
  g_lastGoodGpsMs = millis();
  Serial.println("NAV: Started");
}

void nav_pause() {
  if (g_navState == NAV_RUNNING) {
    g_navState = NAV_PAUSED;
    g_targetLeft = 0;
    g_targetRight = 0;
    Serial.println("NAV: Paused");
  }
}

void nav_resume() {
  if (g_navState == NAV_PAUSED) {
    g_navState = NAV_RUNNING;
    g_pidIntegral = 0.0f;
    g_pidLastError = 0.0f;
    Serial.println("NAV: Resumed");
  }
}

void nav_stop() {
  g_navState = NAV_IDLE;
  g_targetLeft = 0;
  g_targetRight = 0;
  g_pidIntegral = 0.0f;
  g_pidLastError = 0.0f;
  Serial.println("NAV: Stopped");
}

void nav_update() {
  uint32_t now = millis();

  if (g_navState != NAV_RUNNING) {
    return;
  }

  if (now - g_lastNavMs < NAV_UPDATE_MS) {
    return;
  }
  g_lastNavMs = now;

  // Check GPS validity
  if (!g_gpsData.valid || g_gpsData.hAcc > MIN_GPS_ACC_MM) {
    if (now - g_lastGoodGpsMs > GPS_TIMEOUT_MS) {
      Serial.println("NAV: GPS accuracy too low, stopping");
      g_navState = NAV_ERROR;
      g_targetLeft = 0;
      g_targetRight = 0;
      return;
    }
  } else {
    g_lastGoodGpsMs = now;
  }

  // Check if all waypoints reached
  if (g_navWpIndex >= g_navWpTotal) {
    Serial.println("NAV: All waypoints reached");
    g_navState = NAV_DONE;
    g_targetLeft = 0;
    g_targetRight = 0;
    return;
  }

  // Get current waypoint
  Waypoint wp = g_waypoints[g_navWpIndex];

  // Calculate distance and bearing to waypoint
  float dist = distance(g_gpsData.lat, g_gpsData.lon, wp.lat, wp.lon);
  g_navDistToWp = dist;

  // Check if waypoint reached
  if (dist < ARRIVAL_RADIUS) {
    Serial.printf("NAV: Waypoint %d reached\n", g_navWpIndex);
    g_navWpIndex++;
    g_pidIntegral = 0.0f;
    g_pidLastError = 0.0f;
    return;
  }

  // Calculate target bearing
  float targetBearing = bearing(g_gpsData.lat, g_gpsData.lon, wp.lat, wp.lon);

  // Use IMU heading if available, otherwise GPS heading
  float currentHeading = g_imuValid ? g_imuYaw : g_gpsData.headMot;

  // Calculate heading error
  float headingError = normalizeAngle(targetBearing - currentHeading);

  // PID controller
  float dt = NAV_UPDATE_MS / 1000.0f;
  g_pidIntegral += headingError * dt;
  g_pidIntegral = constrain(g_pidIntegral, -50.0f, 50.0f); // Anti-windup

  float derivative = (headingError - g_pidLastError) / dt;
  g_pidLastError = headingError;

  float steerCorrection = PID_KP_HEADING * headingError +
                          PID_KI_HEADING * g_pidIntegral +
                          PID_KD_HEADING * derivative;

  // Base speed modulation
  float baseSpeed = NOMINAL_SPEED;

  // Reduce speed when heading error is large
  if (abs(headingError) > 15.0f) {
    float headingScale = cos(headingError * PI / 180.0f);
    if (headingScale < 0.0f) headingScale = 0.0f;
    baseSpeed *= headingScale;
  }

  // Reduce speed when approaching waypoint (last 1m)
  if (dist < 1.0f) {
    baseSpeed *= dist;
  }

  // Calculate motor speeds
  int16_t leftSpeed = (int16_t)(baseSpeed - steerCorrection);
  int16_t rightSpeed = (int16_t)(baseSpeed + steerCorrection);

  // Clamp to limits
  leftSpeed = constrain(leftSpeed, -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);

  g_targetLeft = leftSpeed;
  g_targetRight = rightSpeed;
}
