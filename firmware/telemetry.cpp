#include "telemetry.h"
#include "config.h"
#include "gps.h"
#include "imu.h"
#include "nav.h"
#include "motors.h"
#include "websocket.h"

static uint32_t g_lastGpsBroadcastMs = 0;
static uint32_t g_lastBatSendMs = 0;
static uint32_t g_lastNavSendMs = 0;

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

void telemetry_update() {
  uint32_t now = millis();

  // GPS telemetry (5 Hz)
  if (now - g_lastGpsBroadcastMs >= GPS_BROADCAST_MS) {
    g_lastGpsBroadcastMs = now;

    if (g_gpsData.valid) {
      char buf[128];
      snprintf(buf, sizeof(buf), "GPS,%.8f,%.8f,%.2f,%d,%u",
               g_gpsData.lat, g_gpsData.lon, g_gpsData.headMot,
               g_gpsData.fixType, g_gpsData.hAcc);
      websocket_send(buf);
    }
  }

  // IMU telemetry (5 Hz)
  if (g_imuValid && now - g_lastGpsBroadcastMs < 10) {
    char buf[32];
    snprintf(buf, sizeof(buf), "IMU,%.2f", g_imuYaw);
    websocket_send(buf);
  }

  // Battery telemetry (2 Hz)
  if (g_haveFeedback && now - g_lastBatSendMs >= BAT_SEND_MS) {
    g_lastBatSendMs = now;

    float vCell = (CELL_COUNT > 0) ? (g_batVoltFiltered / (float)CELL_COUNT) : 0.0f;
    int pct = socFromCellV(vCell);

    char buf[32];
    snprintf(buf, sizeof(buf), "BAT_PCT,%d", pct);
    websocket_send(buf);

    if (SEND_BAT_VERBOSE) {
      char buf2[128];
      float tempC = (float)g_boardTemp * TEMP_SCALE;
      snprintf(buf2, sizeof(buf2), "BAT,V=%.2fV,P=%d%%,temp=%.1fC",
               g_batVoltFiltered, pct, tempC);
      websocket_send(buf2);
    }
  }

  // Navigation telemetry (2 Hz)
  if (now - g_lastNavSendMs >= 500) {
    g_lastNavSendMs = now;

    const char* stateStr = "IDLE";
    if (g_navState == NAV_RUNNING) stateStr = "RUNNING";
    else if (g_navState == NAV_PAUSED) stateStr = "PAUSED";
    else if (g_navState == NAV_DONE) stateStr = "DONE";
    else if (g_navState == NAV_ERROR) stateStr = "ERROR";

    char buf[64];
    snprintf(buf, sizeof(buf), "NAV,%s,%d,%d,%.2f",
             stateStr, g_navWpIndex, g_navWpTotal, g_navDistToWp);
    websocket_send(buf);
  }
}
