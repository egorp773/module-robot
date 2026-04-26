#ifndef GPS_H
#define GPS_H

#include <Arduino.h>

struct GpsData {
  double lat;           // degrees
  double lon;           // degrees
  float headMot;        // degrees, 0-360
  uint8_t fixType;      // 0=none, 3=3D, 4=GNSS+DR
  bool diffSoln;        // RTK flag
  uint32_t hAcc;        // horizontal accuracy, mm
  bool valid;
};

void gps_init();
void gps_update();
void rtcm_relay();

extern GpsData g_gpsData;
extern uint32_t g_lastGpsMs;

#endif // GPS_H
