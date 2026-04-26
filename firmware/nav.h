#ifndef NAV_H
#define NAV_H

#include <Arduino.h>

enum NavState {
  NAV_IDLE,
  NAV_RUNNING,
  NAV_PAUSED,
  NAV_DONE,
  NAV_ERROR
};

struct Waypoint {
  double lat;
  double lon;
};

void nav_init();
void nav_update();
void nav_start();
void nav_pause();
void nav_resume();
void nav_stop();
void nav_clear_route();
bool nav_add_waypoint(double lat, double lon);

extern NavState g_navState;
extern int g_navWpIndex;
extern int g_navWpTotal;
extern float g_navDistToWp;
extern Waypoint g_waypoints[MAX_WAYPOINTS];

// Motor targets set by nav (used by motors.cpp)
extern volatile int16_t g_targetLeft;
extern volatile int16_t g_targetRight;

#endif // NAV_H
