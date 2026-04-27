#include "config.h"
#include "motors.h"
#include "gps.h"
#include "imu.h"
#include "nav.h"
#include "websocket.h"
#include "sound.h"
#include "telemetry.h"

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("\n=== Robot Firmware v2.0 ===");
  Serial.println("GPS + IMU + Autonomous Navigation");

  // Relays
  pinMode(PIN_RELAY_ATTACH, OUTPUT);
  pinMode(PIN_RELAY_MOUNT, OUTPUT);
  setAttachment(false);
  setMount(false);

  // Initialize modules
  motors_init();
  gps_init();
  imu_init();
  sound_init();
  websocket_init();
  nav_init();

  Serial.println("=== READY ===");
}

void loop() {
  websocket_cleanup();

  gps_update();
  imu_update();
  rtcm_relay();

  nav_update();

  motors_check_failsafe();
  motors_update_ramp();
  motors_send();
  motors_receive_feedback();

  telemetry_update();
}
