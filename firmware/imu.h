#ifndef IMU_H
#define IMU_H

#include <Arduino.h>

void imu_init();
void imu_update();

extern float g_imuYaw;      // degrees, 0-360
extern bool g_imuValid;
extern uint32_t g_lastImuMs;

#endif // IMU_H
