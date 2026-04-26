#include "imu.h"
#include "config.h"
#include <Adafruit_BNO08x.h>

float g_imuYaw = 0.0f;
bool g_imuValid = false;
uint32_t g_lastImuMs = 0;

static Adafruit_BNO08x bno08x;
static sh2_SensorValue_t sensorValue;

void imu_init() {
  Wire.begin(PIN_IMU_SDA, PIN_IMU_SCL);
  Serial.printf("IMU: I2C SDA=%d SCL=%d\n", PIN_IMU_SDA, PIN_IMU_SCL);

  if (!bno08x.begin_I2C()) {
    Serial.println("IMU: BNO085 not found!");
    g_imuValid = false;
    return;
  }

  Serial.println("IMU: BNO085 found");

  if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 20000)) { // 50 Hz (20ms)
    Serial.println("IMU: Failed to enable rotation vector");
    g_imuValid = false;
    return;
  }

  Serial.println("IMU: Rotation vector enabled at 50 Hz");
  g_imuValid = true;
}

void imu_update() {
  if (!g_imuValid) return;

  if (bno08x.wasReset()) {
    Serial.println("IMU: Sensor was reset");
    if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 20000)) {
      Serial.println("IMU: Failed to re-enable rotation vector");
      g_imuValid = false;
      return;
    }
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      float qw = sensorValue.un.rotationVector.real;
      float qx = sensorValue.un.rotationVector.i;
      float qy = sensorValue.un.rotationVector.j;
      float qz = sensorValue.un.rotationVector.k;

      // Extract yaw from quaternion
      float yaw = atan2(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
      yaw = yaw * 180.0f / PI;

      // Normalize to 0-360
      if (yaw < 0) yaw += 360.0f;

      g_imuYaw = yaw;
      g_lastImuMs = millis();
    }
  }
}
