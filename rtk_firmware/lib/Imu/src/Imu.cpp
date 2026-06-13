// Imu.cpp - Adafruit_BNO08x wrapper. MIT.

#include "Imu.h"
#include "RtkConfig.h"
#include <Wire.h>

bool Imu::begin(uint8_t sdaPin, uint8_t sclPin) {
    Wire.begin(sdaPin, sclPin, 400000);
    // I2C-сканер для диагностики
    Serial.print("[IMU] I2C scan: ");
    bool found = false;
    for (uint8_t addr = 0x08; addr < 0x78; ++addr) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("0x%02X ", addr);
            found = true;
        }
    }
    if (!found) Serial.print("none");
    Serial.println();

    // BNO08x: основной адрес 0x4A. Если не нашёлся — пробуем 0x4B
    if (_bno.begin_I2C(0x4A)) {
        Serial.println("[IMU] BNO08x at 0x4A");
    } else if (_bno.begin_I2C(0x4B)) {
        Serial.println("[IMU] BNO08x at 0x4B");
    } else {
        return false;
    }
    _bno.enableReport(SH2_GAME_ROTATION_VECTOR, 20000);  // 50 Hz — абсолютный yaw
    _bno.enableReport(SH2_GYROSCOPE_CALIBRATED, 20000);  // 50 Hz — yaw rate для fusion
    _running = true;
    return true;
}

void Imu::loop() {
    if (!_running) return;
    if (!_bno.getSensorEvent(&_val)) {
        return;
    }
    if (_val.sensorId == SH2_GAME_ROTATION_VECTOR) {
        float qx = _val.un.gameRotationVector.i;
        float qy = _val.un.gameRotationVector.j;
        float qz = _val.un.gameRotationVector.k;
        float qw = _val.un.gameRotationVector.real;
        // yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
        float siny = 2.0f * (qw * qz + qx * qy);
        float cosy = 1.0f - 2.0f * (qy * qy + qz * qz);
        float yaw = atan2f(siny, cosy) * 180.0f / M_PI;
        if (yaw < 0) yaw += 360.0f;
        _yaw = yaw;
        _has = true;
        _lastMs = millis();
    } else if (_val.sensorId == SH2_GYROSCOPE_CALIBRATED) {
        // gyroscope.z — угловая скорость вокруг вертикали (рад/с).
        // heading растёт по часовой => знак выверяется IMU_YAW_RATE_SIGN при калибровке.
        float wz = _val.un.gyroscope.z;            // rad/s
        _yawRateDps = wz * 180.0f / M_PI * (float)IMU_YAW_RATE_SIGN;
        _lastMs = millis();
    }
    // calibration-чтение: Adafruit 1.2.5 не даёт удобного API; quality фиксируем
    // как 1 — IMU считается "рабочим", как только пришёл первый game rot vector
    if (_has && _quality == 0) _quality = 1;
}
