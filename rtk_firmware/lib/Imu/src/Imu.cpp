// Imu.cpp - Adafruit_BNO08x wrapper. MIT.

#include "Imu.h"
#include "RtkConfig.h"
#include <Wire.h>

static float normalizeYawDeg(float yaw) {
    while (yaw < 0.0f) yaw += 360.0f;
    while (yaw >= 360.0f) yaw -= 360.0f;
    return yaw;
}

static float correctedYawDeg(float bnoYaw) {
    return normalizeYawDeg(IMU_ROT_YAW_OFFSET_DEG - bnoYaw - IMU_COMPASS_CORRECTION_DEG);
}

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
    // Geomagnetic Rotation Vector — абсолютный heading от магнитного севера без gyro-дрейфа.
    // Rotation Vector тоже абсолютный, но может не приходить/иметь плохую accuracy рядом с мотором.
    _bno.enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR, 20000);
    _bno.enableReport(SH2_ROTATION_VECTOR, 20000);
    _bno.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 20000);
    _bno.enableReport(SH2_GAME_ROTATION_VECTOR, 20000);  // fallback: только относительный heading
    _bno.enableReport(SH2_GYROSCOPE_CALIBRATED, 20000);  // 50 Hz — yaw rate для fusion
    _running = true;
    return true;
}

void Imu::loop() {
    if (!_running) return;
    if (!_bno.getSensorEvent(&_val)) {
        return;
    }
    if (_val.sensorId == SH2_MAGNETIC_FIELD_CALIBRATED) {
        _magCount++;
        float mx = _val.un.magneticField.x;
        float my = _val.un.magneticField.y;
        float mz = _val.un.magneticField.z;
        _magX = mx;
        _magY = my;
        _magZ = mz;
        float rawYaw = normalizeYawDeg(atan2f(my, mx) * 180.0f / M_PI);
        _rawYaw = rawYaw;
        // Знак: при вращении робота по часовой _yaw должен РАСТИ. Раньше было
        // IMU_YAW_OFFSET_DEG - raw, и на твоём BNO085 это давало падение на 25°
        // при повороте на +90° (275→250). Меняем на +, чтобы heading шёл в ту
        // же сторону, что и направление вращения.
        // Publish raw magnetic heading for startup/stationary correction. StateEstimator
        // only fuses absolute yaw while the rover is stationary, because motor current can
        // distort raw magnetic yaw by tens of degrees during motion.
        // Diagnostic only: raw magnetic yaw did not track a real 90 deg rotation.
    } else if (_val.sensorId == SH2_GEOMAGNETIC_ROTATION_VECTOR) {
        _geoCount++;
        float qx = _val.un.geoMagRotationVector.i;
        float qy = _val.un.geoMagRotationVector.j;
        float qz = _val.un.geoMagRotationVector.k;
        float qw = _val.un.geoMagRotationVector.real;
        float siny = 2.0f * (qw * qz + qx * qy);
        float cosy = 1.0f - 2.0f * (qy * qy + qz * qz);
        float yaw = normalizeYawDeg(atan2f(siny, cosy) * 180.0f / M_PI);
        _geoYaw = yaw;
        float accRad = _val.un.geoMagRotationVector.accuracy;
        // Отбрасываем магнитометр при accRad > 0.3 рад (~17°) — наводки моторов/силовых.
        // При таких accRad heading прыгает на десятки градусов, Stanley крутит робота.
        // Если уже держим валидный heading с прошлого пакета — оставляем его.
        if (true && accRad <= 0.5f) {
            _rawYaw = yaw;
            // Знак + (см. SH2_MAGNETIC_FIELD_CALIBRATED выше)
            _yaw = correctedYawDeg(yaw);
            _yawAccRad = accRad;
            _yawFromMag = true;
            _has = true;
            _lastMs = millis();
            _lastYawMs = _lastMs;
        }
    } else if (_val.sensorId == SH2_ROTATION_VECTOR) {
        _rotCount++;
        // Rotation Vector с магнитометром. Кватернион (i,j,k,real) описывает ориентацию
        // в мировой ENU-системе (X=East, Y=North, Z=Up). yaw — рыскание (heading),
        // знак: CCW = +. Нам нужен CW от севера → yaw стандартный (0=север, 90=восток).
        float qx = _val.un.rotationVector.i;
        float qy = _val.un.rotationVector.j;
        float qz = _val.un.rotationVector.k;
        float qw = _val.un.rotationVector.real;
        // Стандартный yaw из кватерниона (Tait-Bryan ZYX). Знак проверен в openmower.
        float siny = 2.0f * (qw * qz + qx * qy);
        float cosy = 1.0f - 2.0f * (qy * qy + qz * qz);
        float yaw = normalizeYawDeg(atan2f(siny, cosy) * 180.0f / M_PI);
        _rotYaw = yaw;
        float accRad = _val.un.rotationVector.accuracy;   // 1-sigma, rad
        if (accRad <= 0.5f) {   // пропускаем geomagnetic даже при наводках моторов ~28°, чтобы heading не терялся
            _rawYaw = yaw;
            _yaw = correctedYawDeg(yaw);
            _yawAccRad = accRad;
            _yawFromMag = true;
            _has = true;
            _lastMs = millis();
            _lastYawMs = _lastMs;
        }
    } else if (_val.sensorId == SH2_GAME_ROTATION_VECTOR) {
        _gameCount++;
        // Fallback без магнитометра. Дрейфует, но даёт "хоть какой-то" heading.
        // Используем ТОЛЬКО если rotationVector не дал валидного (accRad > 0.5).
        float qx = _val.un.gameRotationVector.i;
        float qy = _val.un.gameRotationVector.j;
        float qz = _val.un.gameRotationVector.k;
        float qw = _val.un.gameRotationVector.real;
        float siny = 2.0f * (qw * qz + qx * qy);
        float cosy = 1.0f - 2.0f * (qy * qy + qz * qz);
        float yaw = normalizeYawDeg(atan2f(siny, cosy) * 180.0f / M_PI);
        _gameYaw = yaw;
        if (!_yawFromMag) {     // не затираем валидный mag-heading game-версией
            _rawYaw = yaw;
            // Знак + (см. SH2_MAGNETIC_FIELD_CALIBRATED выше)
            _yaw = correctedYawDeg(yaw);
            _yawAccRad = 0.5f;  // плохая оценка
            _yawFromMag = false;
            _has = true;
            _lastMs = millis();
            _lastYawMs = _lastMs;
        }
    } else if (_val.sensorId == SH2_GYROSCOPE_CALIBRATED) {
        _gyroCount++;
        // gyroscope.z — угловая скорость вокруг вертикали (рад/с).
        // heading растёт по часовой => знак выверяется IMU_YAW_RATE_SIGN при калибровке.
        float wx = _val.un.gyroscope.x;
        float wy = _val.un.gyroscope.y;
        float wz = _val.un.gyroscope.z;
        uint32_t nowMs = millis();
        _gyroXDps = wx * 180.0f / M_PI;
        _gyroYDps = wy * 180.0f / M_PI;
        _gyroZDps = wz * 180.0f / M_PI;
        if (_lastGyroMs != 0) {
            float dt = (nowMs - _lastGyroMs) * 0.001f;
            if (dt > 0.0f && dt < 0.2f) {
                _gyroIntXDeg += _gyroXDps * dt;
                _gyroIntYDeg += _gyroYDps * dt;
                _gyroIntZDeg += _gyroZDps * dt;
            }
        }
        _lastGyroMs = nowMs;
        _yawRateDps = _gyroZDps * (float)IMU_YAW_RATE_SIGN;
        _lastMs = nowMs;
    }
    // calibration-чтение: Adafruit 1.2.5 не даёт удобного API; quality фиксируем
    // как 1 — IMU считается "рабочим", как только пришёл первый rot vector
    if (_has && _quality == 0) _quality = 1;
}
