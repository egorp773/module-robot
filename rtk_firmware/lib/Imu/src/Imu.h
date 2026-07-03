// Imu.h - Adafruit_BNO08x wrapper. Используем SH2_ROTATION_VECTOR (gyro+accel+mag)
// как основной источник абсолютного heading от магнитного севера. MIT.

#pragma once
#include <Arduino.h>
#include <Adafruit_BNO08x.h>

class Imu {
public:
    bool begin(uint8_t sdaPin, uint8_t sclPin);
    void loop();

    bool hasData() const { return _has; }
    // Абсолютный heading 0..360° CW от магнитного севера (как GPS headMot).
    // Если магнитометр не откалиброван — fallback на gameRotationVector (дрейфует).
    float yawDeg() const { return _yaw; }
    float rawYawDeg() const { return _rawYaw; }
    // Точность heading, радианы (BNO08x estimate). Чем меньше — тем лучше.
    // ~5° (0.087 rad) — посредственно. <2° (0.035 rad) — хорошо.
    float yawAccRad() const { return _yawAccRad; }
    uint32_t yawAgeMs(uint32_t nowMs) const {
        if (_lastYawMs == 0) return 0xFFFFFFFFu;
        return (nowMs >= _lastYawMs) ? (nowMs - _lastYawMs) : 0;
    }
    // true если heading от магнитометра (rotationVector) и точность < ~10°.
    bool yawFromMag() const { return _yawFromMag; }
    float magX() const { return _magX; }
    float magY() const { return _magY; }
    float magZ() const { return _magZ; }
    float gameYawDeg() const { return _gameYaw; }
    float rotYawDeg() const { return _rotYaw; }
    float geoYawDeg() const { return _geoYaw; }
    float gyroXDps() const { return _gyroXDps; }
    float gyroYDps() const { return _gyroYDps; }
    float gyroZDps() const { return _gyroZDps; }
    float gyroIntXDeg() const { return _gyroIntXDeg; }
    float gyroIntYDeg() const { return _gyroIntYDeg; }
    float gyroIntZDeg() const { return _gyroIntZDeg; }
    uint32_t magCount() const { return _magCount; }
    uint32_t geoCount() const { return _geoCount; }
    uint32_t rotCount() const { return _rotCount; }
    uint32_t gameCount() const { return _gameCount; }
    uint32_t gyroCount() const { return _gyroCount; }
    // угловая скорость рысканья (yaw rate), град/с. Знак: + по часовой (как heading).
    float yawRateDps() const { return _yawRateDps; }
    uint32_t ageMs(uint32_t nowMs) const {
        if (!_has) return 0xFFFFFFFFu;
        return (nowMs >= _lastMs) ? (nowMs - _lastMs) : 0;  // clamp mid-loop underflow
    }
    bool fresh() const { return _has; }
    int  quality() const { return _quality; }   // 0..3
    bool calibrated() const { return _quality >= 2; }

    // для IMU reset через app
    void reset() { _has = false; _yawFromMag = false; _lastYawMs = 0; }

private:
    Adafruit_BNO08x _bno;
    sh2_SensorValue_t _val;
    bool _has = false;
    float _yaw = 0;
    float _rawYaw = 0;
    float _yawAccRad = 999.0f;
    bool  _yawFromMag = false;
    float _magX = 0;
    float _magY = 0;
    float _magZ = 0;
    float _gameYaw = 0;
    float _rotYaw = 0;
    float _geoYaw = 0;
    float _gyroXDps = 0;
    float _gyroYDps = 0;
    float _gyroZDps = 0;
    float _gyroIntXDeg = 0;
    float _gyroIntYDeg = 0;
    float _gyroIntZDeg = 0;
    float _yawRateDps = 0;
    uint32_t _lastMs = 0;
    uint32_t _lastYawMs = 0;
    uint32_t _lastGyroMs = 0;
    uint32_t _magCount = 0;
    uint32_t _geoCount = 0;
    uint32_t _rotCount = 0;
    uint32_t _gameCount = 0;
    uint32_t _gyroCount = 0;
    int _quality = 0;
    bool _running = false;
};
