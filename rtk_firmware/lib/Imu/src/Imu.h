// Imu.h - Adafruit_BNO08x wrapper, Game Rotation Vector 50 Hz. MIT.

#pragma once
#include <Arduino.h>
#include <Adafruit_BNO08x.h>

class Imu {
public:
    bool begin(uint8_t sdaPin, uint8_t sclPin);
    void loop();

    bool hasData() const { return _has; }
    float yawDeg() const { return _yaw; }
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
    void reset() { _has = false; }

private:
    Adafruit_BNO08x _bno;
    sh2_SensorValue_t _val;
    bool _has = false;
    float _yaw = 0;
    float _yawRateDps = 0;
    uint32_t _lastMs = 0;
    int _quality = 0;
    bool _running = false;
};
