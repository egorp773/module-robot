// Imu.h - Adafruit_BNO08x wrapper for stable robot heading.

#pragma once
#include <Arduino.h>
#include <Adafruit_BNO08x.h>

#include "ImuMath.h"

class Imu {
public:
    bool begin(uint8_t sdaPin, uint8_t sclPin);
    void loop();

    bool hasData() const { return _has; }

    // Robot heading in the project convention:
    // 0 = North, 90 = East, clockwise-positive.
    // If an absolute yaw is not ready yet, this returns the best available
    // relative orientation, but yawIsAbsolute()/headingState() tell the truth.
    float yawDeg() const { return _robotYawDeg; }
    float rawYawDeg() const { return _rawYawDeg; }
    float sourceYawDeg() const { return _sourceYawDeg; }
    float absYawDeg() const { return _absYawDeg; }
    float magNorm() const { return _magNorm; }
    float yawAccRad() const { return _yawAccRad; }
    ImuYawSource yawSource() const { return _yawSource; }
    ImuHeadingState headingState() const { return _headingState; }
    bool yawIsAbsolute() const { return ImuMath::yawStateIsAbsolute(_headingState); }
    bool yawAbsoluteValid() const { return yawIsAbsolute(); }
    bool yawFromMag() const { return yawIsAbsolute(); }
    uint32_t yawAgeMs(uint32_t nowMs) const {
        if (_lastYawMs == 0) return 0xFFFFFFFFu;
        return (nowMs >= _lastYawMs) ? (nowMs - _lastYawMs) : 0;
    }
    uint32_t ageMs(uint32_t nowMs) const {
        if (!_has) return 0xFFFFFFFFu;
        return (nowMs >= _lastMs) ? (nowMs - _lastMs) : 0;
    }
    bool fresh() const { return _has; }
    int  quality() const { return _quality; }
    bool calibrated() const { return _quality >= 2; }
    bool magDisturbed() const { return _magDisturbed; }

    float magX() const { return _magX; }
    float magY() const { return _magY; }
    float magZ() const { return _magZ; }
    float gameYawDeg() const { return _gameYawDeg; }
    float rotYawDeg() const { return _rotYawDeg; }
    float geoYawDeg() const { return _geoYawDeg; }
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
    float yawRateDps() const { return _yawRateDps; }
    uint32_t lastReportAgeMs(uint32_t nowMs) const { return ageMs(nowMs); }

    // Manual calibration helpers exposed to Serial/WebSocket.
    bool startCalibration(String* err = nullptr);
    bool saveCalibration(String* err = nullptr);
    bool clearCalibration(String* err = nullptr);
    bool tareYaw(bool persist, String* err = nullptr);
    bool clearTare(String* err = nullptr);
    bool clearDcdAndReset(String* err = nullptr);
    bool persistTare(String* err = nullptr);
    bool setDynamicCalibration(bool enabled, String* err = nullptr);
    bool saveDcdNow(String* err = nullptr);

    // For startup validation / diagnostics.
    float startupAbsDeltaDeg() const { return _startupAbsDeltaDeg; }
    uint32_t startupAbsStableMs(uint32_t nowMs) const {
        return _absCandidateSinceMs == 0 ? 0 : (nowMs - _absCandidateSinceMs);
    }

    void reset() {
        _has = false;
        _lastYawMs = 0;
        _lastRotYawMs = 0;
        _lastGeoYawMs = 0;
        _lastGoodRotYawMs = 0;
        _lastGoodGeoYawMs = 0;
        _absCandidateSinceMs = 0;
        _absCandidateSource = ImuYawSource::NONE;
        _startupAbsDeltaDeg = 0;
        _magDisturbed = false;
        _absoluteReady = false;
        _headingState = ImuHeadingState::IMU_NO_DATA;
        _yawSource = ImuYawSource::NONE;
    }

private:
    bool updateAbsoluteCandidate(ImuYawSource source, float sourceYawDeg, float robotYawDeg, float accRad, uint32_t nowMs);
    void updateHeadingState(uint32_t nowMs);
    void rememberMagNorm(float mx, float my, float mz);
    Adafruit_BNO08x _bno;
    sh2_SensorValue_t _val;
    bool _has = false;
    float _robotYawDeg = 0.0f;
    float _rawYawDeg = 0.0f;
    float _sourceYawDeg = 0.0f;
    float _absYawDeg = 0.0f;
    float _yawAccRad = 999.0f;
    ImuYawSource _yawSource = ImuYawSource::NONE;
    ImuHeadingState _headingState = ImuHeadingState::IMU_NO_DATA;
    bool _magDisturbed = false;
    float _magX = 0.0f;
    float _magY = 0.0f;
    float _magZ = 0.0f;
    float _magNorm = 0.0f;
    float _magNormBaseline = 0.0f;
    bool _magNormBaselineValid = false;
    float _gameYawDeg = 0.0f;
    float _rotYawDeg = 0.0f;
    float _geoYawDeg = 0.0f;
    float _gyroXDps = 0.0f;
    float _gyroYDps = 0.0f;
    float _gyroZDps = 0.0f;
    float _gyroIntXDeg = 0.0f;
    float _gyroIntYDeg = 0.0f;
    float _gyroIntZDeg = 0.0f;
    float _yawRateDps = 0.0f;
    float _startupAbsDeltaDeg = 0.0f;
    uint32_t _lastMs = 0;
    uint32_t _lastYawMs = 0;
    uint32_t _lastGyroMs = 0;
    uint32_t _lastRotYawMs = 0;
    uint32_t _lastGeoYawMs = 0;
    uint32_t _lastGoodRotYawMs = 0;
    uint32_t _lastGoodGeoYawMs = 0;
    uint32_t _absCandidateSinceMs = 0;
    ImuYawSource _absCandidateSource = ImuYawSource::NONE;
    float _absCandidateYawDeg = 0.0f;
    bool _absoluteReady = false;
    uint32_t _magCount = 0;
    uint32_t _geoCount = 0;
    uint32_t _rotCount = 0;
    uint32_t _gameCount = 0;
    uint32_t _gyroCount = 0;
    int _quality = 0;
    bool _running = false;
};
