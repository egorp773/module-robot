// ImuMath.h - pure IMU heading helpers shared by firmware and tests.

#pragma once

#include <Arduino.h>

enum class ImuYawSource : uint8_t {
    NONE = 0,
    ROTATION_VECTOR,
    GEOMAGNETIC_ROTATION_VECTOR,
    GAME_ROTATION_VECTOR,
    GYROSCOPE_CALIBRATED,
};

enum class ImuHeadingState : uint8_t {
    IMU_NO_DATA = 0,
    IMU_RELATIVE_ONLY,
    IMU_ABSOLUTE_UNCALIBRATED,
    IMU_ABSOLUTE_OK,
    IMU_MAG_DISTURBED,
    IMU_STALE,
};

namespace ImuMath {

inline float normalizeDeg360(float deg) {
    while (deg < 0.0f) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;
    return deg;
}

inline float wrapDeg180(float deg) {
    deg = normalizeDeg360(deg);
    if (deg > 180.0f) deg -= 360.0f;
    return deg;
}

// Convert a raw BNO heading into the robot heading convention:
// 0 = North, 90 = East, clockwise-positive.
//
// Formula:
//   heading = normalize(sign * normalize(rawYawDeg) + mountOffsetDeg + compassAdjustDeg)
//
// sign = +1 keeps the sensor heading direction.
// sign = -1 flips the sensor direction into robot heading convention.
inline float imuRawToRobotHeadingDeg(
    float rawYawDeg,
    float headingSign,
    float mountOffsetDeg,
    float compassAdjustDeg) {
    const float raw = normalizeDeg360(rawYawDeg);
    return normalizeDeg360(headingSign * raw + mountOffsetDeg + compassAdjustDeg);
}

inline float headingCorrectionDeg(float currentHeadingDeg, float trueHeadingDeg) {
    return wrapDeg180(trueHeadingDeg - currentHeadingDeg);
}

inline float applyHeadingCorrectionDeg(float headingDeg, float correctionDeg) {
    return normalizeDeg360(headingDeg + correctionDeg);
}

inline float rtkForwardHeadingDeg(float dxEast, float dyNorth) {
    return normalizeDeg360(atan2f(dxEast, dyNorth) * 180.0f / M_PI);
}

inline bool yawStateIsAbsolute(ImuHeadingState state) {
    return state == ImuHeadingState::IMU_ABSOLUTE_OK;
}

inline bool yawSourceIsAbsolute(ImuYawSource source) {
    return source == ImuYawSource::ROTATION_VECTOR ||
           source == ImuYawSource::GEOMAGNETIC_ROTATION_VECTOR;
}

inline const char* yawSourceName(ImuYawSource source) {
    switch (source) {
        case ImuYawSource::ROTATION_VECTOR: return "ROTATION_VECTOR";
        case ImuYawSource::GEOMAGNETIC_ROTATION_VECTOR: return "GEOMAGNETIC_ROTATION_VECTOR";
        case ImuYawSource::GAME_ROTATION_VECTOR: return "GAME_ROTATION_VECTOR";
        case ImuYawSource::GYROSCOPE_CALIBRATED: return "GYROSCOPE_CALIBRATED";
        default: return "NONE";
    }
}

inline const char* headingStateName(ImuHeadingState state) {
    switch (state) {
        case ImuHeadingState::IMU_RELATIVE_ONLY: return "RELATIVE_ONLY";
        case ImuHeadingState::IMU_ABSOLUTE_UNCALIBRATED: return "ABSOLUTE_UNCALIBRATED";
        case ImuHeadingState::IMU_ABSOLUTE_OK: return "ABSOLUTE_OK";
        case ImuHeadingState::IMU_MAG_DISTURBED: return "MAG_DISTURBED";
        case ImuHeadingState::IMU_STALE: return "STALE";
        default: return "NO_DATA";
    }
}

inline bool yawStateIsOperational(ImuHeadingState state) {
    return state == ImuHeadingState::IMU_ABSOLUTE_OK ||
           state == ImuHeadingState::IMU_RELATIVE_ONLY ||
           state == ImuHeadingState::IMU_ABSOLUTE_UNCALIBRATED;
}

inline bool canUseAbsoluteYawForNav(
    ImuHeadingState state,
    bool absYawValid,
    float absYawAccRad,
    uint32_t yawAgeMs,
    uint32_t maxAgeMs,
    float maxAccRad = 0.5f) {
    return absYawValid &&
           state == ImuHeadingState::IMU_ABSOLUTE_OK &&
           yawAgeMs <= maxAgeMs &&
           absYawAccRad <= maxAccRad;
}

}  // namespace ImuMath
