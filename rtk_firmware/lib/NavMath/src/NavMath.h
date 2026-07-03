#pragma once

#include <math.h>

namespace NavMath {

constexpr double kPi = 3.14159265358979323846;

inline float normalizeDeg360(float degrees) {
    while (degrees < 0.0f) degrees += 360.0f;
    while (degrees >= 360.0f) degrees -= 360.0f;
    return degrees;
}

inline float wrapDeg180(float degrees) {
    while (degrees > 180.0f) degrees -= 360.0f;
    while (degrees < -180.0f) degrees += 360.0f;
    return degrees;
}

// x = East, y = North; compass heading is clockwise from North.
inline float targetHeadingDeg(float dxEast, float dyNorth) {
    return normalizeDeg360(atan2f(dxEast, dyNorth) * 180.0f /
                           static_cast<float>(kPi));
}

inline void forwardOffset(float distanceM, float headingDeg,
                          float& dxEast, float& dyNorth) {
    const float headingRad =
        headingDeg * static_cast<float>(kPi) / 180.0f;
    dxEast = distanceM * sinf(headingRad);
    dyNorth = distanceM * cosf(headingRad);
}

inline void llaToLocalMeters(double lat, double lon,
                             double originLat, double originLon,
                             float& xEast, float& yNorth) {
    const double phi = originLat * kPi / 180.0;
    const double metersPerDegreeLat =
        111132.92 - 559.82 * cos(2.0 * phi) +
        1.175 * cos(4.0 * phi) - 0.0023 * cos(6.0 * phi);
    const double metersPerDegreeLon =
        111412.84 * cos(phi) - 93.5 * cos(3.0 * phi) +
        0.118 * cos(5.0 * phi);
    xEast = static_cast<float>((lon - originLon) * metersPerDegreeLon);
    yNorth = static_cast<float>((lat - originLat) * metersPerDegreeLat);
}

}  // namespace NavMath
