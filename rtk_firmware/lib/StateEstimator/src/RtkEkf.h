// RtkEkf.h — 5-state Extended Kalman Filter for RTK platform.
// State: [x, y, heading, v, omega].
// x = East, y = North, heading radians: 0 = North, pi/2 = East, clockwise-positive.
// Predict: odometry (v, omega from hoverboard feedback) + IMU gyro (heading).
// Update:  RTK fix (x, y) + GPS motion heading (heading, only when moving).
//
// Covariance matrix P 5x5 stored as flat array (5x5 = 25 floats) — RAM-cheap,
// matrix multiply hand-written, no dependencies.
//
// Algorithm source: general idea from ROS robot_localization (EkfRos), adapted for ESP32.
// MIT.

#pragma once
#include <Arduino.h>

// State dimension. _P_ is renamed to avoid macro clash with ctype.h _P.
#define RTKEKF_N 5

class RtkEkf {
public:
    // Process noise (tuneable). public so they can be overridden by app if needed.
    static float kQ_x;
    static float kQ_y;
    static float kQ_heading;
    static float kQ_v;
    static float kQ_omega;
    // Mahalanobis gate for updatePosition: chi2(0.999, 2) ~ 3 sigma
    static float kMahalanobisGate;

    void begin();

    // Predict step: odometry + IMU. dt in seconds.
    void predict(float dt, float v_mps, float omega_radps, float yawRateRadps);

    // Update step: RTK fix in local meters. cov_xy - variance (m^2), 0 if unknown.
    bool updatePosition(float x, float y, float cov_xy, bool reliableFix);

    // Update step: heading (GPS motion heading) - only when moving.
    bool updateHeading(float headingRad, float cov_rad2, bool reliable);

    // Re-init on origin change.
    void reset(float x0, float y0, float headingRad);

    // Read-only getters.
    float x()       const;
    float y()       const;
    float heading() const;
    float v()       const;
    float omega()   const;
    float covPos()  const;
    float covHead() const;

private:
    // State
    float _x;
    float _y;
    float _h;
    float _v;
    float _w;

    // Covariance 5x5 in row-major: _Pcov[i*N + j]. Initialized in begin().
    float _Pcov[25];

    // helpers
    static void matSet(float* A, int n, int i, int j, float v);
    static void matMul(const float* A, const float* B, float* C, int m, int k, int n);
    static void matAdd(const float* A, const float* B, float* C);
    static void matTranspose(const float* A, float* C, int m, int n);
};
