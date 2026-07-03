// RtkEkf.cpp — implementation of 5-state EKF. MIT.

#include "RtkEkf.h"
#include <math.h>

// Static constants (defined here, declared in .h)
float RtkEkf::kQ_x        = 0.05f;   // m^2 / s  (position noise from v-noise)
float RtkEkf::kQ_y        = 0.05f;
float RtkEkf::kQ_heading  = 0.02f;   // rad^2 / s
float RtkEkf::kQ_v        = 0.10f;
float RtkEkf::kQ_omega    = 0.50f;
float RtkEkf::kMahalanobisGate = 9.21f;

// Getters (out-of-line for C++11 compatibility)
float RtkEkf::x()       const { return _x; }
float RtkEkf::y()       const { return _y; }
float RtkEkf::heading() const { return _h; }
float RtkEkf::v()       const { return _v; }
float RtkEkf::omega()   const { return _w; }
float RtkEkf::covPos()  const { return _Pcov[0]; }
float RtkEkf::covHead() const { return _Pcov[2*RTKEKF_N+2]; }

void RtkEkf::matSet(float* A, int n, int i, int j, float v) { A[i*n+j] = v; }

void RtkEkf::begin() {
    _x = 0; _y = 0; _h = 0; _v = 0; _w = 0;
    for (int i = 0; i < 25; i++) _Pcov[i] = 0;
    _Pcov[0]  = 1e6f;   // x variance
    _Pcov[6]  = 1e6f;   // y variance
    _Pcov[12] = 0.5f;   // heading variance
    _Pcov[18] = 0.5f;   // v variance
    _Pcov[24] = 1.0f;   // omega variance
}

void RtkEkf::reset(float x0, float y0, float headingRad) {
    _x = x0; _y = y0; _h = headingRad;
    _v = 0; _w = 0;
    for (int i = 0; i < 25; i++) _Pcov[i] = 0;
    _Pcov[0]  = 0.05f;   // after origin change we know position to ~20 cm
    _Pcov[6]  = 0.05f;
    _Pcov[12] = 0.5f;
    _Pcov[18] = 0.5f;
    _Pcov[24] = 1.0f;
}

void RtkEkf::matMul(const float* A, const float* B, float* C, int m, int k, int n) {
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            float s = 0;
            for (int p = 0; p < k; p++) s += A[i*k+p] * B[p*n+j];
            C[i*n+j] = s;
        }
    }
}

void RtkEkf::matAdd(const float* A, const float* B, float* C) {
    for (int i = 0; i < 25; i++) C[i] = A[i] + B[i];
}

void RtkEkf::matTranspose(const float* A, float* C, int m, int n) {
    for (int i = 0; i < m; i++)
        for (int j = 0; j < n; j++)
            C[j*m+i] = A[i*n+j];
}

void RtkEkf::predict(float dt, float v_mps, float omega_radps, float yawRateRadps) {
    if (dt <= 0) return;
    if (dt > 0.5f) dt = 0.5f;   // guard against long pauses (boot, stall)

    // === State update (motion model) ===
    // x' = x + v * cos(h) * dt
    // y' = y + v * sin(h) * dt
    // h' = h + yawRate * dt  (IMU gyro primary)
    // v' = v
    // w' = w
    float ch = cosf(_h);
    float sh = sinf(_h);
    _x += v_mps * ch * dt;
    _y += v_mps * sh * dt;
    _h += yawRateRadps * dt;
    // wrap heading to [-pi, pi]
    while (_h >  3.14159265f) _h -= 6.28318530f;
    while (_h < -3.14159265f) _h += 6.28318530f;
    _v = v_mps;
    _w = omega_radps;

    // === Jacobian F (5x5) ===
    float F[25] = {0};
    F[0]  = 1;
    F[6]  = 1;
    F[12] = 1;
    F[18] = 1;
    F[24] = 1;
    matSet(F, 5, 0, 2, -v_mps * sh * dt);   // dx/dh
    matSet(F, 5, 1, 2,  v_mps * ch * dt);   // dy/dh
    matSet(F, 5, 0, 3,  ch * dt);           // dx/dv
    matSet(F, 5, 1, 3,  sh * dt);           // dy/dv
    matSet(F, 5, 2, 4,  dt);                // dh/dw

    // === Process noise Q (diagonal) ===
    float Q[25] = {0};
    Q[0]  = kQ_x       * dt;
    Q[6]  = kQ_y       * dt;
    Q[12] = kQ_heading * dt;
    Q[18] = kQ_v       * dt;
    Q[24] = kQ_omega   * dt;

    // === P' = F * P * F^T + Q ===
    float FP[25];
    matMul(F, _Pcov, FP, 5, 5, 5);
    float Ft[25];
    matTranspose(F, Ft, 5, 5);
    float FPFt[25];
    matMul(FP, Ft, FPFt, 5, 5, 5);
    matAdd(FPFt, Q, _Pcov);

    // Symmetrize
    for (int i = 0; i < 5; i++) {
        for (int j = i+1; j < 5; j++) {
            float s = 0.5f * (_Pcov[i*5+j] + _Pcov[j*5+i]);
            _Pcov[i*5+j] = s;
            _Pcov[j*5+i] = s;
        }
    }
}

bool RtkEkf::updatePosition(float zx, float zy, float cov_xy, bool reliableFix) {
    if (!reliableFix) return false;
    if (cov_xy <= 0) cov_xy = 0.0064f;   // default 8cm -> var 6.4e-3 m^2
    if (cov_xy < 0.0064f) cov_xy = 0.0064f;

    // === Mahalanobis gate ===
    float dx = zx - _x;
    float dy = zy - _y;
    float Pxx = _Pcov[0];
    float Pyy = _Pcov[6];
    float Pxy = _Pcov[1];
    float Sg00 = Pxx + cov_xy;
    float Sg11 = Pyy + cov_xy;
    float det = Sg00 * Sg11 - Pxy * Pxy;
    if (det <= 1e-12f) return false;
    float invDet = 1.0f / det;
    float mahal = (Sg11 * dx * dx - 2.0f * Pxy * dx * dy + Sg00 * dy * dy) * invDet;
    if (mahal > kMahalanobisGate) return false;   // outlier

    // === S = H*P*H^T + R (2x2). R = diag(cov_xy, cov_xy) ===
    float S00 = Pxx + cov_xy;
    float S11 = Pyy + cov_xy;
    float S01 = Pxy;
    float Sdet = S00 * S11 - S01 * S01;
    if (Sdet <= 1e-12f) return false;
    float Sinv00 =  S11 / Sdet;
    float Sinv11 =  S00 / Sdet;
    float Sinv01 = -S01 / Sdet;

    // === K = P*H^T*S^-1 (5x2). P*H^T = first 2 columns of P. ===
    float K[10];  // 5x2
    for (int i = 0; i < 5; i++) {
        K[i*2 + 0] = _Pcov[i*5 + 0] * Sinv00 + _Pcov[i*5 + 1] * Sinv01;
        K[i*2 + 1] = _Pcov[i*5 + 0] * Sinv01 + _Pcov[i*5 + 1] * Sinv11;
    }

    // === x' = x + K * y ===
    for (int i = 0; i < 5; i++) {
        if (i == 0) _x += K[i*2+0] * dx + K[i*2+1] * dy;
        else if (i == 1) _y += K[i*2+0] * dx + K[i*2+1] * dy;
        else if (i == 2) _h += K[i*2+0] * dx + K[i*2+1] * dy;
        else if (i == 3) _v += K[i*2+0] * dx + K[i*2+1] * dy;
        else if (i == 4) _w += K[i*2+0] * dx + K[i*2+1] * dy;
    }
    if (_h >  3.14159265f) _h -= 6.28318530f;
    if (_h < -3.14159265f) _h += 6.28318530f;

    // === P' = (I - K*H) * P. H = first 2 columns of identity. ===
    float IKH[25];
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            float v = 0;
            if (j == 0) v = K[i*2+0];
            else if (j == 1) v = K[i*2+1];
            float orig = (i == j) ? 1.0f : 0.0f;
            IKH[i*5+j] = orig - v;
        }
    }
    float newP[25];
    matMul(IKH, _Pcov, newP, 5, 5, 5);
    for (int i = 0; i < 25; i++) _Pcov[i] = newP[i];

    // Symmetrize
    for (int i = 0; i < 5; i++) {
        for (int j = i+1; j < 5; j++) {
            float s = 0.5f * (_Pcov[i*5+j] + _Pcov[j*5+i]);
            _Pcov[i*5+j] = s;
            _Pcov[j*5+i] = s;
        }
    }

    // Guard against degeneracy
    if (_Pcov[0]  < 1e-6f) _Pcov[0]  = 1e-6f;
    if (_Pcov[6]  < 1e-6f) _Pcov[6]  = 1e-6f;
    if (_Pcov[12] < 1e-3f) _Pcov[12] = 1e-3f;
    if (_Pcov[18] < 1e-3f) _Pcov[18] = 1e-3f;
    if (_Pcov[24] < 1e-2f) _Pcov[24] = 1e-2f;

    return true;
}

bool RtkEkf::updateHeading(float headingRad, float cov_rad2, bool reliable) {
    if (!reliable) return false;
    if (cov_rad2 <= 0) cov_rad2 = 0.01f;   // ~5 deg default

    // Innovation (with wrap-around in radians)
    float diff = headingRad - _h;
    while (diff >  3.14159265f) diff -= 6.28318530f;
    while (diff < -3.14159265f) diff += 6.28318530f;

    // Mahalanobis gate (1D)
    float S = _Pcov[12] + cov_rad2;
    if (S <= 1e-12f) return false;
    float mahal = diff * diff / S;
    if (mahal > 9.0f) return false;

    // K = P[2,:] / S (5x1, third row of P divided by S)
    float K[5];
    for (int i = 0; i < 5; i++) K[i] = _Pcov[i*5 + 2] / S;

    // x' = x + K * diff
    _x += K[0] * diff;
    _y += K[1] * diff;
    _h += K[2] * diff;
    _v += K[3] * diff;
    _w += K[4] * diff;
    if (_h >  3.14159265f) _h -= 6.28318530f;
    if (_h < -3.14159265f) _h += 6.28318530f;

    // P' = (I - K*H) * P. H = [0 0 1 0 0]
    float newP[25];
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            float v = (j == 2) ? K[i] : 0.0f;
            float orig = (i == j) ? 1.0f : 0.0f;
            newP[i*5+j] = (orig - v) * _Pcov[i*5+j];
        }
    }
    for (int i = 0; i < 25; i++) _Pcov[i] = newP[i];

    // Symmetrize
    for (int i = 0; i < 5; i++) {
        for (int j = i+1; j < 5; j++) {
            float s = 0.5f * (_Pcov[i*5+j] + _Pcov[j*5+i]);
            _Pcov[i*5+j] = s;
            _Pcov[j*5+i] = s;
        }
    }

    if (_Pcov[12] < 1e-3f) _Pcov[12] = 1e-3f;
    return true;
}
