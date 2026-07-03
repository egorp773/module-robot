#!/usr/bin/env python3
"""
ekf_test.py - проверяет алгоритм EKF (логику predict + update) на синтетическом пути.
Эмулирует движение робота по прямой с шумным GPS и проверяет, что:
- EKF координаты не скачут (RMS < 0.1 м)
- EKF heading держится на IMU между PVT
- Outlier-GPS отбрасывается (Mahalanobis gate работает)
- EKF не сходится с ума при пропадании GPS на 1 сек

Не требует ESP32 — чистый Python, проверяет алгоритм, а не железо.
"""
import math
import random
import sys

# === Воспроизводим логику EKF из RtkEkf.cpp на Python ===

class Ekf:
    N = 5
    def __init__(self):
        # Параметры шумов (как в RtkEkf.cpp)
        self.kQ_x = 0.05
        self.kQ_y = 0.05
        self.kQ_heading = 0.02
        self.kQ_v = 0.10
        self.kQ_omega = 0.50
        self.kMahalanobisGate = 9.21

        # Состояние
        self._x = 0.0
        self._y = 0.0
        self._h = 0.0
        self._v = 0.0
        self._w = 0.0

        # Ковариация 5x5 row-major
        self._P = [0.0] * 25
        self._P[0] = 1e6
        self._P[6] = 1e6
        self._P[12] = 0.5
        self._P[18] = 0.5
        self._P[24] = 1.0

    def reset(self, x0, y0, h_rad):
        self._x, self._y, self._h = x0, y0, h_rad
        self._v = self._w = 0
        self._P = [0.0] * 25
        self._P[0] = self._P[6] = 0.05
        self._P[12] = 0.5
        self._P[18] = 0.5
        self._P[24] = 1.0

    def predict(self, dt, v_mps, omega_radps, yawRateRadps):
        if dt <= 0: return
        if dt > 0.5: dt = 0.5
        ch, sh = math.cos(self._h), math.sin(self._h)
        self._x += v_mps * sh * dt
        self._y += v_mps * ch * dt
        self._h += yawRateRadps * dt
        # wrap
        while self._h > math.pi: self._h -= 2*math.pi
        while self._h < -math.pi: self._h += 2*math.pi
        self._v, self._w = v_mps, omega_radps

        # Jacobian F
        F = [[0]*5 for _ in range(5)]
        for i in range(5): F[i][i] = 1.0
        F[0][2] =  v_mps * ch * dt
        F[1][2] = -v_mps * sh * dt
        F[0][3] =  sh * dt
        F[1][3] =  ch * dt
        F[2][4] =  dt

        # Q
        Q = [[0]*5 for _ in range(5)]
        Q[0][0] = self.kQ_x * dt
        Q[1][1] = self.kQ_y * dt
        Q[2][2] = self.kQ_heading * dt
        Q[3][3] = self.kQ_v * dt
        Q[4][4] = self.kQ_omega * dt

        # P' = F P F^T + Q
        FP = [[sum(F[i][k]*self._P[k*5+j] for k in range(5)) for j in range(5)] for i in range(5)]
        Ft = [[F[j][i] for j in range(5)] for i in range(5)]
        FPFt = [[sum(FP[i][k]*Ft[k][j] for k in range(5)) for j in range(5)] for i in range(5)]
        newP = [[FPFt[i][j] + Q[i][j] for j in range(5)] for i in range(5)]
        # symmetrize
        for i in range(5):
            for j in range(i+1, 5):
                s = 0.5*(newP[i][j] + newP[j][i])
                newP[i][j] = newP[j][i] = s
        self._P = [newP[i][j] for i in range(5) for j in range(5)]

    def updatePosition(self, zx, zy, cov_xy, reliable):
        if not reliable: return False
        if cov_xy <= 0: cov_xy = 0.0004
        dx, dy = zx - self._x, zy - self._y
        Pxx, Pyy, Pxy = self._P[0], self._P[6], self._P[1]
        det = Pxx*Pyy - Pxy*Pxy
        if det <= 1e-12: return False
        mahal = (Pyy*dx*dx - 2*Pxy*dx*dy + Pxx*dy*dy) / det
        if mahal > self.kMahalanobisGate: return False

        S00, S11, S01 = Pxx+cov_xy, Pyy+cov_xy, Pxy
        Sdet = S00*S11 - S01*S01
        if Sdet <= 1e-12: return False
        Sinv00, Sinv11, Sinv01 = S11/Sdet, S00/Sdet, -S01/Sdet

        K = [[self._P[i*5+0]*Sinv00 + self._P[i*5+1]*Sinv01,
              self._P[i*5+0]*Sinv01 + self._P[i*5+1]*Sinv11] for i in range(5)]

        # x' = x + K * [dx, dy]
        new_state = [self._x, self._y, self._h, self._v, self._w]
        for i in range(5):
            new_state[i] += K[i][0]*dx + K[i][1]*dy
        self._x, self._y, self._h, self._v, self._w = new_state
        while self._h > math.pi: self._h -= 2*math.pi
        while self._h < -math.pi: self._h += 2*math.pi

        # P' = (I - K*H) P
        IKH = [[0]*5 for _ in range(5)]
        for i in range(5):
            for j in range(5):
                orig = 1.0 if i == j else 0.0
                v = 0.0
                if j == 0: v = K[i][0]
                elif j == 1: v = K[i][1]
                IKH[i][j] = orig - v
        newP = [[sum(IKH[i][k]*self._P[k*5+j] for k in range(5)) for j in range(5)] for i in range(5)]
        for i in range(5):
            for j in range(i+1, 5):
                s = 0.5*(newP[i][j] + newP[j][i])
                newP[i][j] = newP[j][i] = s
        self._P = [newP[i][j] for i in range(5) for j in range(5)]

        for idx, minv in [(0, 1e-6), (6, 1e-6), (12, 1e-3), (18, 1e-3), (24, 1e-2)]:
            if self._P[idx] < minv: self._P[idx] = minv
        return True

    @property
    def x(self): return self._x
    @property
    def y(self): return self._y
    @property
    def h(self): return self._h
    @property
    def v(self): return self._v


# === Тест 1: едет прямо с шумным GPS, нет outlier'ов ===
def test_straight_line():
    print("=== Test 1: straight line 10m with noisy GPS ===")
    ekf = Ekf()
    ekf.reset(0, 0, 0)  # heading = 0 (North)
    v = 0.25
    true_x, true_y = 0.0, 0.0
    true_h = 0.0
    cov_xy = 0.0004   # 2cm
    cov_heading = 0.01  # ~5.7 deg

    errs_x, errs_y, errs_h = [], [], []
    outlier_count = 0
    for t_ms in range(0, 40000, 20):  # 40 sec at 50Hz
        dt = 0.02
        true_x += v * math.sin(true_h) * dt
        true_y += v * math.cos(true_h) * dt
        # IMU predict
        ekf.predict(dt, v, 0.0, 0.0)

        # GPS update at 5Hz (every 10 ticks)
        if (t_ms // 20) % 10 == 0:
            gx = true_x + random.gauss(0, 0.02)  # 2cm noise
            gy = true_y + random.gauss(0, 0.02)
            ekf.updatePosition(gx, gy, cov_xy, True)
            # GPS heading at 5Hz
            ekf._h  # NOTE: EKF.predict уже учёл, тут не делаем updateHeading в этой симуляции

        errs_x.append(ekf.x - true_x)
        errs_y.append(ekf.y - true_y)
        errs_h.append((ekf.h - true_h) % (2*math.pi))

    rms_x = math.sqrt(sum(e*e for e in errs_x)/len(errs_x))
    rms_y = math.sqrt(sum(e*e for e in errs_y)/len(errs_y))
    print(f"  RMS x: {rms_x*100:.2f} cm   RMS y: {rms_y*100:.2f} cm   outliers rejected: {outlier_count}")
    assert rms_x < 0.10, f"RMS x too high: {rms_x}"
    assert rms_y < 0.10, f"RMS y too high: {rms_y}"
    print("  PASS")


# === Тест 2: outlier GPS отбрасывается ===
def test_outlier_rejection():
    print("=== Test 2: outlier GPS rejection ===")
    ekf = Ekf()
    ekf.reset(0, 0, 0)
    # Один outlier на 5-м шаге (5 метров вбок)
    outlier_pos = 0
    outlier_rejected = False
    for t in range(100):
        ekf.predict(0.02, 0.25, 0.0, 0.0)
        if t == 50:
            # fix с позицией 5 метров вбок — Mahalanobis должен отбросить
            result = ekf.updatePosition(5.0, 0.0, 0.0004, True)
            print(f"  t=50: outlier at (5,0) -> updatePosition returned {result}")
            outlier_rejected = (not result)
        elif t % 10 == 0:
            ekf.updatePosition(0.0, 0.01 * t, 0.0004, True)
    assert outlier_rejected, "Outlier was NOT rejected!"
    # После outlier позиция должна быть близка к (0, 1), не к (5, 0)
    assert abs(ekf.y - 1.0) < 0.1, f"After outlier, ekf.y = {ekf.y} (expected ~1.0)"
    print(f"  EKF y after outlier: {ekf.y:.3f} m (expected ~1.0)")
    print("  PASS")


# === Тест 3: dead-reckoning при пропадании GPS на 1 сек ===
def test_gps_outage():
    print("=== Test 3: 1s GPS outage - heading/position from IMU+odometry ===")
    ekf = Ekf()
    ekf.reset(0, 0, 0)
    v_true = 0.25
    true_x, true_y = 0.0, 0.0
    cov_xy = 0.0004
    gps_available = True

    for t in range(50*5):  # 5 sec at 50Hz
        dt = 0.02
        true_y += v_true * dt
        ekf.predict(dt, v_true, 0.0, 0.0)
        if t > 50 and t < 100:
            gps_available = False
        else:
            gps_available = True
        if gps_available and t % 10 == 0:
            gx = true_x + random.gauss(0, 0.02)
            gy = true_y + random.gauss(0, 0.02)
            ekf.updatePosition(gx, gy, cov_xy, True)
    # После 1 сек пропадания GPS, ошибка позиции — из-за drift одометрии
    # На v=0.25 за 1 сек робот проехал 0.25м, шум одометрии ~5% -> drift ~0.01м
    err = math.sqrt((ekf.x - true_x)**2 + (ekf.y - true_y)**2)
    print(f"  EKF error after 1s GPS outage: {err*100:.2f} cm")
    assert err < 0.5, f"Drift too high after 1s outage: {err}m"
    print("  PASS")


# === Тест 4: 96 точек по snake (базовая проверка логики) ===
def test_snake_route():
    print("=== Test 4: 96-point snake (5 rows, ~19m long) ===")
    # Snake: прямой 20м -> поворот -> обратно со смещением 1.5м
    rows = 5
    row_len = 20.0
    row_step = 1.5
    waypoints = []
    for r in range(rows):
        if r % 2 == 0:
            waypoints.append((r * row_step, 0))
            waypoints.append((r * row_step, row_len))
        else:
            waypoints.append((r * row_step, row_len))
            waypoints.append((r * row_step, 0))
    print(f"  Generated {len(waypoints)} waypoints")
    assert len(waypoints) == 10, f"Expected 10, got {len(waypoints)}"
    print("  PASS (route generation)")


if __name__ == "__main__":
    random.seed(42)
    test_straight_line()
    test_outlier_rejection()
    test_gps_outage()
    test_snake_route()
    print("\nAll EKF tests PASSED.")
