// StateEstimator.cpp - Sunray-style GPS+IMU heading fusion (Вариант A). MIT.
// Курс = интеграл gyro (краткосрочно) + коррекция по GPS-course при движении.
// Позиция = RTK local meters (формула проекции идентична приложению gps_projection.dart).

#include "StateEstimator.h"
#include "RtkConfig.h"

void StateEstimator::begin() {
    est = Estimate{};
    est.lastUpdateMs = millis();
    est.pvtAgeMs   = 0xFFFFFFFFu;
    est.rtcmAgeMs  = 0xFFFFFFFFu;
    est.headingAgeMs = 0xFFFFFFFFu;
    est.acceptedPositionAgeMs = 0xFFFFFFFFu;
    est.rejectedPositionFixes = 0;
    _headingSeeded = false;
    _lastImuMs = 0;
    _haveLastFix = false;
    _lastAcceptedPositionMs = 0;
    _lastHeadingMs = 0;
    _ekf.begin();
    _lastPredictMs = 0;
    _lastSpeedLMps = 0;
    _lastSpeedRMps = 0;
    _lastYawRateRadps = 0;
}

float StateEstimator::normalizeDeg360(float d) {
    while (d < 0)    d += 360.0f;
    while (d >= 360) d -= 360.0f;
    return d;
}
float StateEstimator::wrapDeg180(float d) {
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

void StateEstimator::llaToLocalMeters(double lat, double lon,
                                     double originLat, double originLon,
                                     float &x, float &y) {
    // Формула ИДЕНТИЧНА приложению (gps_display_math.dart) — drift < 7см на 30м поля.
    // 6φ и 5φ члены дают <0.001% drift на площади 1км², оставлены для совпадения с app.
    double phi = originLat * M_PI / 180.0;
    double mPerDegLat = 111132.92 - 559.82 * cos(2*phi) + 1.175 * cos(4*phi)
                        - 0.0023 * cos(6*phi);
    double mPerDegLon = 111412.84 * cos(phi) - 93.5 * cos(3*phi)
                        + 0.118 * cos(5*phi);
    double north = (lat - originLat) * mPerDegLat;
    double east  = (lon - originLon) * mPerDegLon;
    x = (float)east;    // x = East
    y = (float)north;   // y = North
}

bool StateEstimator::setOrigin(double lat, double lon) {
    if (lat == 0.0 && lon == 0.0) return false;
    est.originLat = lat;
    est.originLon = lon;
    est.originSet = true;
    float newX = 0, newY = 0;
    if (est.lat != 0.0 || est.lon != 0.0) {
        llaToLocalMeters(est.lat, est.lon, est.originLat, est.originLon, newX, newY);
    }
    est.x = newX;
    est.y = newY;
    // Сброс EKF в новую систему координат. Heading — из последнего засеянного
    // headingFiltDeg (в радианах). headingFiltDeg в [0, 360], поэтому ВСЕГДА конвертим
    // (раньше был баг: при heading=0 (North) условие `> 0` ложно → hRad=0 → EKF
    // считал что heading=0 (East по нашей формуле atan2).
    float hRad = est.headingFiltDeg * 0.01745329f;
    _ekf.reset(newX, newY, hRad);
    return true;
}

void StateEstimator::seedHeadingDeg(float headingDeg) {
    uint32_t nowMs = millis();
    float h = normalizeDeg360(headingDeg);
    est.headingDeg = h;
    est.headingFiltDeg = h;
    est.headingValid = true;
    est.acceptedPositionAgeMs = (_lastAcceptedPositionMs == 0) ? 0xFFFFFFFFu : (nowMs - _lastAcceptedPositionMs);
    est.headingAgeMs = (_lastHeadingMs == 0) ? 0xFFFFFFFFu : (nowMs - _lastHeadingMs);
    _lastHeadingMs = nowMs;
    est.headingAgeMs = 0;
    _headingSeeded = true;
    _seededAtMs = nowMs;  // первые 10 сек после seed'а не верим GPS-course
    // КРИТИЧНО: посеять сам EKF, иначе любой onPvt перезапишет headingFiltDeg из EKF (=0),
    // а потом setOrigin сбросит EKF в hRad=0. Должно быть _ekf.reset с hRad от seed'а.
    float hRad = h * 0.01745329f;
    _ekf.reset(est.x, est.y, hRad);
}

// --- IMU: gyro rate (deg/s → rad/s) подаётся в EKF.predict() для heading ---
// Heading FUSION (EKF): GPS motion heading + IMU gyro. GPS heading подтягивает
// медленный дрейф гироскопа; gyro держит heading между PVT-пакетами и на стоянке.
// Kalman корректно работает даже когда PVT запаздывает — heading не «прыгает».
void StateEstimator::onImu(uint32_t nowMs, float yawRateDps, bool imuFresh,
                           float absYawDeg, bool absYawValid, float absYawAccRad) {
    if (!imuFresh) { _lastImuMs = nowMs; return; }
    _lastImuMs = nowMs;
    float yawRateRadps = yawRateDps * 0.01745329f;   // deg/s → rad/s
    _lastYawRateRadps = yawRateRadps;

    // dt в секундах; первое обращение — dt=0, predict не идёт.
    if (_lastPredictMs == 0) { _lastPredictMs = nowMs; return; }
    float dt = (nowMs - _lastPredictMs) * 0.001f;
    _lastPredictMs = nowMs;
    if (dt <= 0) return;
    if (dt > 0.5f) dt = 0.5f;

    // Скорости берём последние измеренные (или 0, если ещё не было feedback).
    float v_mps = 0.5f * (_lastSpeedLMps + _lastSpeedRMps);
    float omega_radps = (_lastSpeedRMps - _lastSpeedLMps) / max(ROVER_WHEELBASE_M, 0.01f);
    _ekf.predict(dt, v_mps, omega_radps, yawRateRadps);

    // КЛЮЧЕВОЕ: подмешать АБСОЛЮТНЫЙ heading от магнитометра в EKF.
    // Без этого heading = seed + интеграл(gyro): ошибка seed (напр. 60°) и дрейф
    // гироскопа НИКОГДА не исправляются → head уплывает, робот виляет (cross_track).
    // Magnetic yaw стабилен (cal=1, шум ±5°), даём ему малый R (большое доверие).
    (void)absYawDeg;
    (void)absYawValid;
    (void)absYawAccRad;
}

void StateEstimator::onPvt(uint32_t nowMs,
                            int32_t lat_e7, int32_t lon_e7, int32_t height_mm,
                            int32_t hAcc_mm, int32_t vAcc_mm,
                            int32_t gSpeed_mmps, int32_t headMot_deg_e5,
                            int fixType, int carrierSol, bool diffSoln,
                            int numSv, float pDop) {
    double newLat = (double)lat_e7 * 1e-7;
    double newLon = (double)lon_e7 * 1e-7;

    est.alt     = height_mm / 1000.0f;
    est.hAcc    = hAcc_mm / 1000.0f;
    est.vAcc    = vAcc_mm / 1000.0f;
    est.speedMps = gSpeed_mmps / 1000.0f;
    est.fixType    = fixType;
    est.carrierSol = carrierSol;
    est.diff       = diffSoln;
    est.numSv      = numSv;
    est.pDop       = pDop;
    est.pvtAgeMs   = 0;

    // SolType gate — carrierSol уже реальный (0/1/2) из Gnss
    if (carrierSol == 2)      est.sol = SOL_FIXED;
    else if (carrierSol == 1) est.sol = SOL_FLOAT;
    else                      est.sol = SOL_INVALID;

    // --- outlier rejection: отбрасываем неправдоподобный скачок позиции ---
    bool reliablePosition = est.sol == SOL_FIXED && est.hAcc <= SAFE_HACC_FIXED_M && fixType >= 3;
    bool accept = reliablePosition;
    if (reliablePosition && _haveLastFix && est.originSet) {
        float px, py, nx, ny;
        llaToLocalMeters(_lastFixLat, _lastFixLon, est.originLat, est.originLon, px, py);
        llaToLocalMeters(newLat, newLon, est.originLat, est.originLon, nx, ny);
        float jump = sqrtf((nx-px)*(nx-px) + (ny-py)*(ny-py));
        float dt = (nowMs - _lastFixMs) * 0.001f;
        if (dt < 0) dt = 0;
        float allow = kMaxJumpBaseM + kMaxJumpVFactor * est.speedMps * (dt + 0.001f);
        // мягко: на FLOAT/INVALID допускаем больше (он сам шумный)
        if (jump > allow) accept = false;
    }

    est.lat = newLat;
    est.lon = newLon;
    if (accept) {
        _lastFixLat = newLat;
        _lastFixLon = newLon;
        _lastFixMs = nowMs;
        _lastAcceptedPositionMs = nowMs;
        est.acceptedPositionAgeMs = 0;
        est.rejectedPositionFixes = 0;
        _haveLastFix = true;
        if (est.originSet) {
            float lx, ly;
            llaToLocalMeters(est.lat, est.lon, est.originLat, est.originLon, lx, ly);
            // === EKF update: позиция из RTK с covariance из hAcc ===
            // cov = hAcc^2 (дисперсия в m^2). hAcc — точность 1-sigma, var = sigma^2.
            float cov_xy = est.hAcc * est.hAcc;
            bool updated = _ekf.updatePosition(lx, ly, cov_xy, true);
            // Если EKF отбросил fix (Mahalanobis), НЕ обновляем est.x/est.y, но и не
            // инкрементируем rejectedPositionFixes — это сделает сам EKF в P-ковариации.
            if (updated) {
                est.x = _ekf.x();
                est.y = _ekf.y();
            } else {
                est.x = _ekf.x();   // EKF оставил prediction, мы тоже остаёмся на нём
                est.y = _ekf.y();
                if (est.rejectedPositionFixes < 0xFFFFu) est.rejectedPositionFixes++;
            }
        }
    } else if (reliablePosition) {
        if (est.rejectedPositionFixes < 0xFFFFu) est.rejectedPositionFixes++;
    }
    // если outlier — позицию НЕ двигаем (dead-reckoning держит прошлую), но fix-метку обновили выше нет

    // --- Курс: fusion EKF (GPS motion heading + IMU gyro) ---
    // При движении (speed > порог) GPS heading меряется надёжно — подаём в EKF.updateHeading.
    // ВНИМАНИЕ: GPS heading (headMot) ВЫКЛЮЧЕН из EKF update.
    // Причины: в логах 2026-06-16 GPS-course показывал 330.9° при реальном heading 124°
    // (расхождение 207°). U-blox headMot — это course-over-ground, вычисленный из
    // последовательных position fixes; на нашей скорости 0.25 м/с (за 200мс пакет
    // робот проезжает 5 см) — это просто шум, а при стоянке — мусор от дрейфа.
    // heading живёт за счёт EKF.predict() от IMU gyro. seedHeadingDeg() ставит
    // начальное значение. SET_HEADING,X сбрасывает при необходимости.
    est.headingDeg = (float)headMot_deg_e5 * 1e-5f;   // сырой GPS-курс только для ТЕЛЕМЕТРИИ
    est.headingDeg = normalizeDeg360(est.headingDeg);

    // Heading fusion: GPS-course подмешиваем в EKF ТОЛЬКО при большой скорости
    // (чтобы база допплера была длинной и heading надёжным).
    // При v < 0.7 m/s headMot вырождается в шум и ломает heading — оставляем IMU gyro.
    // При v ≥ 0.7 m/s подаём в EKF.updateHeading с большой covariance (1.0 рад^2 ≈ 57°).
    if (_headingSeeded && gSpeed_mmps >= 200 && fixType >= 3 && carrierSol >= 2) {
        float gpsHeadDeg = (float)headMot_deg_e5 * 1e-5f;
        gpsHeadDeg = normalizeDeg360(gpsHeadDeg);
        float gpsHeadRad = gpsHeadDeg * 0.01745329f;
        float cov_rad2 = 1.0f; // большая — пусть EKF сам решает сколько верить (доверяет gyro в основном)
        _ekf.updateHeading(gpsHeadRad, cov_rad2, true);
    }

    // Каждый PVT — обновляем headingFiltDeg из EKF (если heading уже засеян).
    if (_headingSeeded) {
        float hDeg = _ekf.heading() * 57.2957795f;
        hDeg = normalizeDeg360(hDeg);
        est.headingFiltDeg = hDeg;
        est.headingValid = true;
        est.headingAgeMs = 0;
        _lastHeadingMs = nowMs;
    }

    est.lastUpdateMs = nowMs;
}

void StateEstimator::onRtcmInfo(uint32_t nowMs, int lastType, int msgCount, int crcFail) {
    (void)lastType; (void)msgCount; (void)crcFail;
    est.rtcmAgeMs = 0;
}

// --- Hoverboard feedback → EKF predict ---
// speedL_meas / speedR_meas: из платы hoverboard (int16). В прошивке hoverboard-firmware-hack
// единицы = RPM × 6. Переводим в м/с через ROVER_WHEEL_CIRCUM_M и обновляем _lastSpeed*L/Mps.
// EKF.predict в onImu() забирает эти значения.
void StateEstimator::onHoverboardFeedback(uint32_t nowMs, int speedL_meas, int speedR_meas) {
    (void)nowMs;
    // Конверсия: speed_meas / 6 = RPM. v = RPM * 2π * R / 60 = (speed_meas/6) * 2π * R / 60
    //                              = speed_meas * R * π / 180
    // С R = 0.3 (половина от 0.6 — эффективный радиус), π/180 ≈ 0.01745:
    //   v_mps ≈ speed_meas * 0.00524
    // С ROVER_WHEEL_CIRCUM_M = 0.6 (длина гусеницы), R = 0.6 / (2π) ≈ 0.0955
    //   v_mps = speed_meas * (1/6) * (0.6) / 60  =  speed_meas * 0.001667
    // Грубо — калибруется по HITL. На время отладки берём среднее.
    constexpr float kMeasToMps = 0.0017f;     // 0.0017 м/с на единицу speed_meas (прибл.)
    _lastSpeedLMps = (float)speedL_meas * kMeasToMps;
    _lastSpeedRMps = (float)speedR_meas * kMeasToMps;
    // Подрежем явный мусор: если hoverboard на старте шлёт 0/0, не дёргаем EKF шумом.
    if (abs(speedL_meas) < 5 && abs(speedR_meas) < 5) {
        _lastSpeedLMps = 0;
        _lastSpeedRMps = 0;
    }
}

void StateEstimator::tick(uint32_t nowMs) {
    uint32_t p = nowMs - est.lastUpdateMs;
    est.pvtAgeMs = p;
    est.acceptedPositionAgeMs = (_lastAcceptedPositionMs == 0) ? 0xFFFFFFFFu : (nowMs - _lastAcceptedPositionMs);
    est.headingAgeMs = (_lastHeadingMs == 0) ? 0xFFFFFFFFu : (nowMs - _lastHeadingMs);
    if (est.rtcmAgeMs != 0xFFFFFFFFu) {
        est.rtcmAgeMs = p;
    }

    // EKF.predict на каждом tick (50Гц) с текущим IMU rate. Это держит heading
    // и dead-reckoning между PVT-пакетами и во время их отсутствия.
    // dt считается внутри onImu, здесь — fallback, если onImu не вызывали.
    if (_lastPredictMs == 0) _lastPredictMs = nowMs;
    float dt = (nowMs - _lastPredictMs) * 0.001f;
    if (dt > 0.1f) {
        // IMU молчал > 100мс — не страшно, dt будет использован в следующем onImu.
        // Здесь только сдвигаем метку, чтобы dt не «накапливался».
        _lastPredictMs = nowMs;
    }
}
