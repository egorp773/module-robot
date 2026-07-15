// StateEstimator.cpp - Sunray-style GPS+IMU heading fusion (Вариант A). MIT.
// Курс = интеграл gyro (краткосрочно) + абсолютный yaw от IMU.
// Позиция = RTK local meters (формула проекции идентична приложению gps_projection.dart).

#include "StateEstimator.h"
#include "RtkConfig.h"
#include "NavMath.h"
#include "ImuMath.h"

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
    _lastRtcmMs = 0;
    _ekf.begin();
    _lastPredictMs = 0;
    _lastSpeedLMps = 0;
    _lastSpeedRMps = 0;
    _lastYawRateRadps = 0;
    _fieldSafetyMode = false;
    _antennaForwardOffsetM = ROVER_ANTENNA_FORWARD_OFFSET_M;
    _antennaLeftOffsetM = ROVER_ANTENNA_LEFT_OFFSET_M;
    _antennaCorrectionEnabled = true;
    _lastHeadingOutputMs = 0;
    _lastHeadingJumpLogMs = 0;
    _courseRotatingInPlace = false;
    _courseCommandedLinearMps = 0;
    _courseWindowValid = false;
    _courseWindowStartX = _courseWindowStartY = 0;
    _courseWindowStartMs = 0;
    _courseWindowAccumMs = 0;
    _courseGyroAccumDeg = 0;
}

float StateEstimator::normalizeDeg360(float d) {
    return NavMath::normalizeDeg360(d);
}
float StateEstimator::wrapDeg180(float d) {
    return NavMath::wrapDeg180(d);
}

void StateEstimator::setAntennaOffsets(float forwardM, float leftM) {
    _antennaForwardOffsetM = isfinite(forwardM) ? forwardM : 0.0f;
    _antennaLeftOffsetM = isfinite(leftM) ? leftM : 0.0f;
    updateControlPoint();
}

void StateEstimator::setAntennaCorrectionEnabled(bool enabled) {
    if (_antennaCorrectionEnabled == enabled) return;
    _antennaCorrectionEnabled = enabled;
    updateControlPoint();
}

void StateEstimator::setGpsCourseMotionContext(bool rotatingInPlace,
                                                float commandedLinearMps) {
    _courseRotatingInPlace = rotatingInPlace;
    _courseCommandedLinearMps = isfinite(commandedLinearMps)
        ? commandedLinearMps : 0.0f;
}

void StateEstimator::updateControlPoint() {
    if (!_antennaCorrectionEnabled || !_headingSeeded ||
        (_antennaForwardOffsetM == 0.0f && _antennaLeftOffsetM == 0.0f)) {
        est.x = est.rawAntennaX;
        est.y = est.rawAntennaY;
        return;
    }
    const float h = est.headingFiltDeg * 0.0174532925f;
    est.x = est.rawAntennaX - _antennaForwardOffsetM * sinf(h)
            + _antennaLeftOffsetM * cosf(h);
    est.y = est.rawAntennaY - _antennaForwardOffsetM * cosf(h)
            - _antennaLeftOffsetM * sinf(h);
}

bool StateEstimator::headingMeasurementPlausible(uint32_t nowMs,
                                                  float candidateDeg,
                                                  const char* source,
                                                  float yawRateDps,
                                                  float measurementDtSec,
                                                  float gyroExpectedDeg) {
    if (!_headingSeeded || _lastHeadingOutputMs == 0) return true;
    const float dt = measurementDtSec >= 0.0f
        ? measurementDtSec
        : min((nowMs - _lastHeadingOutputMs) * 0.001f, 0.5f);
    const float oldDeg = ImuMath::normalizeDeg360(_ekf.heading() * 57.2957795f);
    const float delta = ImuMath::wrapDeg180(candidateDeg - oldDeg);
    const float gyroExpected = isfinite(gyroExpectedDeg)
        ? gyroExpectedDeg : yawRateDps * dt;
    const float allowed = 8.0f + min(8.0f, fabsf(gyroExpected) * 1.5f);
    if (fabsf(delta) <= allowed) return true;
    if (_lastHeadingJumpLogMs == 0 ||
        (nowMs - _lastHeadingJumpLogMs) >= 500u) {
        Serial.printf("[HEADING_JUMP_REJECT] old=%.1f candidate=%.1f delta=%+.1f "
                      "gyroExpected=%+.1f dt=%.3f source=%s\n",
                      (double)oldDeg, (double)candidateDeg, (double)delta,
                      (double)gyroExpected, (double)dt,
                      source ? source : "unknown");
        _lastHeadingJumpLogMs = nowMs;
    }
    return false;
}

void StateEstimator::llaToLocalMeters(double lat, double lon,
                                     double originLat, double originLon,
                                     float &x, float &y) {
    // Формула ИДЕНТИЧНА приложению (gps_display_math.dart) — drift < 7см на 30м поля.
    // 6φ и 5φ члены дают <0.001% drift на площади 1км², оставлены для совпадения с app.
    NavMath::llaToLocalMeters(lat, lon, originLat, originLon, x, y);
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
    est.rawAntennaX = newX;
    est.rawAntennaY = newY;
    // Сброс EKF в новую систему координат. Heading — из последнего засеянного
    // headingFiltDeg (в радианах). headingFiltDeg в [0, 360], поэтому ВСЕГДА конвертим
    // (раньше был баг: при heading=0 (North) условие `> 0` ложно → hRad=0 → EKF
    // считал что heading=0 (East по нашей формуле atan2).
    float hRad = est.headingFiltDeg * 0.01745329f;
    _ekf.reset(newX, newY, hRad);
    updateControlPoint();
    return true;
}

void StateEstimator::seedHeadingDeg(float headingDeg, ImuYawSource yawSource) {
    uint32_t nowMs = millis();
    float h = ImuMath::normalizeDeg360(headingDeg);
    const bool sourceAbsolute = ImuMath::yawSourceIsAbsolute(yawSource);
    est.headingDeg = h;
    est.headingFiltDeg = h;
    est.headingValid = true;
    est.absYawDeg = h;
    est.absYawValid = sourceAbsolute;
    est.yawIsAbsolute = sourceAbsolute;
    est.yawSource = yawSource;
    est.headingUsedByEstimator = sourceAbsolute;
    est.acceptedPositionAgeMs = (_lastAcceptedPositionMs == 0) ? 0xFFFFFFFFu : (nowMs - _lastAcceptedPositionMs);
    est.headingAgeMs = (_lastHeadingMs == 0) ? 0xFFFFFFFFu : (nowMs - _lastHeadingMs);
    _lastHeadingMs = nowMs;
    est.headingAgeMs = 0;
    _headingSeeded = true;
    _seededAtMs = nowMs;  // первые 10 сек после seed'а не верим GPS-course
    // КРИТИЧНО: посеять сам EKF, иначе любой onPvt перезапишет headingFiltDeg из EKF (=0),
    // а потом setOrigin сбросит EKF в hRad=0. Должно быть _ekf.reset с hRad от seed'а.
    float hRad = h * 0.01745329f;
    _ekf.reset(est.rawAntennaX, est.rawAntennaY, hRad);
    _lastHeadingOutputMs = nowMs;
    updateControlPoint();
}

// --- IMU: gyro rate (deg/s → rad/s) подаётся в EKF.predict() для heading ---
// Heading FUSION (EKF): IMU gyro prediction + absolute IMU yaw correction.
// GPS course-over-ground fusion is temporarily disabled in onPvt().
// Kalman корректно работает даже когда PVT запаздывает — heading не «прыгает».
void StateEstimator::onImu(uint32_t nowMs, float yawRateDps, bool imuFresh,
                           float absYawDeg, bool absYawValid, float absYawAccRad,
                           ImuYawSource yawSource, bool yawIsAbsolute) {
    if (!imuFresh) { _lastImuMs = nowMs; return; }
    _lastImuMs = nowMs;
    float yawRateRadps = yawRateDps * 0.01745329f;   // deg/s → rad/s
    _lastYawRateRadps = yawRateRadps;

    float dt = 0.0f;
    if (_lastPredictMs == 0) {
        _lastPredictMs = nowMs;
    } else {
        dt = (nowMs - _lastPredictMs) * 0.001f;
        _lastPredictMs = nowMs;
        if (dt > 0.5f) dt = 0.5f;
    }

    if (dt > 0.0f) {
        if (_courseWindowValid)
            _courseGyroAccumDeg += yawRateDps * dt;
        // Скорости берём последние измеренные (или 0, если ещё не было feedback).
        float v_mps = 0.5f * (_lastSpeedLMps + _lastSpeedRMps);
        float omega_radps = (_lastSpeedRMps - _lastSpeedLMps) / max(ROVER_WHEELBASE_M, 0.01f);
        _ekf.predict(dt, v_mps, omega_radps, yawRateRadps);
    }

    est.absYawDeg = ImuMath::normalizeDeg360(absYawDeg);
    est.absYawValid = absYawValid &&
                      yawIsAbsolute &&
                      ImuMath::yawSourceIsAbsolute(yawSource) &&
                      absYawAccRad <= IMU_ABS_YAW_MAX_ACC_RAD;
    est.absYawAccRad = absYawAccRad;
    est.yawSource = yawSource;
    est.yawIsAbsolute = yawIsAbsolute;

    bool headingUsed = false;
    if (_headingSeeded && est.absYawValid &&
        headingMeasurementPlausible(nowMs, est.absYawDeg,
                                    ImuMath::yawSourceName(yawSource),
                                    yawRateDps)) {
        float yawRad = est.absYawDeg * 0.01745329f;
        float covRad2 = absYawAccRad * absYawAccRad;
        if (covRad2 < 0.0004f) covRad2 = 0.0004f;  // ~1.1 deg minimum
        if (covRad2 > 2.0f) covRad2 = 2.0f;
        headingUsed = _ekf.updateHeading(yawRad, covRad2, true);
    }
    est.headingUsedByEstimator = headingUsed;

    if (_headingSeeded) {
        float hDeg = _ekf.heading() * 57.2957795f;
        hDeg = ImuMath::normalizeDeg360(hDeg);
        est.headingFiltDeg = hDeg;
        est.headingValid = true;
        est.headingAgeMs = 0;
        _lastHeadingMs = nowMs;
        _lastHeadingOutputMs = nowMs;
        updateControlPoint();
    }
}

void StateEstimator::onPvt(uint32_t nowMs,
                            int32_t lat_e7, int32_t lon_e7, int32_t height_mm,
                            int32_t hAcc_mm, int32_t vAcc_mm,
                            int32_t gSpeed_mmps, int32_t headMot_deg_e5,
                            int fixType, int carrierSol, bool diffSoln,
                            int numSv, float pDop,
                            int32_t headAcc_deg_e5,
                            uint32_t pvtIntervalMs) {
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
    const float fixedLimit = _fieldSafetyMode ? FIELD_HACC_FIXED_M
                                              : SAFE_HACC_FIXED_M;
    const float floatLimit = _fieldSafetyMode ? FIELD_HACC_FLOAT_M
                                              : SAFE_HACC_FLOAT_M;
    bool reliablePosition =
        fixType >= 3 &&
        ((est.sol == SOL_FIXED && est.hAcc <= fixedLimit) ||
         (est.sol == SOL_FLOAT && est.hAcc <= floatLimit));
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
                est.rawAntennaX = _ekf.x();
                est.rawAntennaY = _ekf.y();
            } else {
                est.rawAntennaX = _ekf.x();
                est.rawAntennaY = _ekf.y();
                if (est.rejectedPositionFixes < 0xFFFFu) est.rejectedPositionFixes++;
            }
            updateControlPoint();
        }
    } else if (reliablePosition) {
        if (est.rejectedPositionFixes < 0xFFFFu) est.rejectedPositionFixes++;
    }
    // если outlier — позицию НЕ двигаем (dead-reckoning держит прошлую), но fix-метку обновили выше нет

    // --- Курс: raw GPS course для телеметрии ---
    est.headingDeg = (float)headMot_deg_e5 * 1e-5f;
    est.headingDeg = ImuMath::normalizeDeg360(est.headingDeg);
    est.gpsCourseAccDeg = (float)headAcc_deg_e5 * 1e-5f;

    // --- GPS course-over-ground → EKF heading update ---
    // Без этого курс после RTK-align держится ТОЛЬКО на интеграле гироскопа
    // и дрейфует (1-3°/мин) → cross-track растёт → «кривая» езда.
    // Защита от шума на стоянке: фьюзим только когда
    //   * позиция принята (accept) и решение FLOAT/FIXED,
    //   * реальная скорость >= GPS_COURSE_FUSE_MIN_MPS,
    //   * headAcc валиден и не хуже GPS_COURSE_FUSE_MAX_ACC_DEG.
    // Ковариация — честная, из headAcc (1-sigma), поэтому слабый course
    // почти не тянет, а уверенный (быстрое прямое движение) — тянет сильно.
    // Mahalanobis-гейт внутри updateHeading отсекает выбросы.
    est.gpsCourseUsed = false;
    est.gpsCourseWindowDistM = 0;
    est.gpsCourseWindowMs = 0;
    const bool courseMotionValid = _headingSeeded && est.originSet && accept &&
        !_courseRotatingInPlace &&
        fabsf(_courseCommandedLinearMps) >= 0.05f &&
        est.speedMps >= 0.05f &&
        (est.sol == SOL_FIXED || est.sol == SOL_FLOAT);
    if (!courseMotionValid) {
        _courseWindowValid = false;
        _courseGyroAccumDeg = 0;
        _courseWindowAccumMs = 0;
    } else if (!_courseWindowValid) {
        _courseWindowValid = true;
        _courseWindowStartX = est.x;
        _courseWindowStartY = est.y;
        _courseWindowStartMs = nowMs;
        _courseWindowAccumMs = 0;
        _courseGyroAccumDeg = 0;
    } else {
        if (pvtIntervalMs > 0u && pvtIntervalMs < 2000u)
            _courseWindowAccumMs += pvtIntervalMs;
        const float courseDx = est.x - _courseWindowStartX;
        const float courseDy = est.y - _courseWindowStartY;
        const float courseDistance = sqrtf(courseDx * courseDx +
                                           courseDy * courseDy);
        const uint32_t courseWindowMs = _courseWindowAccumMs > 0
            ? _courseWindowAccumMs : (nowMs - _courseWindowStartMs);
        est.gpsCourseWindowDistM = courseDistance;
        est.gpsCourseWindowMs = courseWindowMs;
        if (courseDistance >= 0.15f && courseWindowMs >= 400u) {
            const float motionCourseDeg = NavMath::targetHeadingDeg(
                courseDx, courseDy);
            const float courseAccDeg = est.gpsCourseAccDeg;
            if (courseAccDeg > 0.01f &&
                courseAccDeg <= GPS_COURSE_FUSE_MAX_ACC_DEG) {
                const float courseRad = motionCourseDeg * 0.01745329f;
                const float sigmaRad = courseAccDeg * 0.01745329f;
                float cov = sigmaRad * sigmaRad;
                if (cov < 0.0012f) cov = 0.0012f;
                const float measurementDt = courseWindowMs * 0.001f;
                if (headingMeasurementPlausible(
                        nowMs, motionCourseDeg, "gps_course_window",
                        _lastYawRateRadps * 57.2957795f,
                        measurementDt, _courseGyroAccumDeg)) {
                    est.gpsCourseUsed = _ekf.updateHeading(courseRad, cov, true);
                }
            }
            _courseWindowStartX = est.x;
            _courseWindowStartY = est.y;
            _courseWindowStartMs = nowMs;
            _courseWindowAccumMs = 0;
            _courseGyroAccumDeg = 0;
        }
    }

    // PVT may publish the current EKF value, but must not make an IMU-derived
    // heading look fresh. Course fusion above uses its own displacement window.
    if (_headingSeeded) {
        float hDeg = _ekf.heading() * 57.2957795f;
        hDeg = ImuMath::normalizeDeg360(hDeg);
        est.headingFiltDeg = hDeg;
        est.headingValid = true;
        _lastHeadingOutputMs = nowMs;
        updateControlPoint();
    }

    est.lastUpdateMs = nowMs;
}

void StateEstimator::onRtcmInfo(uint32_t nowMs, int lastType, int msgCount, int crcFail) {
    (void)lastType; (void)msgCount; (void)crcFail;
    _lastRtcmMs = nowMs;
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
    // БАГФИКС: раньше rtcmAgeMs зеркалил возраст PVT (est.rtcmAgeMs = p) —
    // телеметрия врала. Теперь считается от реального последнего RTCM-пакета.
    est.rtcmAgeMs = (_lastRtcmMs == 0) ? 0xFFFFFFFFu : (nowMs - _lastRtcmMs);

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
