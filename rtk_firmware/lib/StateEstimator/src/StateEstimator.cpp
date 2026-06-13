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
    _headingSeeded = false;
    _lastImuMs = 0;
    _haveLastFix = false;
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
    // Формула ИДЕНТИЧНА приложению (gps_projection.dart) — иначе координаты разъедутся.
    double phi = originLat * M_PI / 180.0;
    double mPerDegLat = 111132.92 - 559.82 * cos(2*phi) + 1.175 * cos(4*phi);
    double mPerDegLon = 111412.84 * cos(phi) - 93.5 * cos(3*phi);
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
    est.x = est.y = 0;
    return true;
}

void StateEstimator::seedHeadingDeg(float headingDeg) {
    float h = normalizeDeg360(headingDeg);
    est.headingDeg = h;
    est.headingFiltDeg = h;
    est.headingValid = true;
    est.headingAgeMs = 0;
    _headingSeeded = true;
}

// --- IMU: интегрируем yaw rate в краткосрочный курс ---
void StateEstimator::onImu(uint32_t nowMs, float yawRateDps, bool imuFresh) {
    if (!imuFresh) { _lastImuMs = nowMs; return; }
    if (_lastImuMs == 0) { _lastImuMs = nowMs; return; }
    float dt = (nowMs - _lastImuMs) * 0.001f;
    _lastImuMs = nowMs;
    if (dt <= 0 || dt > 0.5f) return;   // защита от выбросов dt

    if (!_headingSeeded) return;        // пока нет абсолютного нуля — не интегрируем

    // интеграция gyro — даёт быстрый курс даже на месте (то, чего не было раньше)
    est.headingFiltDeg = normalizeDeg360(est.headingFiltDeg + yawRateDps * dt);
    est.headingValid = true;
    est.headingAgeMs = 0;
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
    bool accept = true;
    if (_haveLastFix && est.originSet) {
        float px, py, nx, ny;
        llaToLocalMeters(_lastFixLat, _lastFixLon, est.originLat, est.originLon, px, py);
        llaToLocalMeters(newLat, newLon, est.originLat, est.originLon, nx, ny);
        float jump = sqrtf((nx-px)*(nx-px) + (ny-py)*(ny-py));
        float dt = (nowMs - _lastFixMs) * 0.001f;
        if (dt < 0) dt = 0;
        float allow = kMaxJumpBaseM + kMaxJumpVFactor * est.speedMps * (dt + 0.001f);
        // мягко: на FLOAT/INVALID допускаем больше (он сам шумный)
        if (est.sol != SOL_FIXED) allow *= 3.0f;
        if (jump > allow) accept = false;
    }

    if (accept) {
        est.lat = newLat;
        est.lon = newLon;
        _lastFixLat = newLat;
        _lastFixLon = newLon;
        _lastFixMs = nowMs;
        _haveLastFix = true;
        if (est.originSet) {
            float lx, ly;
            llaToLocalMeters(est.lat, est.lon, est.originLat, est.originLon, lx, ly);
            est.x = lx;
            est.y = ly;
        }
    }
    // если outlier — позицию НЕ двигаем (dead-reckoning держит прошлую), но fix-метку обновили выше нет

    // --- GPS-course коррекция курса (только при движении) ---
    est.headingDeg = (float)headMot_deg_e5 * 1e-5f;   // сырой GPS-курс (для телеметрии)
    est.headingDeg = normalizeDeg360(est.headingDeg);
    if (est.speedMps > kGpsHeadingMinMps && est.sol != SOL_INVALID && accept) {
        if (!_headingSeeded) {
            // первый надёжный курс — сидим абсолютный ноль
            seedHeadingDeg(est.headingDeg);
        } else {
            // комплементарно подтягиваем gyro-интеграл к GPS-курсу
            float d = wrapDeg180(est.headingDeg - est.headingFiltDeg);
            est.headingFiltDeg = normalizeDeg360(est.headingFiltDeg + kGpsCourseAlpha * d);
        }
        est.headingValid = true;
        est.headingAgeMs = 0;
    }

    est.lastUpdateMs = nowMs;
}

void StateEstimator::onRtcmInfo(uint32_t nowMs, int lastType, int msgCount, int crcFail) {
    (void)lastType; (void)msgCount; (void)crcFail;
    est.rtcmAgeMs = 0;
}

void StateEstimator::tick(uint32_t nowMs) {
    uint32_t p = nowMs - est.lastUpdateMs;
    est.pvtAgeMs = p;
    // heading теперь живёт за счёт gyro-интеграла (onImu), не «протухает» на месте
    est.headingAgeMs = 0;
    if (est.rtcmAgeMs != 0xFFFFFFFFu) {
        est.rtcmAgeMs = p;
    }
}
