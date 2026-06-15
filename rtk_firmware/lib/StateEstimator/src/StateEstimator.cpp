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
    if (est.lat != 0.0 || est.lon != 0.0) {
        float lx, ly;
        llaToLocalMeters(est.lat, est.lon, est.originLat, est.originLon, lx, ly);
        est.x = lx;
        est.y = ly;
    } else {
        est.x = est.y = 0;
    }
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
}

// --- IMU: НЕ участвует в курсе (GPS-only heading) ---
// На предыдущих итерациях здесь интегрировался yaw rate гироскопа в headingFiltDeg.
// Это вызывало дрейф курса на 5-15° за минуту из-за drift BNO085, и в комбинации с
// медленным GPS-обновлением (5 Hz, замороженным при скорости <3 см/с) робот уезжал
// с маршрута. Сейчас курс берётся ТОЛЬКО из GPS motion heading в onPvt().
// IMU оставлен подключённым (для будущего tilt-детекта и для safety IMU-fresh гейта),
// но в навигацию НЕ вмешивается.
void StateEstimator::onImu(uint32_t nowMs, float yawRateDps, bool imuFresh) {
    (void)yawRateDps;
    if (!imuFresh) { _lastImuMs = nowMs; return; }
    _lastImuMs = nowMs;
    // intentionally no-op: headingFiltDeg живёт за счёт GPS в onPvt()
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
            est.x = lx;
            est.y = ly;
        }
    } else if (reliablePosition) {
        if (est.rejectedPositionFixes < 0xFFFFu) est.rejectedPositionFixes++;
    }
    // если outlier — позицию НЕ двигаем (dead-reckoning держит прошлую), но fix-метку обновили выше нет

    // --- Курс ТОЛЬКО из GPS motion heading (GPS-only, без IMU) ---
    // При движении курс = направление вектора скорости GPS (headMot). Это единственный
    // источник курса. При стоянии (speed <= порог) курс ЗАМОРАЖИВАЕТСЯ (держим последний),
    // никакого гиро-интеграла — иначе крутка на месте. Лёгкое сглаживание по GPS-курсу,
    // чтобы шум headMot на низкой скорости не дёргал руль.
    est.headingDeg = (float)headMot_deg_e5 * 1e-5f;   // сырой GPS-курс
    est.headingDeg = normalizeDeg360(est.headingDeg);
    if (est.speedMps > kGpsHeadingMinMpsActive && est.sol != SOL_INVALID && accept) {
        if (!_headingSeeded) {
            // первый надёжный курс — берём как есть
            seedHeadingDeg(est.headingDeg);
        } else {
            // лёгкое сглаживание к GPS-курсу (быстрое — это и есть наш курс)
            float d = wrapDeg180(est.headingDeg - est.headingFiltDeg);
            if (fabsf(d) > kGpsHeadingSnapDeg) {
                est.headingFiltDeg = est.headingDeg;
            } else {
                est.headingFiltDeg = normalizeDeg360(est.headingFiltDeg + kGpsCourseAlphaActive * d);
            }
        }
        est.headingValid = true;
        est.headingAgeMs = 0;
        _lastHeadingMs = nowMs;
    }
    // при стоянии — est.headingFiltDeg не меняется (заморожен на последнем GPS-курсе)

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
    est.acceptedPositionAgeMs = (_lastAcceptedPositionMs == 0) ? 0xFFFFFFFFu : (nowMs - _lastAcceptedPositionMs);
    est.headingAgeMs = (_lastHeadingMs == 0) ? 0xFFFFFFFFu : (nowMs - _lastHeadingMs);
    if (est.rtcmAgeMs != 0xFFFFFFFFu) {
        est.rtcmAgeMs = p;
    }
}
