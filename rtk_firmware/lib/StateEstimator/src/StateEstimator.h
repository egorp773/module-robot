// StateEstimator.h - Sunray-style GPS+heading fusion. MIT. Алгоритмы из Sunray (GPLv3).

#pragma once
#include <Arduino.h>
#include "RtkEkf.h"
#include "ImuMath.h"

enum SolType : uint8_t {
    SOL_INVALID = 0,
    SOL_FLOAT   = 1,
    SOL_FIXED   = 2,
};

// Все поля читаются из main loop. Потокобезопасность не нужна (один loop).
struct Estimate {
    // position
    double  lat = 0;          // degrees
    double  lon = 0;
    float   alt = 0;          // meters
    float   hAcc = 999;       // meters
    float   vAcc = 999;       // meters
    // motion
    float   speedMps = 0;     // 2D ground speed
    float   headingDeg = 0;   // 0..360, GPS heading (motion-based)
    bool    headingValid = false;
    float   absYawDeg = 0;
    bool    absYawValid = false;
    float   absYawAccRad = 999;
    ImuYawSource yawSource = ImuYawSource::NONE;
    bool    yawIsAbsolute = false;
    bool    headingUsedByEstimator = false;
    // GPS course-over-ground fusion диагностика
    bool    gpsCourseUsed = false;    // последний PVT скорректировал курс EKF
    float   gpsCourseAccDeg = 999;    // headAcc последнего PVT, градусы
    float   gpsCourseWindowDistM = 0;
    uint32_t gpsCourseWindowMs = 0;
    // quality
    SolType sol = SOL_INVALID;
    int     numSv = 0;
    float   pDop = 99;
    bool    diff = false;
    int     fixType = 0;      // raw u-blox fix type
    int     carrierSol = 0;   // raw u-blox carrier solution
    // ages (ms since update)
    uint32_t pvtAgeMs   = 0xFFFFFFFFu;
    uint32_t rtcmAgeMs  = 0xFFFFFFFFu;
    uint32_t headingAgeMs = 0xFFFFFFFFu;
    uint32_t acceptedPositionAgeMs = 0xFFFFFFFFu;
    uint16_t rejectedPositionFixes = 0;
    // filtered position (local x,y) — для nav
    float   rawAntennaX = 0, rawAntennaY = 0;
    float   x = 0, y = 0;   // corrected robot control point
    // filtered heading (low-pass, freeze when stationary)
    float   headingFiltDeg = 0;
    uint32_t lastUpdateMs = 0;
    // origin for local projection
    bool    originSet = false;
    double  originLat = 0, originLon = 0;
};

class StateEstimator {
public:
    void begin();

    // apply new raw PVT message.
    // headAcc_deg_e5 — точность headMot (deg*1e-5); дефолт = "нет данных",
    // GPS-course fusion при этом не выполняется.
    void onPvt(uint32_t nowMs,
               int32_t lat_e7, int32_t lon_e7, int32_t height_mm,
               int32_t hAcc_mm, int32_t vAcc_mm,
               int32_t gSpeed_mmps, int32_t headMot_deg_e5,
               int fixType, int carrierSol, bool diffSoln,
               int numSv, float pDop,
               int32_t headAcc_deg_e5 = 18000000,
               uint32_t pvtIntervalMs = 0);

    // apply new RXM-RTCM info (F9P decode side)
    void onRtcmInfo(uint32_t nowMs, int lastType, int msgCount, int crcFail);

    // Одометрия от hoverboard feedback: speedL_meas, speedR_meas (в единицах платы).
    // Внутри переводим в м/с через ROVER_WHEEL_CIRCUM_M и используем для EKF predict.
    void onHoverboardFeedback(uint32_t nowMs, int speedL_meas, int speedR_meas);

    // периодический tick — обновляет возраст и фильтры
    void tick(uint32_t nowMs);

    // НОВОЕ: подать данные IMU (yaw rate, град/с) — интегрируется для краткосрочного курса.
    // Вызывать каждый tick ДО tick(). dtMs — фактический интервал.
    // absYawDeg/absYawValid — абсолютный курс от магнитометра (0..360 CW от севера):
    // если валиден, подаётся в EKF как measurement update и убирает дрейф/ошибку seed.
    void onImu(uint32_t nowMs, float yawRateDps, bool imuFresh,
               float absYawDeg = 0.0f, bool absYawValid = false, float absYawAccRad = 999.0f,
               ImuYawSource yawSource = ImuYawSource::NONE, bool yawIsAbsolute = false);

    // origin для перевода lat/lon → x/y
    bool setOrigin(double lat, double lon);
    void clearOrigin() { est.originSet = false; }
    void seedHeadingDeg(float headingDeg, ImuYawSource yawSource = ImuYawSource::NONE);   // абсолютная калибровка курса (CAL_HEADING)
    void setFieldSafetyMode(bool enabled) { _fieldSafetyMode = enabled; }
    void setAntennaOffsets(float forwardM, float leftM);
    void setAntennaCorrectionEnabled(bool enabled);
    void setGpsCourseMotionContext(bool rotatingInPlace,
                                   float commandedLinearMps);

    const Estimate& get() const { return est; }

    // helpers
    static void llaToLocalMeters(double lat, double lon,
                                 double originLat, double originLon,
                                 float &x, float &y);
    static float normalizeDeg360(float d);
    static float wrapDeg180(float d);

private:
    Estimate est;
    static constexpr float kGpsCourseAlphaActive = 0.30f;
    static constexpr float kGpsHeadingMinMpsActive = 0.30f;  // GPS-course мусор на стоянке
    static constexpr float kGpsHeadingSnapDeg = 45.0f;
    static constexpr float kImuYawRateDeadbandDps = 0.35f;
    static constexpr float kImuMaxDeltaDeg = 12.0f;
    // Комплементарный фильтр курса (Sunray-style, Вариант A):
    //   fused = integrate(gyro) ; при движении подтягиваем к GPS-course с этим alpha.
    static constexpr float kGpsCourseAlpha   = 0.05f;   // доля коррекции по GPS-курсу за PVT
    static constexpr float kGpsHeadingMinMps = 0.15f;   // ниже — GPS-курсу не верим
    static constexpr float kPosAlpha         = 1.00f;   // позиция RTK точная — без сглаживания

    // outlier rejection по скачку позиции
    static constexpr float kMaxJumpBaseM     = 0.50f;   // допуск + v*dt
    static constexpr float kMaxJumpVFactor   = 2.0f;

    bool headingMeasurementPlausible(uint32_t nowMs, float candidateDeg,
                                     const char* source, float yawRateDps,
                                     float measurementDtSec = -1.0f,
                                     float gyroExpectedDeg = NAN);
    void updateControlPoint();

    bool     _originLocked = false;
    bool     _headingSeeded = false;
    uint32_t _lastImuMs = 0;
    bool     _haveLastFix = false;
    double   _lastFixLat = 0, _lastFixLon = 0;
    uint32_t _lastFixMs = 0;
    uint32_t _lastAcceptedPositionMs = 0;
    uint32_t _lastHeadingMs = 0;
    uint32_t _lastRtcmMs = 0;
    uint32_t _seededAtMs = 0;       // millis() момента seed'а; в первые 3 сек не верим GPS-course
    bool _fieldSafetyMode = false;
    float _antennaForwardOffsetM = 0;
    float _antennaLeftOffsetM = 0;
    bool _antennaCorrectionEnabled = true;
    uint32_t _lastHeadingOutputMs = 0;
    uint32_t _lastHeadingJumpLogMs = 0;
    bool _courseRotatingInPlace = false;
    float _courseCommandedLinearMps = 0;
    bool _courseWindowValid = false;
    float _courseWindowStartX = 0;
    float _courseWindowStartY = 0;
    uint32_t _courseWindowStartMs = 0;
    uint32_t _courseWindowAccumMs = 0;
    float _courseGyroAccumDeg = 0;

    // EKF
    RtkEkf _ekf;
    uint32_t _lastPredictMs = 0;
    float _lastSpeedLMps = 0;
    float _lastSpeedRMps = 0;
    float _lastYawRateRadps = 0;
};
