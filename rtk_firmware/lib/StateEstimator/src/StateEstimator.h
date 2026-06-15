// StateEstimator.h - Sunray-style GPS+heading fusion. MIT. Алгоритмы из Sunray (GPLv3).

#pragma once
#include <Arduino.h>
#include "RtkEkf.h"

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
    float   x = 0, y = 0;
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

    // apply new raw PVT message
    void onPvt(uint32_t nowMs,
               int32_t lat_e7, int32_t lon_e7, int32_t height_mm,
               int32_t hAcc_mm, int32_t vAcc_mm,
               int32_t gSpeed_mmps, int32_t headMot_deg_e5,
               int fixType, int carrierSol, bool diffSoln,
               int numSv, float pDop);

    // apply new RXM-RTCM info (F9P decode side)
    void onRtcmInfo(uint32_t nowMs, int lastType, int msgCount, int crcFail);

    // Одометрия от hoverboard feedback: speedL_meas, speedR_meas (в единицах платы).
    // Внутри переводим в м/с через ROVER_WHEEL_CIRCUM_M и используем для EKF predict.
    void onHoverboardFeedback(uint32_t nowMs, int speedL_meas, int speedR_meas);

    // периодический tick — обновляет возраст и фильтры
    void tick(uint32_t nowMs);

    // НОВОЕ: подать данные IMU (yaw rate, град/с) — интегрируется для краткосрочного курса.
    // Вызывать каждый tick ДО tick(). dtMs — фактический интервал.
    void onImu(uint32_t nowMs, float yawRateDps, bool imuFresh);

    // origin для перевода lat/lon → x/y
    bool setOrigin(double lat, double lon);
    void clearOrigin() { est.originSet = false; }
    void seedHeadingDeg(float headingDeg);   // абсолютная калибровка курса (CAL_HEADING)

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
    static constexpr float kGpsHeadingMinMpsActive = 0.03f;
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

    bool     _originLocked = false;
    bool     _headingSeeded = false;
    uint32_t _lastImuMs = 0;
    bool     _haveLastFix = false;
    double   _lastFixLat = 0, _lastFixLon = 0;
    uint32_t _lastFixMs = 0;
    uint32_t _lastAcceptedPositionMs = 0;
    uint32_t _lastHeadingMs = 0;

    // EKF
    RtkEkf _ekf;
    uint32_t _lastPredictMs = 0;
    float _lastSpeedLMps = 0;
    float _lastSpeedRMps = 0;
    float _lastYawRateRadps = 0;
};
