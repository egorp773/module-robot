// Safety.h - watchdog, e-stop. MIT.

#pragma once
#include <Arduino.h>
#include "StateEstimator.h"
#include "Imu.h"
#include "Route.h"

enum SafetyLevel : uint8_t {
    SAFETY_OK = 0,            // можно ехать
    SAFETY_DEGRADED,          // едем медленно (FLOAT)
    SAFETY_HOLD,              // стоим, RTK плохой
    SAFETY_ESTOP,             // жёсткий стоп
};

struct SafetyInput {
    bool wsConnected = false;
    uint32_t lastWsRxMs = 0;
    uint32_t lastCmdMs  = 0;     // любой M,left,right или ROUTE_/NAV_
    bool navRequested = false;
    SolType sol = SOL_INVALID;
    int numSv = 0;
    float pDop = 99;
    float hAcc = 999;
    uint32_t pvtAgeMs = 0xFFFFFFFFu;
    bool originLocked = false;
    bool routeReady = false;
};

class Safety {
public:
    void begin();
    void tick(uint32_t nowMs, const SafetyInput& in, const StateEstimator& est, const Imu& imu);
    SafetyLevel level() const { return _level; }
    bool allowMotion() const { return _level == SAFETY_OK || _level == SAFETY_DEGRADED; }
    bool allowNav()    const { return _level == SAFETY_OK || _level == SAFETY_DEGRADED; }
    bool requireOrigin() const { return _level == SAFETY_OK; }
    const char* reason() const { return _reason; }

private:
    SafetyLevel _level = SAFETY_ESTOP;
    const char* _reason = "boot";
    bool _inited = false;
};
