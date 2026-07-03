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
    uint32_t rtcmAgeMs = 0xFFFFFFFFu;
    uint32_t acceptedPositionAgeMs = 0xFFFFFFFFu;
    uint32_t headingAgeMs = 0xFFFFFFFFu;
    uint16_t rejectedPositionFixes = 0;
    bool originLocked = false;
    bool routeReady = false;
    // True while Serial AUTO_ALIGN_HEADING_BY_RTK is running. Lets the
    // motion survive a missing WebSocket link so the operator can debug
    // via Serial Monitor alone. WebSocket-issued alignments must NOT
    // set this — they should still abort on WS disconnect.
    bool serialDebugMotion = false;
    // True if estimator heading is valid for navigation. Combines the
    // "trusted" sources: BNO085 absolute OK, manual trust flag, and
    // RTK-motion alignment. Set by rover.cpp each tick.
    bool headingTrustedForNav = false;
    // True when the current heading trust comes from IMU (absolute OK or
    // manual trust). When false the navigation does not depend on BNO085
    // being live, so IMU staleness must not be fatal. Used to gate the
    // imu_stale Safety check.
    bool headingUsesImu = false;
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
