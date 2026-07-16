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

enum SafetyMode : uint8_t {
    SAFETY_MODE_NORMAL = 0,
    SAFETY_MODE_FIELD,
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
    uint32_t publishedPvtTimestampMs = 0u;
    uint32_t currentLoopGeneration = 0u;
    uint32_t rtcmAgeMs = 0xFFFFFFFFu;
    // PVT staleness threshold the caller wants enforced this tick.
    // Defaults to SAFE_PVT_AGE_MS for normal navigation. Auto-alignment
    // sets a looser value (ALIGN_MAX_PVT_AGE_MS) so a single PVT jitter
    // past 1 s does not abort the run. RTCM is still gated by
    // SAFE_RTK_AGE_MS.
    uint32_t maxPvtAgeMs = 0;
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
    // True only while AUTO_ALIGN_HEADING_BY_RTK is actively collecting the
    // displacement used to establish the first trusted heading.
    bool rtkAlignmentActive = false;
    // Narrow START_SETTLE recovery: rover.cpp sets this only for a FIXED,
    // hAcc-valid PVT published in this loop while motors are still zero.
    bool alignStartupFreshPvtRecovery = false;
    // True if estimator heading is valid for navigation. Combines the
    // "trusted" sources: BNO085 absolute OK, manual trust flag, and
    // RTK-motion alignment. Set by rover.cpp each tick.
    bool headingTrustedForNav = false;
    // True when the current heading trust comes from IMU (absolute OK or
    // manual trust). When false the navigation does not depend on BNO085
    // being live, so IMU staleness must not be fatal. Used to gate the
    // imu_stale Safety check.
    bool headingUsesImu = false;
    // True while the rover is actively rotating (turn-in-place or in-place
    // corner turn). hAcc на подвижной платформе кратковременно прыгает до
    // 2-3 см при повороте корпуса — это нормальный F9P jitter, не деградация
    // фикса. Safety в этом случае даёт короткий tolerance на hAcc-spike
    // вместо мгновенного HOLD/ESTOP.
    bool rotatingInPlace = false;
    bool motorMotionCommanded = false;
    bool motorFeedbackAlive = true;
    bool motorHardwareFault = false;
    bool internalFinite = true;
    bool routeStateCritical = false;
    bool boundaryAndZoneAllowed = true;
};

class Safety {
public:
    void begin();
    void tick(uint32_t nowMs, const SafetyInput& in, const StateEstimator& est, const Imu& imu);
    void setMode(SafetyMode mode);
    SafetyMode mode() const { return _mode; }
    const char* modeName() const;
    static const char* levelName(SafetyLevel level);
    SafetyLevel level() const { return _level; }
    bool allowMotion() const { return _level == SAFETY_OK || _level == SAFETY_DEGRADED; }
    bool allowNav()    const { return _level == SAFETY_OK || _level == SAFETY_DEGRADED; }
    bool requireOrigin() const { return _level == SAFETY_OK; }
    const char* reason() const { return _reason; }
    const SafetyInput& lastInput() const { return _lastInput; }
    SafetyLevel lastCandidateLevel() const { return _lastCandidateLevel; }
    const char* lastCandidateReason() const { return _lastCandidateReason; }
    uint32_t statusGeneration() const { return _statusGeneration; }
    uint32_t evaluatedPvtTimestampMs() const {
        return _evaluatedPvtTimestampMs;
    }
    uint32_t pvtAgeAtEvaluationMs() const {
        return _pvtAgeAtEvaluationMs;
    }
    uint32_t evaluatedLoopGeneration() const {
        return _evaluatedLoopGeneration;
    }

private:
    SafetyLevel _level = SAFETY_ESTOP;
    const char* _reason = "boot";
    SafetyMode _mode = SAFETY_MODE_NORMAL;
    bool _inited = false;
    bool _evaluated = false;
    uint32_t _recoverySinceMs = 0;
    uint32_t _skyBadSinceMs = 0;
    SafetyInput _lastInput{};
    SafetyLevel _lastCandidateLevel = SAFETY_ESTOP;
    const char* _lastCandidateReason = "boot";
    uint32_t _statusGeneration = 0u;
    uint32_t _evaluatedPvtTimestampMs = 0u;
    uint32_t _pvtAgeAtEvaluationMs = 0xFFFFFFFFu;
    uint32_t _evaluatedLoopGeneration = 0u;

    void applyCandidate(uint32_t nowMs, SafetyLevel level, const char* reason,
                        const SafetyInput& in);
    void changeLevel(SafetyLevel level, const char* reason, const SafetyInput& in);
};
