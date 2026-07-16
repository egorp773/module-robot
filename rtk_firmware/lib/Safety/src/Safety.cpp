// Safety.cpp - runtime NORMAL/FIELD safety policy. MIT.

#include "Safety.h"
#include "RtkConfig.h"
#include "PvtSafetyTimeline.h"
#include <math.h>
#include <string.h>

const char* Safety::levelName(SafetyLevel level) {
    switch (level) {
        case SAFETY_OK: return "OK";
        case SAFETY_DEGRADED: return "DEGRADED";
        case SAFETY_HOLD: return "HOLD";
        case SAFETY_ESTOP: return "ESTOP";
    }
    return "UNKNOWN";
}

const char* Safety::modeName() const {
    return _mode == SAFETY_MODE_FIELD ? "FIELD" : "NORMAL";
}

static const char* solName(SolType sol) {
    switch (sol) {
        case SOL_FIXED: return "FIXED";
        case SOL_FLOAT: return "FLOAT";
        default: return "INVALID";
    }
}

void Safety::begin() {
    // FIELD is deliberately runtime-only and never restored from storage.
    _mode = SAFETY_MODE_NORMAL;
    _level = SAFETY_ESTOP;
    _reason = "boot";
    _inited = true;
    _evaluated = false;
    _recoverySinceMs = 0;
    _skyBadSinceMs = 0;
    _lastInput = SafetyInput{};
    _lastCandidateLevel = SAFETY_ESTOP;
    _lastCandidateReason = "boot";
    _statusGeneration = 0u;
    _evaluatedPvtTimestampMs = 0u;
    _pvtAgeAtEvaluationMs = 0xFFFFFFFFu;
    _evaluatedLoopGeneration = 0u;
}

void Safety::setMode(SafetyMode mode) {
    _mode = mode;
    _evaluated = false;
    _recoverySinceMs = 0;
    _skyBadSinceMs = 0;
}

void Safety::changeLevel(SafetyLevel level, const char* reason,
                         const SafetyInput& in) {
    const SafetyLevel old = _level;
    _level = level;
    _reason = reason;
    if (old == level) return;
    Serial.printf("[SAFETY_CHANGE] mode=%s oldLevel=%s newLevel=%s reason=%s "
                  "pvtAge=%u headingAge=%u sol=%s hAcc=%.3f rtcmAge=%u\n",
                  modeName(), levelName(old), levelName(level),
                  reason ? reason : "-", (unsigned)in.pvtAgeMs,
                  (unsigned)in.headingAgeMs, solName(in.sol),
                  (double)in.hAcc, (unsigned)in.rtcmAgeMs);
}

void Safety::applyCandidate(uint32_t nowMs, SafetyLevel candidate,
                            const char* reason, const SafetyInput& in) {
    if (!_evaluated) {
        _evaluated = true;
        _recoverySinceMs = 0;
        changeLevel(candidate, reason, in);
        return;
    }

    const bool retainedPvtStale =
        (_level == SAFETY_HOLD || _level == SAFETY_ESTOP) &&
        _reason != nullptr && strcmp(_reason, "pvt_stale") == 0;
    if (pvtsafety::applyFreshPvtStatusImmediately(
            in.alignStartupFreshPvtRecovery,
            retainedPvtStale,
            candidate <= SAFETY_DEGRADED)) {
        _recoverySinceMs = 0;
        changeLevel(candidate, reason, in);
        return;
    }

    // Worsening is immediate. Same-level reason changes are retained for
    // SAFETY_STATUS but are intentionally not logged as state transitions.
    if (candidate >= _level) {
        _recoverySinceMs = 0;
        if (candidate == _level) {
            _reason = reason;
        } else {
            changeLevel(candidate, reason, in);
        }
        return;
    }

    // A transition which would permit motion again, and DEGRADED -> OK,
    // requires two continuous seconds of healthy input.
    const bool motionRecovery =
        (_level >= SAFETY_HOLD && candidate <= SAFETY_DEGRADED) ||
        (_level == SAFETY_DEGRADED && candidate == SAFETY_OK);
    if (!motionRecovery) {
        _recoverySinceMs = 0;
        changeLevel(candidate, reason, in);
        return;
    }
    if (_recoverySinceMs == 0) _recoverySinceMs = nowMs;
    if ((uint32_t)(nowMs - _recoverySinceMs) >= FIELD_RECOVERY_STABLE_MS) {
        _recoverySinceMs = 0;
        changeLevel(candidate, reason, in);
    }
}

void Safety::tick(uint32_t nowMs, const SafetyInput& in,
                  const StateEstimator& est, const Imu& imu) {
    (void)est;
    if (!_inited) return;
    _lastInput = in;
    ++_statusGeneration;
    if (_statusGeneration == 0u) ++_statusGeneration;
    _evaluatedPvtTimestampMs = in.publishedPvtTimestampMs;
    _pvtAgeAtEvaluationMs = in.pvtAgeMs;
    _evaluatedLoopGeneration = in.currentLoopGeneration;

    SafetyLevel level = SAFETY_OK;
    const char* reason = "manual_ok";
    auto choose = [&](SafetyLevel l, const char* r) {
        if (l > level) { level = l; reason = r; }
        else if (l == level && level != SAFETY_OK) { reason = r; }
    };

    const bool field = _mode == SAFETY_MODE_FIELD;
    const bool motion = in.navRequested || in.serialDebugMotion ||
                        in.rtkAlignmentActive ||
                        in.motorMotionCommanded;
    const bool ignoreSerialLink = field && in.serialDebugMotion;

    // Hard faults are never relaxed by FIELD.
    if (!in.internalFinite) choose(SAFETY_ESTOP, "internal_nonfinite");
    if (in.motorHardwareFault) choose(SAFETY_ESTOP, "motor_hardware_fault");
    if (motion && !in.motorFeedbackAlive) choose(SAFETY_ESTOP, "motor_feedback_lost");
    if (in.routeStateCritical) choose(SAFETY_ESTOP, "route_state_critical");
    if (in.navRequested && !in.boundaryAndZoneAllowed)
        choose(SAFETY_ESTOP, "boundary_or_forbidden_zone");

    // A missing WebSocket is not a fault while completely idle. Once any
    // motion is requested, the normal link policy applies.
    if (!in.wsConnected && !ignoreSerialLink && motion)
        choose(SAFETY_ESTOP, "ws_disconnected");
    if (in.navRequested && in.lastCmdMs != 0 && nowMs >= in.lastCmdMs &&
        (nowMs - in.lastCmdMs) > SAFE_NAV_TIMEOUT_MS && !ignoreSerialLink)
        choose(SAFETY_ESTOP, "nav_cmd_timeout");

    if (field && motion) {
        if (in.pvtAgeMs > FIELD_PVT_ESTOP_MS)
            choose(SAFETY_ESTOP, "pvt_stale");
        else if (in.pvtAgeMs > FIELD_PVT_HOLD_MS)
            choose(SAFETY_HOLD, "pvt_stale");
        else if (in.pvtAgeMs > FIELD_PVT_DEGRADED_MS)
            choose(SAFETY_DEGRADED, "pvt_stale");

        if (in.acceptedPositionAgeMs > FIELD_POSITION_ESTOP_MS)
            choose(SAFETY_ESTOP, "position_unavailable");
        else if (in.acceptedPositionAgeMs > FIELD_POSITION_HOLD_MS)
            choose(SAFETY_HOLD, "position_stale");

        if (!in.rtkAlignmentActive) {
            if (in.headingAgeMs > FIELD_HEADING_ESTOP_MS)
                choose(SAFETY_ESTOP, "heading_unavailable");
            else if (in.headingAgeMs > FIELD_HEADING_HOLD_MS)
                choose(SAFETY_HOLD, "heading_stale");
        }
    } else if (motion) {
        const uint32_t maxPvt = in.maxPvtAgeMs == 0 ? SAFE_PVT_AGE_MS
                                                    : in.maxPvtAgeMs;
        if (in.pvtAgeMs > maxPvt) choose(SAFETY_ESTOP, "pvt_stale");
        if (in.acceptedPositionAgeMs > SAFE_ACCEPTED_POS_AGE_MS)
            choose(SAFETY_HOLD, "position_stale");
        if (!in.rtkAlignmentActive && in.headingAgeMs > SAFE_HEADING_AGE_MS)
            choose(SAFETY_HOLD, "heading_stale");
    }

    if ((in.navRequested || in.rtkAlignmentActive) &&
        in.rtcmAgeMs > SAFE_RTK_AGE_MS)
        choose(SAFETY_ESTOP, "rtcm_stale");

    const bool skyBad = in.numSv < SAFE_NUM_SV ||
                        !isfinite(in.pDop) || in.pDop > SAFE_PDOP_MAX;
    if (field && motion) {
        if (skyBad) {
            if (_skyBadSinceMs == 0) _skyBadSinceMs = nowMs;
            if ((uint32_t)(nowMs - _skyBadSinceMs) > FIELD_SKY_BAD_HOLD_MS)
                choose(SAFETY_HOLD, "sky_bad");
        } else {
            _skyBadSinceMs = 0;
        }
    } else {
        _skyBadSinceMs = 0;
        if (skyBad && motion) choose(SAFETY_HOLD, "sky_bad");
    }

    if (in.navRequested && !in.rtkAlignmentActive &&
        in.headingUsesImu && !field &&
        imu.ageMs(nowMs) > SAFE_IMU_AGE_MS)
        choose(SAFETY_ESTOP, "imu_stale");

    if (in.navRequested) {
        if (!in.rtkAlignmentActive) {
            if (!in.originLocked) choose(SAFETY_HOLD, "no_origin");
            if (!in.routeReady) choose(SAFETY_HOLD, "no_route");
            if (!in.headingTrustedForNav)
                choose(SAFETY_HOLD, "heading_not_trusted");
        }
        const uint16_t jumpLimit = field ? FIELD_GPS_JUMP_CONSECUTIVE
                                         : SAFE_REJECTED_POSITION_FIXES_MAX;
        if (in.rejectedPositionFixes >= jumpLimit)
            choose(SAFETY_HOLD, "gps_jump");

        if (in.sol == SOL_INVALID) {
            choose(SAFETY_HOLD, "rtk_invalid");
        } else if (in.sol == SOL_FIXED) {
            const float limit = field ? FIELD_HACC_FIXED_M : SAFE_HACC_FIXED_M;
            if (in.hAcc > limit) choose(SAFETY_HOLD, "rtk_fixed_hacc_high");
            else if (field && in.hAcc > SAFE_HACC_FIXED_M)
                choose(SAFETY_DEGRADED, "rtk_fixed_reduced_speed");
        } else if (in.sol == SOL_FLOAT) {
            const float limit = field ? FIELD_HACC_FLOAT_M : SAFE_HACC_FLOAT_M;
            if (in.hAcc > limit) choose(SAFETY_HOLD, "rtk_float_hacc_high");
            else choose(SAFETY_DEGRADED, "rtk_float_reduced_speed");
        }
        if (level == SAFETY_OK)
            reason = in.rtkAlignmentActive ? "align_heading_pending"
                                           : "rtk_fixed";
    } else if (in.serialDebugMotion) {
        if (in.sol == SOL_INVALID) choose(SAFETY_HOLD, "manual_no_fix");
        else if (in.sol == SOL_FLOAT) {
            const float limit = field ? FIELD_HACC_FLOAT_M : SAFE_HACC_FLOAT_M;
            if (in.hAcc > limit) choose(SAFETY_HOLD, "manual_float_hacc_high");
            else choose(SAFETY_DEGRADED, "manual_float");
        } else {
            const float limit = field ? FIELD_HACC_FIXED_M : SAFE_HACC_FIXED_M;
            if (in.hAcc > limit) choose(SAFETY_HOLD, "manual_fixed_hacc_high");
            else if (field && in.hAcc > SAFE_HACC_FIXED_M)
                choose(SAFETY_DEGRADED, "manual_fixed_reduced_speed");
        }
        if (level == SAFETY_OK)
            reason = in.rtkAlignmentActive ? "align_heading_pending"
                                           : "manual_fixed";
    }

    _lastCandidateLevel = level;
    _lastCandidateReason = reason;
    applyCandidate(nowMs, level, reason, in);
}
