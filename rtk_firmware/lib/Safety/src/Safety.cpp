// Safety.cpp - MIT.

#include "Safety.h"
#include "RtkConfig.h"

void Safety::begin() {
    _level = SAFETY_ESTOP;
    _reason = "boot";
    _inited = true;
}

void Safety::tick(uint32_t nowMs, const SafetyInput& in, const StateEstimator& est, const Imu& imu) {
    if (!_inited) return;
    auto set = [&](SafetyLevel l, const char* r) { _level = l; _reason = r; };

    // 1. жёсткие failsafe — всегда побеждают. ws_disconnected is NOT
    // fatal when the operator is running a Serial-only debug session
    // (e.g. AUTO_ALIGN_HEADING_BY_RTK with no app). STOP / ESTOP / HOLD
    // are still reachable via serialDebugMotion=false after alignment
    // ends, so the gate is tight whenever motion is real and attributable.
    if (!in.wsConnected && !in.serialDebugMotion) {
        set(SAFETY_ESTOP, "ws_disconnected");
        return;
    }
    // WebSocket callbacks run asynchronously. A callback can store lastCmdMs
    // a few milliseconds after the main loop captured nowMs. Guard the
    // subtraction to avoid unsigned underflow and an immediate false timeout.
    if (in.navRequested && in.lastCmdMs != 0 &&
        nowMs >= in.lastCmdMs &&
        (nowMs - in.lastCmdMs) > SAFE_NAV_TIMEOUT_MS) {
        set(SAFETY_ESTOP, "nav_cmd_timeout");
        return;
    }
    if (!in.navRequested) {
        // During serialDebugMotion we may still want motion even when
        // navRequested=false (alignment drives motor directly, not via
        // the follower). Let the rest of the gates apply.
        if (!in.serialDebugMotion) {
            set(SAFETY_OK, "manual_ok");
            return;
        }
    }
    // PVT staleness threshold is per-tick (in.maxPvtAgeMs) so the caller
    // can relax it for AUTO_ALIGN_HEADING_BY_RTK while keeping the strict
    // SAFE_PVT_AGE_MS for normal navigation.
    const uint32_t maxPvt = in.maxPvtAgeMs == 0 ? SAFE_PVT_AGE_MS : in.maxPvtAgeMs;
    if (in.pvtAgeMs > maxPvt) {
        set(SAFETY_ESTOP, "pvt_stale");
        return;
    }
    if (in.navRequested && in.rtcmAgeMs > SAFE_RTK_AGE_MS) {
        set(SAFETY_ESTOP, "rtcm_stale");
        return;
    }
    if (in.numSv < SAFE_NUM_SV || in.pDop > SAFE_PDOP_MAX) {
        set(SAFETY_HOLD, "sky_bad");
        return;
    }

    // 2. в навигации — IMU stale is only fatal when navigation actually
    //    depends on a live IMU. RTK-motion aligned heading does NOT
    //    require fresh BNO085; the EKF integrates gyro in estimator.cpp
    //    and `headingAgeMs` is the real staleness metric for that path.
    if (in.navRequested && in.headingUsesImu) {
        if (imu.ageMs(nowMs) > SAFE_IMU_AGE_MS) {
            set(SAFETY_ESTOP, "imu_stale");
            return;
        }
    }

    // 3. проверка RTK fix / float + accuracy
    if (in.navRequested) {
        if (!in.originLocked) {
            set(SAFETY_HOLD, "no_origin");
            return;
        }
        if (!in.routeReady) {
            set(SAFETY_HOLD, "no_route");
            return;
        }
        // Heading trust gate. Three acceptable paths:
        //   - BNO085 absolute OK (preferred, canUseAbsoluteYawForNav)
        //   - manual trust flag (debug escape hatch)
        //   - RTK-motion alignment this boot AND estimator heading still
        //     fresh (set by rover.cpp via headingTrustedForNav)
        // If none of these holds we refuse to drive.
        if (!in.headingTrustedForNav) {
            set(SAFETY_HOLD, "heading_not_trusted");
            return;
        }
        if (in.acceptedPositionAgeMs > SAFE_ACCEPTED_POS_AGE_MS) {
            set(SAFETY_HOLD, "position_stale");
            return;
        }
        if (in.rejectedPositionFixes > SAFE_REJECTED_POSITION_FIXES_MAX) {
            set(SAFETY_HOLD, "gps_jump");
            return;
        }
        if (in.headingAgeMs > SAFE_HEADING_AGE_MS) {
            set(SAFETY_HOLD, "heading_stale");
            return;
        }
        if (in.sol == SOL_INVALID) {
            set(SAFETY_HOLD, "rtk_invalid");
            return;
        }
        if (in.sol == SOL_FIXED) {
            if (in.hAcc > SAFE_HACC_FIXED_M) {
                set(SAFETY_HOLD, "rtk_fixed_hacc_gt_2cm");
            } else {
                set(SAFETY_OK, "rtk_fixed");
            }
        } else if (in.sol == SOL_FLOAT) {
            // FLOAT с hAcc<=4см — вполне достаточно для езды по точкам.
            // Раньше тут стоял HOLD, но FIXED может не приходить минутами из-за
            // узких заполнений неба или отражений, а FLOAT уже даёт 2-4см.
            // Едем в DEGRADED (медленнее, но едем).
            if (in.hAcc > 0.05f) {
                set(SAFETY_HOLD, "rtk_float_hacc_gt_5cm");
            } else {
                set(SAFETY_DEGRADED, "rtk_float_ok");
            }
        }
        return;
    }

    // 4. serialDebugMotion path: alignment drives motors directly
    // (bypassing the follower). We still demand RTK is alive; the
    // alignment state machine aborts itself on RTK loss independently.
    if (in.serialDebugMotion) {
        if (in.sol == SOL_INVALID) {
            set(SAFETY_HOLD, "manual_no_fix");
        } else if (in.sol == SOL_FLOAT) {
            if (in.hAcc > 0.05f) {
                set(SAFETY_HOLD, "manual_float_hacc_gt_5cm");
            } else {
                set(SAFETY_OK, "manual_float");
            }
        } else if (in.sol == SOL_FIXED) {
            if (in.hAcc > SAFE_HACC_FIXED_M) {
                set(SAFETY_HOLD, "manual_fixed_hacc_gt_2cm");
            } else {
                set(SAFETY_OK, "manual_fixed");
            }
        }
        return;
    }

    // 5. ручной режим — IMU опционален, RTK не нужен
    if (in.sol == SOL_INVALID) {
        set(SAFETY_HOLD, "manual_no_fix");
    } else if (in.sol == SOL_FLOAT) {
        set(SAFETY_DEGRADED, "manual_float");
    } else {
        set(SAFETY_OK, "manual_ok");
    }
}
