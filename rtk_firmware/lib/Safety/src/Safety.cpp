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

    // 1. жёсткие failsafe — всегда побеждают
    if (!in.wsConnected) {
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
        set(SAFETY_OK, "manual_ok");
        return;
    }
    if (in.pvtAgeMs > SAFE_PVT_AGE_MS) {
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

    // 2. в навигации — IMU must be fresh
    if (in.navRequested) {
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
        if (!ImuMath::canUseAbsoluteYawForNav(
                imu.headingState(),
                imu.yawAbsoluteValid(),
                imu.yawAccRad(),
                imu.yawAgeMs(nowMs),
                SAFE_IMU_AGE_MS)) {
            set(SAFETY_HOLD, "imu_no_absolute_heading");
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

    // 4. ручной режим — IMU опционален, RTK не нужен
    if (in.sol == SOL_INVALID) {
        set(SAFETY_HOLD, "manual_no_fix");
    } else if (in.sol == SOL_FLOAT) {
        set(SAFETY_DEGRADED, "manual_float");
    } else {
        set(SAFETY_OK, "manual_ok");
    }
}
