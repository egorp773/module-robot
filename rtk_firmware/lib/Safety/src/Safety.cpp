// Safety.cpp - MIT.

#include "Safety.h"
#include "RtkConfig.h"

void Safety::begin() {
    _level = SAFETY_ESTOP;
    _reason = "boot";
    _inited = true;
}

// Helper: link-watchdog reasons that must NOT abort Serial/debug
// autonomous motion (AUTO_ALIGN_HEADING_BY_RTK / GO_FORWARD / GO_L_SHAPE_DEBUG
// / GO_SQUARE_DEBUG). These commands are self-contained — they don't
// stream joystick bytes; lastCmdMs simply stops being refreshed while
// the command runs. The bug they used to die on:
//   [SERIAL-MOTION] safety abort level=3 reason=nav_cmd_timeout ageSinceBegin=144
// даже когда RTK/heading/hAcc в норме.
static bool isIgnorableLinkWatchdogReason(const char* reason) {
    if (!reason) return false;
    return strcmp(reason, "ws_disconnected") == 0 ||
           strcmp(reason, "nav_cmd_timeout") == 0 ||
           strcmp(reason, "manual_cmd_timeout") == 0;
}

// Helper: для serial/debug autonomous motion мягко относимся к
// КРАТКОВРЕМЕННОМУ pvt_stale, если RTK реально был зафиксирован
// недавно, hAcc держится в норме и heading всё ещё trusted. RTK
// чипы иногда пропускают один PVT, особенно при переключении между
// SOL_FLOAT и SOL_FIXED; без этого толеранса тест валится через
// 170-300 мс после старта.
// Жёсткие причины (STOP / hard fault / heading_stale > SAFE_HEADING_AGE_MS
// / motor feedback lost / GPS jump) — НЕ толерируем, они остаются
// фатальными через обычный Safety::tick().
static bool isSoftToleratedForSerialMotion(const SafetyInput& in,
                                            const char* reason) {
    if (!reason) return false;
    if (!in.serialDebugMotion) return false;
    if (strcmp(reason, "pvt_stale") != 0) return false;
    // Допуск только если heading всё ещё доверяем и hAcc в норме.
    if (!in.headingTrustedForNav) return false;
    if (in.hAcc > 0.05f) return false;
    // Допуск только в коротком окне после последнего приёма PVT.
    if (in.pvtAgeMs == 0xFFFFFFFFu) return false;
    constexpr uint32_t SOFT_PVT_TOLERANCE_MS = 1500u;
    return in.pvtAgeMs <= SOFT_PVT_TOLERANCE_MS;
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
    // Link-watchdog (`nav_cmd_timeout`, `manual_cmd_timeout`) is real
    // safety only when motion is streaming external bytes (WS joystick,
    // manual M,left,right). For Serial/debug autonomous motion
    // (`serialDebugMotion == true`) the command was issued once from
    // Serial/WsServer and runs to completion on its own; the link
    // going stale mid-command is not a fault, it's the normal lifetime
    // of that command. So we downgrade to SAFETY_OK with a transient
    // reason so the operator sees we noticed but the run continues.
    if (in.serialDebugMotion) {
        const char* prev = _reason;
        if (prev && isIgnorableLinkWatchdogReason(prev)) {
            Serial.printf("[SERIAL-MOTION] ignoring safety reason=%s during serial motion\n", prev);
            set(SAFETY_OK, "serial_motion_link_idle");
            return;
        }
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
        if (isSoftToleratedForSerialMotion(in, "pvt_stale")) {
            // Кратковременный pvt_stale во время serial/debug motion
            // толерируем: heading ещё доверяем, hAcc в норме, PVT-пропуск
            // не длиннее 1.5 с. Держим SAFETY_DEGRADED, моторы не стопим.
            Serial.printf("[SERIAL-MOTION] warning pvt_stale tolerated age=%u hAcc=%.3f during serial debug\n",
                          (unsigned)in.pvtAgeMs, (double)in.hAcc);
            set(SAFETY_DEGRADED, "pvt_soft_stale");
            return;
        }
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
                // Turn-in-place tolerance: при повороте корпуса hAcc
                // кратковременно подскакивает до 2-3 см. Это нормальный
                // F9P jitter на подвижной платформе, не потеря FIXED.
                // Разрешаем до 5 см в этом окне, НЕ валим в HOLD —
                // иначе follower отменяется на каждом corner turn.
                if (in.rotatingInPlace && in.hAcc <= 0.05f) {
                    Serial.printf("[SAFETY] tolerate hAcc=%.3f during turn-in-place\n",
                                  (double)in.hAcc);
                    set(SAFETY_OK, "rtk_fixed_turn_jitter");
                } else {
                    set(SAFETY_HOLD, "rtk_fixed_hacc_high");
                }
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
                set(SAFETY_HOLD, "manual_fixed_hacc_high");
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
