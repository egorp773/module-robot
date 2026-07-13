// rover.cpp - entry point. Rover side.
// Sunray-style: единый цикл с тасками по периоду, всё в loop().

#include <Arduino.h>
#include "RtkConfig.h"
#include "RoverDebug.h"
#include "StateEstimator.h"
#include "Imu.h"
#include "Gnss.h"
#include "RtcmLink.h"
#include "Route.h"
#include "Motor.h"
#include "Safety.h"
#include "WsServer.h"
#include "NavMath.h"
#include <WiFi.h>

// --- global objects ---
HardwareSerial F9pSerial(1);
HardwareSerial MotorSerial(2);   // hoverboard UART (Serial2, RX16/TX17)

StateEstimator  g_est;
Imu             g_imu;
Gnss            g_gnss;
RtcmLink        g_rtcm;
Route           g_route;
Motor           g_motor;
Safety          g_safety;
WsServer        g_ws;

static bool g_logEnabled = true;   // LOG,0 / LOG,1 — гасит периодический лог

// --- Heading source / RTK-motion alignment state ---
// IMU absolute heading has been demoted to optional. For navigation we now
// trust a heading aligned from RTK motion (forward drive → atan2(dx,dy)).
// This struct survives the lifetime of one boot, is never persisted to NVS,
// and is cleared on reboot, on CLEAR_HEADING_TRUST, on STOP, or whenever
// AUTO_ALIGN_HEADING_BY_RTK fails.
enum class AlignState : uint8_t {
    IDLE = 0,
    RUNNING,
    OK,
    ERR,
};
static const char* alignStateName(AlignState s) {
    switch (s) {
        case AlignState::IDLE: return "IDLE";
        case AlignState::RUNNING: return "RUNNING";
        case AlignState::OK: return "OK";
        case AlignState::ERR: return "ERR";
    }
    return "IDLE";
}
enum class HeadingSource : uint8_t {
    NONE = 0,
    IMU_ABSOLUTE,
    IMU_MANUAL,
    RTK_MOTION_ALIGNED,
    // Heading seeded from RTK motion alignment, then continuously
    // tracked by integrating the BNO085 gyro through the StateEstimator.
    // `headingDeg` reflects the live estimator heading (rotates with
    // the rover); `lastAlignHeading` keeps the original RTK-only value
    // for diagnostics.
    RTK_MOTION_ALIGNED_PLUS_IMU,
};
static const char* headingSourceName(HeadingSource s) {
    switch (s) {
        case HeadingSource::NONE: return "NONE";
        case HeadingSource::IMU_ABSOLUTE: return "IMU_ABSOLUTE";
        case HeadingSource::IMU_MANUAL: return "IMU_MANUAL";
        case HeadingSource::RTK_MOTION_ALIGNED: return "RTK_MOTION_ALIGNED";
        case HeadingSource::RTK_MOTION_ALIGNED_PLUS_IMU: return "RTK_MOTION_ALIGNED_PLUS_IMU";
    }
    return "NONE";
}
struct HeadingMgr {
    bool trusted = false;
    HeadingSource source = HeadingSource::NONE;
    float headingDeg = 0.0f;
    uint32_t trustedAtMs = 0;
    AlignState alignState = AlignState::IDLE;
    uint32_t alignStartedAtMs = 0;
    float lastAlignHeading = 0.0f;
    float lastAlignDist = 0.0f;
    float lastAlignDx = 0.0f;
    float lastAlignDy = 0.0f;
    float lastAlignHAcc = 0.0f;
    const char* lastAlignError = nullptr;
} g_heading;

// --------------------------------------------------------------------
// Serial-initiated motion registry
// --------------------------------------------------------------------
//
// Both AUTO_ALIGN_HEADING_BY_RTK (when issued from Serial) and the
// GO_FORWARD / GO_NORTH test commands need motors to spin even without a
// connected WebSocket client. We collect them in one struct and expose
// `active` to Safety::tick() via SafetyInput.serialDebugMotion.
//
// The flag clears automatically when:
//   - the originating test finishes (alignment OK/ERR, follower arrival)
//   - STOP is received
//   - safety falls into ESTOP or HOLD
//   - a per-source hard timeout (10 s for GO_*; the alignment has its own)
enum class SerialMotionSource : uint8_t {
    NONE = 0,
    AUTO_ALIGN,
    GO_FORWARD,
    GO_NORTH,
    GO_L_SHAPE,
    GO_SQUARE,
};

static const char* serialMotionSourceName(SerialMotionSource s) {
    switch (s) {
        case SerialMotionSource::AUTO_ALIGN: return "AUTO_ALIGN";
        case SerialMotionSource::GO_FORWARD: return "GO_FORWARD";
        case SerialMotionSource::GO_NORTH:  return "GO_NORTH";
        case SerialMotionSource::GO_L_SHAPE: return "GO_L_SHAPE";
        case SerialMotionSource::GO_SQUARE: return "GO_SQUARE";
        default: return "NONE";
    }
}

struct SerialMotion {
    bool active = false;
    SerialMotionSource source = SerialMotionSource::NONE;
    uint32_t startedAtMs = 0;
    uint32_t safetyArmedAtMs = 0;
    uint32_t timeoutMs = 0;
} g_serialMotion;

// --------------------------------------------------------------------
// L-shape test statistics
// --------------------------------------------------------------------
//
// Captures all the metrics needed to assess a real L-shape test run
// (planned p0/p1/p2; actual start/wp1/finish; per-segment chord / path
// length; cross-track / heading-error maxima; RTK and IMU age maxima;
// safety transitions). Built and updated entirely from existing
// follower and Estimator state — does not touch the follower, the PID /
// pure-pursuit stage, motor low-level or the route math.
//
// Lifecycle:
//   begin() on GO_L_SHAPE_DEBUG command — fills plan, captures actual
//                          start pose, prints START / PLAN.
//   onTick() each loop()  — advances pathLen, cross-track maxima,
//                          RTK / IMU maxima. If debug=true, also prints
//                          a one-line snapshot every 200 ms.
//   onWpChanged()        — detects the 0 → 1 transition, captures
//                          actual wp1, prints WP1_REACHED.
//   finish(reason)       — captures final pose, computes summary,
//                          prints SUMMARY_* + CSV + VERDICT.
struct LShapeTestStats {
    bool active = false;
    bool debug = false;
    String lastResultReason;

    float firstPlan = 0;
    float turnPlan  = 0;
    float secondPlan = 0;
    float h0 = 0;
    float h1 = 0;

    float p0x = 0, p0y = 0;
    float p1x = 0, p1y = 0;
    float p2x = 0, p2y = 0;

    float startX = 0, startY = 0;
    float wp1X = NAN, wp1Y = NAN;
    float finishX = NAN, finishY = NAN;

    float headingStart  = 0;
    float headingWp1    = NAN;
    float headingFinish = NAN;

    uint32_t startedMs = 0;
    uint32_t wp1Ms     = 0;
    uint32_t finishedMs = 0;
    uint32_t lastPrintMs = 0;

    int lastWpIndex = -1;

    float lastX = NAN, lastY = NAN;
    float pathLen   = 0;
    float pathLen1  = 0;
    float pathLen2  = 0;

    float maxCross1 = 0;
    float maxCross2 = 0;
    float maxHeadingErr = 0;

    float    maxHAcc = 0;
    uint32_t maxPvtAge = 0;
    uint32_t maxRtcmAge = 0;
    uint32_t maxHeadingAge = 0;
    uint32_t maxRelYawAge = 0;
    uint32_t maxGyroAge = 0;
};

static LShapeTestStats g_ltest;

struct SquareTestStats {
    bool active = false;
    bool debug = false;
    String lastResultReason;

    float sidePlan = 0;
    float h0 = 0;
    float pX[5]{};
    float pY[5]{};
    float startX = 0;
    float startY = 0;
    float finishX = NAN;
    float finishY = NAN;
    float headingStart = 0;
    float headingFinish = NAN;
    uint32_t startedMs = 0;
    uint32_t finishedMs = 0;
    uint32_t lastPrintMs = 0;
    float pathLen = 0;
    float lastX = NAN;
    float lastY = NAN;
    float maxCross = 0;
    float maxHeadingErr = 0;
    float maxHAcc = 0;
    uint32_t maxPvtAge = 0;
    uint32_t maxRtcmAge = 0;
    uint32_t maxHeadingAge = 0;

    // === Per-segment metrics для SQTEST (4 стороны квадрата) ===
    // cornerActualX[i] / cornerActualY[i] — где робот реально оказался
    // при i-й вершине (0..3). cornerErr[i] = расстояние до плана.
    // segPathLen[i] / segMaxCross[i] / segTurnErr[i] — i-й сегмент.
    float cornerActualX[4]{NAN, NAN, NAN, NAN};
    float cornerActualY[4]{NAN, NAN, NAN, NAN};
    float cornerErr[4]{NAN, NAN, NAN, NAN};
    float segPathLen[4]{0, 0, 0, 0};
    float segMaxCross[4]{0, 0, 0, 0};
    float segTurnErr[4]{NAN, NAN, NAN, NAN};
};

static SquareTestStats g_sqtest;

static float lshapeDist2d(float ax, float ay, float bx, float by) {
    const float dx = bx - ax;
    const float dy = by - ay;
    return sqrtf(dx * dx + dy * dy);
}

// Signed cross-track (positive = point lies to the left of the
// direction A→B in the project convention x=East y=North).
static float lshapeSignedCross(float ax, float ay, float bx, float by, float px, float py) {
    const float vx = bx - ax;
    const float vy = by - ay;
    const float wx = px - ax;
    const float wy = py - ay;
    const float len = sqrtf(vx * vx + vy * vy);
    if (len < 0.001f) return 0.0f;
    return (vx * wy - vy * wx) / len;
}

static float lshapeAlongTrack(float ax, float ay, float bx, float by, float px, float py) {
    const float vx = bx - ax;
    const float vy = by - ay;
    const float wx = px - ax;
    const float wy = py - ay;
    const float len = sqrtf(vx * vx + vy * vy);
    if (len < 0.001f) return 0.0f;
    return (wx * vx + wy * vy) / len;
}

// Per-source hard timeouts. AUTO_ALIGN has its own ALIGN_TIMEOUT_MS;
// GO_* commands get a 10 second ceiling because they typically
// cover ~0.5 m at 0.18 m/s ≈ 3 s of motion, plus some startup slack.
static constexpr uint32_t SERIAL_GO_TIMEOUT_MS     = 10000u;
static constexpr uint32_t SERIAL_L_SHAPE_TIMEOUT_MS = 30000u;
// Безопасность включает safety только через 1 секунду после старта
// serialMotion (бывший 500мс). Раньше при первом тике после begin
// ещё не успевал освежиться lastCmdMs — safety видел "stale link"
// и убивал команду через 144-252 мс после старта.
static constexpr uint32_t SERIAL_MOTION_SAFETY_GRACE_MS = 1000u;

static void serialMotionBegin(SerialMotionSource src, uint32_t timeoutMs) {
    const uint32_t now = millis();
    g_serialMotion.active = true;
    g_serialMotion.source = src;
    g_serialMotion.startedAtMs = now;
    g_serialMotion.safetyArmedAtMs = now + SERIAL_MOTION_SAFETY_GRACE_MS;
    g_serialMotion.timeoutMs = timeoutMs;
    Serial.printf("[SERIAL-MOTION] begin source=%s timeoutMs=%u\n",
                  serialMotionSourceName(src), (unsigned)timeoutMs);
}

// Forward decls: debug test finalizers are defined inside roverdbg below.
// Used here to publish a partial summary if STOP interrupts a test.
namespace roverdbg {
void lShapeFinish(const char* reason);
void lShapeOnTick();
String handleGoSquareDebugLine(float sideM);
void squareFinish(const char* reason);
void precisionDebugOnTick();
bool precisionDebugActive();
// True если идёт corner turn / settle / post-turn-settle в LTEST/SQTEST.
// В эти моменты hAcc кратко прыгает — Safety делает tolerance на spike.
bool precisionIsRotating();
// Гасит g_precision.active. Вызывается строго из serialMotionEnd() —
// страховка от "LTEST-DRIVE-TRACE продолжает писать после safety/stop".
// Не трогает сегменты, маршрут или follower — только precision-state.
void precisionResetForSafetyEnd();
}

// Forward decl: resetFollowerConfig is defined below after the Follower
// struct. Used in serialMotionEnd to revert L-shape test overrides.
static void resetFollowerConfig();

// Forward decl: ltestDriveTraceTick lives inside namespace roverdbg
// (see roverdbg::ltestDriveTraceTick below). It needs g_precision /
// PrecisionPhase which are namespace-local. Call site uses
// roverdbg::ltestDriveTraceTick().
namespace roverdbg { void ltestDriveTraceTick(); }

static void serialMotionEnd(const char* reason) {
    if (!g_serialMotion.active && g_serialMotion.source == SerialMotionSource::NONE) return;
    // If the L-shape test was the active motion, finalise it first so
    // the partial / final summary lands in the Serial log right after
    // [SERIAL-MOTION] end.
    if (g_ltest.active &&
        g_serialMotion.source == SerialMotionSource::GO_L_SHAPE) {
        roverdbg::lShapeFinish(reason ? reason : "unknown");
    }
    if (g_sqtest.active &&
        g_serialMotion.source == SerialMotionSource::GO_SQUARE) {
        roverdbg::squareFinish(reason ? reason : "unknown");
    }
    Serial.printf("[SERIAL-MOTION] end source=%s reason=%s\n",
                  serialMotionSourceName(g_serialMotion.source),
                  reason ? reason : "-");
    // Restore the default follower config so the next regular route
    // (NAV_START, GO_FORWARD, route from WebSocket, …) sees the
    // project's normal arrival radius and speed, not the L-shape
    // override. The L-shape flow has already made the override
    // visible via [LTEST] CONFIG at start; this guarantees it does
    // not leak into subsequent motions.
    resetFollowerConfig();
    // === Точечная защита от "LTEST-DRIVE-TRACE продолжает писать после end" ===
    // Если по какой-то причине precision-flow не закрылся (safety / STOP /
    // timeout сбросили serialMotion.active, но g_precision остался активным
    // с phase=DRIVE) — trace будет писать вечно. Гасим явно через
    // roverdbg::precisionResetForSafetyEnd().
    roverdbg::precisionResetForSafetyEnd();
    g_serialMotion.active = false;
    g_serialMotion.source = SerialMotionSource::NONE;
    g_serialMotion.startedAtMs = 0;
    g_serialMotion.safetyArmedAtMs = 0;
    g_serialMotion.timeoutMs = 0;
}

// --- waypoint follower v2: simple first-route controller ---
struct Follower {
    int   wpIdx = 0;
    bool  running = false;
    bool  paused  = false;
    float linearMps = 0;
    float angularRadps = 0;
    float headingErr = 0;
    float crossTrack = 0;
    float distToTarget = 0;
    float targetX = 0;
    float targetY = 0;
    float targetHeadingDeg = 0;
    bool  arrived = false;
    uint32_t arrivedSinceMs = 0;
    const char* faultReason = nullptr;
    bool progressInit = false;
    float progressX = 0;
    float progressY = 0;
    uint32_t progressSinceMs = 0;
    bool  turnWatchActive = false;
    float turnWatchHeadingDeg = 0;
    float turnWatchAbsErrorDeg = 0;
    float turnWatchCommandSign = 0;
    uint32_t turnWatchSinceMs = 0;
    // Recovery counter: сколько fault подряд в одном WP без прогресса. После порога —
    // не финишируем маршрут, а переключаемся в degraded-режим и пробуем снова.
    int   faultCount = 0;
    int   lastFaultWp = -1;
    // FIXED loss: с момента потери начинаем отсчёт; после SAFE_NAV_TIMEOUT_MS —
    // настоящий fault. 0 = либо нет потери, либо FIXED уже вернулся.
    uint32_t rtkLostSinceMs = 0;
    // Стартовая поза для wpIdx==0. Фиксируется один раз за маршрут —
    // это конец виртуального сегмента A->WP[0].
    bool  startCaptured = false;
    float startX = 0;
    float startY = 0;
    // turn-in-place флаг (раньше был file-static, мог "застрять" в true).
    bool  turnInPlaceActive = false;

    // === Диагностика silent zero ===
    // Последняя причина, по которой мы НЕ отправили команду мотору,
    // хотя сегмент ещё не закончился. Если пусто — мотор шёл нормально.
    const char* speedBlockedReason = nullptr;
    // linear ДО crosstrack-soft и ДО clamp.
    float linearPreClamp = 0;
    // linear ПОСЛЕ crosstrack-soft (или 0 если fault).
    float linearPostClamp = 0;
    // slowFactor и rampFactor тоже полезны для разбора.
    float slowFactorDbg = 0;
    float rampFactorDbg = 0;
    // heading valid/age на момент tick'а — для диагностики
    // "navigation gate закрыт".
    bool  headingValidDbg = false;
    uint32_t headingAgeDbg = 0xFFFFFFFFu;
    uint32_t pvtAgeDbg = 0;
    bool  serialMotionDbg = false;
    uint32_t lastBlockedLogMs = 0;

    void reset() {
        wpIdx = 0; running = false; paused = false;
        linearMps = angularRadps = headingErr = crossTrack = distToTarget = 0;
        targetX = targetY = targetHeadingDeg = 0;
        arrived = false; arrivedSinceMs = 0;
        faultReason = nullptr;
        progressInit = false;
        progressX = progressY = 0;
        progressSinceMs = 0;
        turnWatchActive = false;
        turnWatchHeadingDeg = turnWatchAbsErrorDeg = turnWatchCommandSign = 0;
        turnWatchSinceMs = 0;
        faultCount = 0;
        lastFaultWp = -1;
        rtkLostSinceMs = 0;
        startCaptured = false;
        startX = startY = 0;
        turnInPlaceActive = false;
        speedBlockedReason = nullptr;
        linearPreClamp = linearPostClamp = 0;
        slowFactorDbg = rampFactorDbg = 0;
        headingValidDbg = false;
        headingAgeDbg = 0xFFFFFFFFu;
        pvtAgeDbg = 0;
        serialMotionDbg = false;
        lastBlockedLogMs = 0;
    }
} g_follow;

// --------------------------------------------------------------------
// Follower runtime configuration
// --------------------------------------------------------------------
//
// stepFollower() reads from these runtime variables rather than the
// compile-time macros in RtkConfig.h. Default values match the macros
// exactly, so normal navigation (GO_FORWARD, GO_NORTH, GO_L_SHAPE, ROUTE
// from WebSocket, NAV_START) is bit-identical unless a command
// explicitly overrides the values. GO_L_SHAPE_DEBUG uses this to run
// the test at a smaller arrival radius and a slower forward speed
// without touching the global project constants.
//
// resetFollowerConfig() restores the defaults and is called by the
// serialMotionEnd hook so debug overrides never leak into the next
// regular route.
static float g_arrivalRadiusM      = ROVER_V2_ARRIVAL_RADIUS_M;
static float g_finalArrivalRadiusM = ROVER_V2_FINAL_ARRIVAL_RADIUS_M;
static float g_forwardSpeedMps     = ROVER_V2_FORWARD_MPS;

static void setFollowerConfig(float arrivalRadiusM,
                              float finalArrivalRadiusM,
                              float forwardSpeedMps) {
    g_arrivalRadiusM      = arrivalRadiusM;
    g_finalArrivalRadiusM = finalArrivalRadiusM;
    g_forwardSpeedMps     = forwardSpeedMps;
}

static void resetFollowerConfig() {
    g_arrivalRadiusM      = ROVER_V2_ARRIVAL_RADIUS_M;
    g_finalArrivalRadiusM = ROVER_V2_FINAL_ARRIVAL_RADIUS_M;
    g_forwardSpeedMps     = ROVER_V2_FORWARD_MPS;
}

// Helper: список "link watchdog" причин, которые должны игнорироваться
// во время serial/debug autonomous motion. Эти команды самостоятельные —
// link между WebSocket / Manual и main loop идёт в idle после их запуска,
// это НЕ признак потери командной связи.
static bool isSerialMotionIgnorableSafetyReason(const char* reason) {
    if (!reason) return false;
    return strcmp(reason, "ws_disconnected") == 0 ||
           strcmp(reason, "nav_cmd_timeout") == 0 ||
           strcmp(reason, "manual_cmd_timeout") == 0;
}

// Called once per loop() from rover.cpp. Clears the serialDebugMotion
// override when the test drive completes (arrival / safety fault /
// alignment finish / hard timeout).
static void serialMotionTick() {
    if (!g_serialMotion.active) return;
    const uint32_t now = millis();
    if (g_safety.level() == SAFETY_ESTOP || g_safety.level() == SAFETY_HOLD) {
        const char* safetyReason = g_safety.reason();
        if (now < g_serialMotion.safetyArmedAtMs &&
            safetyReason != nullptr &&
            strcmp(safetyReason, "ws_disconnected") == 0) {
            // Старый grace-only-bypass для ws_disconnected — оставлен,
            // но Safety.cpp уже сейчас возвращает SAFETY_OK для этих причин
            // во время serial motion. Этот блок остаётся как защита на
            // первые SERIAL_MOTION_SAFETY_GRACE_MS после begin().
            return;
        }
        if (safetyReason != nullptr &&
            isSerialMotionIgnorableSafetyReason(safetyReason)) {
            // Link watchdog не должен убивать self-contained serial motion
            // (AUTO_ALIGN / GO_FORWARD / GO_L_SHAPE_DEBUG / GO_SQUARE_DEBUG).
            Serial.printf("[SERIAL-MOTION] ignoring safety reason=%s during serial motion\n",
                          safetyReason);
            return;
        }
        Serial.printf("[SERIAL-MOTION] safety abort level=%d reason=%s ageSinceBegin=%u\n",
                      (int)g_safety.level(),
                      safetyReason ? safetyReason : "-",
                      (unsigned)(now - g_serialMotion.startedAtMs));
        serialMotionEnd("safety");
        return;
    }
    if (g_follow.arrived && !roverdbg::precisionDebugActive()) {
        serialMotionEnd("arrived");
        return;
    }
    if (g_serialMotion.source == SerialMotionSource::AUTO_ALIGN &&
        g_heading.alignState != AlignState::RUNNING) {
        // Alignment finished (OK or ERR). The alignment state machine
        // already stops the motors; we just need to drop the
        // serialDebugMotion override so ws_disconnected re-engages.
        serialMotionEnd("align_finished");
        return;
    }
    if (g_serialMotion.timeoutMs != 0 &&
        (now - g_serialMotion.startedAtMs) > g_serialMotion.timeoutMs) {
        serialMotionEnd("timeout");
        // also stop the motors to be safe
        g_motor.stopImmediately();
        return;
    }
}

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float wrapDeg180Local(float d) {
    return NavMath::wrapDeg180(d);
}

// Раньше прокачивала g_motor.loop() вручную во время ожидания. Теперь поток команд
// держит фоновая TX-задача на ядре 0 (g_motor.startTxTask), поэтому здесь просто ждём —
// дёргать loop() вручную НЕЛЬЗЯ: будет гонка двух нитей за один Serial2.
static void serviceMotorFor(uint32_t durationMs) {
    delay(durationMs);
}

static void followerFault(const char* reason) {
    // Мягкий fault: НЕ финишируем маршрут, НЕ глушим мотоpы глобально. Считаем попытки
    // на текущем WP. После ROVER_FAULT_RECOVERY_COUNT фейлов подряд — сдаёмся и
    // переходим в ERROR (приложение увидит через NAV,... телеметрию). Это даёт шанс
    // восстановиться после GPS-бленка / кратковременной потери FIXED.
    g_follow.faultReason = reason;
    g_follow.progressInit = false;
    g_motor.stopImmediately();
    if (g_follow.lastFaultWp != g_follow.wpIdx) {
        g_follow.lastFaultWp = g_follow.wpIdx;
        g_follow.faultCount = 1;
    } else {
        g_follow.faultCount++;
    }
    if (g_follow.faultCount >= ROVER_FAULT_RECOVERY_COUNT) {
        g_follow.running = false;
        g_follow.arrived = false;
        g_route.finish();
        Serial.printf("[ROVER] fault terminal: %s (count=%d)\n", reason, g_follow.faultCount);
    } else {
        Serial.printf("[ROVER] fault soft: %s (count=%d/%d) — retrying\n",
                      reason, g_follow.faultCount, ROVER_FAULT_RECOVERY_COUNT);
    }
}

static bool checkFollowerProgress(const Estimate& e, float commandedSpeed) {
    uint32_t now = millis();
    if (!g_safety.allowMotion() || fabsf(commandedSpeed) < ROVER_STUCK_MIN_CMD_MPS) {
        g_follow.progressInit = false;
        return true;
    }
    if (!g_follow.progressInit) {
        g_follow.progressInit = true;
        g_follow.progressX = e.x;
        g_follow.progressY = e.y;
        g_follow.progressSinceMs = now;
        return true;
    }
    float dx = e.x - g_follow.progressX;
    float dy = e.y - g_follow.progressY;
    float moved = sqrtf(dx * dx + dy * dy);
    if (moved >= ROVER_STUCK_MIN_MOVE_M || e.speedMps >= ROVER_STUCK_MIN_CMD_MPS) {
        g_follow.progressX = e.x;
        g_follow.progressY = e.y;
        g_follow.progressSinceMs = now;
        return true;
    }
    if ((now - g_follow.progressSinceMs) > ROVER_STUCK_TIMEOUT_MS) {
        followerFault("stuck_no_progress");
        return false;
    }
    return true;
}

static bool waitForInitialAbsoluteHeading(uint32_t timeoutMs, float& outHeadingDeg) {
    uint32_t start = millis();
    uint32_t stableSince = 0;
    float firstHeading = 0.0f;
    bool haveFirst = false;
    while ((uint32_t)(millis() - start) < timeoutMs) {
        g_imu.loop();
        uint32_t now = millis();
        if (!g_imu.fresh() || g_imu.yawAgeMs(now) >= SAFE_IMU_AGE_MS) {
            delay(10);
            continue;
        }
        if (g_imu.headingState() != ImuHeadingState::IMU_ABSOLUTE_OK ||
            !g_imu.yawAbsoluteValid() ||
            g_imu.yawAccRad() > IMU_ABS_YAW_MAX_ACC_RAD) {
            delay(10);
            continue;
        }
        float cur = g_imu.yawDeg();
        if (!haveFirst) {
            firstHeading = cur;
            stableSince = now;
            haveFirst = true;
        }
        if (fabsf(wrapDeg180Local(cur - firstHeading)) > IMU_STARTUP_MAX_JUMP_DEG) {
            firstHeading = cur;
            stableSince = now;
            haveFirst = false;
            delay(10);
            continue;
        }
        if (stableSince != 0 && (now - stableSince) >= IMU_STARTUP_STATIONARY_MS) {
            outHeadingDeg = cur;
            return true;
        }
        delay(10);
    }
    return false;
}

static bool ensureTestOrigin() {
    if (g_est.get().originSet) return true;
    const auto& e = g_est.get();
    // FIXED only в автономных тестах — то же правило, что и stepFollower.
    const bool rtkOk = (e.sol == SOL_FIXED && e.hAcc <= SAFE_HACC_FIXED_M);
    if (!rtkOk || e.lat == 0.0 || e.lon == 0.0) return false;
    if (!g_est.setOrigin(e.lat, e.lon)) return false;
    const auto& after = g_est.get();
    Serial.printf("[TEST-ORIGIN] mapOriginLat=%.8f mapOriginLon=%.8f "
                  "currentRobotX=%.3f currentRobotY=%.3f\n",
                  after.originLat, after.originLon, after.x, after.y);
    return true;
}

static bool startGoRoute(const char* tag, float distanceM, bool forwardFromHeading) {
    if (!isfinite(distanceM) || distanceM <= 0.0f || distanceM > 2.0f) {
        Serial.printf("[%s] refusing: distance %.2f outside (0,2.0]m\n",
                      tag, (double)distanceM);
        return false;
    }
    if (!ensureTestOrigin()) {
        Serial.printf("[%s] refusing: no map origin and no usable RTK fix\n", tag);
        return false;
    }

    const auto& e = g_est.get();
    const float headingDeg =
        e.headingValid ? e.headingFiltDeg : g_imu.yawDeg();
    float dx = 0.0f;
    float dy = distanceM;
    if (forwardFromHeading) {
        NavMath::forwardOffset(distanceM, headingDeg, dx, dy);
    }
    const float startX = e.x;
    const float startY = e.y;
    const float targetX = startX + dx;
    const float targetY = startY + dy;
    const float margin = 0.75f;
    const float minX = fminf(startX, targetX) - margin;
    const float maxX = fmaxf(startX, targetX) + margin;
    const float minY = fminf(startY, targetY) - margin;
    const float maxY = fmaxf(startY, targetY) + margin;

    g_follow.reset();
    if (!g_route.beginUpload(2, e.originLat, e.originLon)) {
        Serial.printf("[%s] refusing: route begin failed\n", tag);
        return false;
    }
    g_route.addWaypoint(0, startX, startY);
    g_route.addWaypoint(1, targetX, targetY);
    g_route.beginBoundary(4);
    g_route.addBoundaryPoint(0, minX, minY);
    g_route.addBoundaryPoint(1, maxX, minY);
    g_route.addBoundaryPoint(2, maxX, maxY);
    g_route.addBoundaryPoint(3, minX, maxY);
    g_route.endBoundary();
    g_route.beginForbidden(0, nullptr);
    g_route.endForbidden();
    g_route.endUpload();
    g_route.start();
    g_ws.requestDebugNavigation();
    // Mark this as a serial-initiated motion so ws_disconnected does
    // not stop the test drive. Cleared by handleStopLine or by
    // serialMotionTick on arrival/safety/timeout.
    serialMotionBegin(strcmp(tag, "GO_NORTH") == 0
                          ? SerialMotionSource::GO_NORTH
                          : SerialMotionSource::GO_FORWARD,
                      SERIAL_GO_TIMEOUT_MS);
    Serial.printf("[%s] distance=%.2f heading=%.1f start=(%.2f,%.2f) "
                  "target=(%.2f,%.2f) origin=(%.8f,%.8f) speed=%.2f\n",
                  tag, (double)distanceM, (double)headingDeg,
                  (double)startX, (double)startY,
                  (double)targetX, (double)targetY,
                  e.originLat, e.originLon,
                  (double)g_forwardSpeedMps);
    return true;
}

static void connectWiFi() {
    WiFi.disconnect(true, true);
    delay(300);
    WiFi.mode(WIFI_OFF);
    delay(300);
    WiFi.mode(WIFI_STA);
    WiFi.persistent(false);
    WiFi.setSleep(false);
    IPAddress ip, gw, sn, dns;
    ip.fromString(ROVER_IP);
    gw.fromString("192.168.31.1");
    sn.fromString("255.255.255.0");
    dns = gw;
    WiFi.config(ip, gw, sn, dns);
    WiFi.setAutoReconnect(true);
    WiFi.setTxPower(WIFI_POWER_19_5dBm);

    for (int attempt = 1; attempt <= 3 && WiFi.status() != WL_CONNECTED; ++attempt) {
        Serial.printf("[ROVER] wifi attempt %d ssid=%s\n", attempt, WIFI_SSID);
        WiFi.begin(WIFI_SSID, WIFI_PASS);
        int t = 0;
        while (WiFi.status() != WL_CONNECTED && t < 20) {
            serviceMotorFor(500);
            Serial.print(".");
            t++;
        }
        if (WiFi.status() != WL_CONNECTED) {
            Serial.printf("\n[ROVER] wifi attempt %d failed status=%d\n",
                          attempt, (int)WiFi.status());
            WiFi.disconnect(false, false);
            serviceMotorFor(1000);
        }
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[ROVER] wifi connected ip=%s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\n[ROVER] wifi TIMEOUT");
    }
}

static void stepFollower() {
    // === LTEST DRIVE PATH TRACE ===
    // По нему видно, реально ли stepFollower() вызывается для LTEST
    // и какой linear сейчас считает. Если ты видишь эти строки, но
    // cmdL=cmdR=0 — значит linear=0 внутри расчёта (slow/ramp/turn).
    // Если строк НЕТ — stepFollower() вообще не вызывается.
    roverdbg::ltestDriveTraceTick();
    if (!g_route.isRunning()) {
        g_follow.linearMps = g_follow.angularRadps = 0;
        g_motor.setLinearAngularSpeed(0, 0, true);
        return;
    }
    if (g_route.isPaused()) {
        g_follow.linearMps = g_follow.angularRadps = 0;
        g_motor.setLinearAngularSpeed(0, 0, true);
        return;
    }

    const auto& e = g_est.get();
    int total = g_route.count();
    NavPoint current{e.x, e.y};

    // RTK gate (FIXED only в автономном режиме).
    const bool rtkOk = (e.sol == SOL_FIXED && e.hAcc <= SAFE_HACC_FIXED_M);
    if (!rtkOk || e.pvtAgeMs > SAFE_PVT_AGE_MS ||
        e.acceptedPositionAgeMs > SAFE_ACCEPTED_POS_AGE_MS) {
        // Мягкая реакция: плавный стоп + ожидание reacquire в течение
        // SAFE_NAV_TIMEOUT_MS. Только если FIXED не вернулся за это окно
        // — реальный fault. followerFault() сам инкрементирует счётчик
        // и после ROVER_FAULT_RECOVERY_COUNT фаулит маршрут.
        g_follow.linearMps = 0;
        g_follow.angularRadps = 0;
        g_motor.stopImmediately();
        if (g_follow.rtkLostSinceMs == 0) g_follow.rtkLostSinceMs = millis();
        if ((millis() - g_follow.rtkLostSinceMs) > SAFE_NAV_TIMEOUT_MS) {
            followerFault(rtkOk ? "position_stale" : "rtk_lost");
            g_follow.rtkLostSinceMs = 0;
        }
        return;
    }
    g_follow.rtkLostSinceMs = 0;
    g_follow.faultReason = nullptr;

    if (!g_route.positionAllowed(current, ROVER_BOUNDARY_TOLERANCE_M)) {
        followerFault("perimeter_violation");
        return;
    }

    if (total == 0 || g_follow.wpIdx >= total) {
        g_follow.running = false;
        g_follow.arrived = true;
        g_follow.linearMps = 0;
        g_follow.angularRadps = 0;
        g_route.finish();
        g_motor.setLinearAngularSpeed(0, 0, true);
        return;
    }

    // === Сегмент A -> B (line follower) ===
    //   A = предыдущий WP, либо стартовая поза для wpIdx==0;
    //   B = текущий WP (g_follow.wpIdx).
    // Сейчас робот едет по waypoint'у (bearing на B), а не по линии AB.
    // Это и есть "срезание углов + дуга после поворота + перескок курса".
    // Переходим к Stanley-стилю line following с lookahead на сегменте.
    NavPoint a;
    if (g_follow.wpIdx == 0) {
        // Стартовая поза — фиксируется при первом тике follower'а и
        // остаётся неизменной до конца маршрута (или до nextUpload).
        if (!g_follow.startCaptured) {
            g_follow.startX = current.x;
            g_follow.startY = current.y;
            g_follow.startCaptured = true;
        }
        a.x = g_follow.startX;
        a.y = g_follow.startY;
    } else {
        const Waypoint& prevWp = g_route.waypoint(g_follow.wpIdx - 1);
        a.x = prevWp.p.x;
        a.y = prevWp.p.y;
    }
    const Waypoint& wp = g_route.waypoint(g_follow.wpIdx);
    const NavPoint b = wp.p;

    const float segVx = b.x - a.x;
    const float segVy = b.y - a.y;
    const float segLen = sqrtf(segVx * segVx + segVy * segVy);

    // along-track (проекция на ось A->B), cross-track (signed).
    // Используем уже существующие lshape-along/Cross-функции — те же
    // математика, просто переиспользуем, чтобы не плодить дублей.
    const float along = lshapeAlongTrack(a.x, a.y, b.x, b.y, current.x, current.y);
    const float cross = lshapeSignedCross(a.x, a.y, b.x, b.y, current.x, current.y);
    g_follow.crossTrack = cross;

    // --- Переключение waypoint ---
    //   * промежуточные WP: along-track >= segLen (минус safety margin);
    //   * финальный WP (wpIdx == total-1): финиш по радиусу finalArrival
    //     (0.10 м), чтобы корректно закрывать route.
    // Решение "по distToWp < arrival" раньше срезало углы.
    const bool isFinalWp = (g_follow.wpIdx == total - 1);
    const bool reachedByAlong = (segLen > 0.001f) &&
                                (along >= segLen - 0.05f);
    const float distToB = sqrtf((b.x - current.x) * (b.x - current.x) +
                                (b.y - current.y) * (b.y - current.y));
    const bool reachedByRadius = distToB <
        (isFinalWp ? g_finalArrivalRadiusM : g_arrivalRadiusM);
    if (reachedByAlong || reachedByRadius) {
        g_follow.faultCount = 0;
        g_follow.lastFaultWp = -1;
        g_follow.wpIdx++;
        if (g_follow.wpIdx >= total) {
            g_follow.running = false;
            g_follow.arrived = true;
            g_follow.linearMps = 0;
            g_follow.angularRadps = 0;
            g_route.finish();
            g_motor.setLinearAngularSpeed(0, 0, true);
            return;
        }
        // Не делаем резкий turn-in-place сразу после переключения, если
        // угол между сегментами небольшой. Решение принимается ниже — если
        // wpIdx уже инкрементирован, на следующем тике возьмём новый сегмент.
        return;
    }

    // === Stanley line-follower на сегменте A->B ===
    // headingErr — относительно направления сегмента (НЕ bearing на B).
    const float desiredHeading =
        (segLen > 0.001f) ? NavMath::targetHeadingDeg(segVx, segVy)
                          : e.headingFiltDeg;
    const float headingErr = wrapDeg180Local(desiredHeading - e.headingFiltDeg);
    const float absErr = fabsf(headingErr);
    g_follow.targetHeadingDeg = desiredHeading;
    g_follow.headingErr = headingErr;

    // Целевая точка на сегменте: проекция текущей позиции + lookahead.
    // НО клампим до B, чтобы не уезжать за текущий WP.
    const float lookahead = (segLen > 0.001f) ? ROVER_LOOKAHEAD_M : 0.0f;
    float tgtProj = along + lookahead;
    if (tgtProj > segLen) tgtProj = segLen;
    float targetX, targetY;
    if (segLen > 0.001f) {
        const float u = tgtProj / segLen;
        targetX = a.x + segVx * u;
        targetY = a.y + segVy * u;
    } else {
        targetX = current.x;
        targetY = current.y;
    }
    g_follow.targetX = targetX;
    g_follow.targetY = targetY;
    g_follow.distToTarget = distToB;

    NavPoint targetN{targetX, targetY};
    if (!g_route.segmentAllowed(current, targetN,
                                ROVER_BOUNDARY_TOLERANCE_M,
                                ROVER_BOUNDARY_SAMPLE_M)) {
        followerFault("route_segment_blocked");
        return;
    }

    float linear = 0.0f;
    float angular = 0.0f;

    // === Turn-in-place: гистерезис входа/выхода + P-профиль + settle по yawRate ===
    // Раньше: bang-bang фиксированная omega=0.95 → перелёт 30°+ и осцилляции.
    // Теперь: P-профиль clamp(KP*err, minW, maxW) со знаком, выход только когда
    // ошибка < EXIT_DEG И |yawRate| < SETTLE_RATE_DPS.
    // Флаг состояния теперь живёт в g_follow (а не file-static), и сбрасывается
    // в g_follow.reset() — иначе между сессиями он мог застрять в true.
    const float absYawRate = fabsf(g_imu.yawRateDps());

    auto exitTurnInPlace = [&]() {
        g_follow.turnInPlaceActive = false;
        g_follow.turnWatchActive = false;
    };

    if (!g_follow.turnInPlaceActive && absErr > ROVER_V2_TURN_IN_PLACE_DEG) {
        // Вход в turn-in-place.
        g_follow.turnInPlaceActive = true;
        g_follow.turnWatchActive = true;
        g_follow.turnWatchHeadingDeg = e.headingFiltDeg;
        g_follow.turnWatchAbsErrorDeg = absErr;
        g_follow.turnWatchCommandSign = (headingErr >= 0.0f) ? 1.0f : -1.0f;
        g_follow.turnWatchSinceMs = millis();
    }

    if (g_follow.turnInPlaceActive) {
        // Проверка выхода: ошибка упала + yaw rate осел.
        const bool headingOk = absErr < ROVER_TURN_EXIT_DEG;
        const bool rateOk    = (absYawRate < ROVER_TURN_SETTLE_RATE_DPS);
        if (headingOk && rateOk) {
            exitTurnInPlace();
        } else {
            // P-профиль: omega = clamp(KP*|err|, MIN, MAX) со знаком err.
            const float mag = fabsf(ROVER_TURN_KP * headingErr);
            const float clamped = clampf(mag, ROVER_TURN_MIN_RADPS, ROVER_V2_TURN_RADPS);
            angular = (headingErr >= 0.0f ? clamped : -clamped);
            linear = 0.0f;

            // Watchdog от старого кода оставлен: если за ROVER_V2_TURN_WATCHDOG_MS
            // направление/ошибка не улучшились — fault.
            const uint32_t now = millis();
            if ((now - g_follow.turnWatchSinceMs) > ROVER_V2_TURN_WATCHDOG_MS) {
                const float signedDelta =
                    wrapDeg180Local(e.headingFiltDeg - g_follow.turnWatchHeadingDeg);
                const bool directionOk =
                    signedDelta * g_follow.turnWatchCommandSign >= ROVER_V2_TURN_MIN_DELTA_DEG;
                const bool errorImproved =
                    absErr <= g_follow.turnWatchAbsErrorDeg - ROVER_V2_TURN_MIN_ERROR_IMPROVE_DEG;
                if (!directionOk || !errorImproved) {
                    exitTurnInPlace();
                    followerFault(!directionOk ? "turn_wrong_direction_or_stuck"
                                               : "turn_not_converging");
                    return;
                }
                g_follow.turnWatchHeadingDeg = e.headingFiltDeg;
                g_follow.turnWatchAbsErrorDeg = absErr;
                g_follow.turnWatchSinceMs = now;
            }
        }
    }

    if (!g_follow.turnInPlaceActive) {
        // === Drive-on-line (Stanley) ===
        // Если headingErr большой (но < V2_TURN_IN_PLACE_DEG), притормаживаем.
        const float headingRad = headingErr * (float)M_PI / 180.0f;
        const float speedBase = (g_safety.level() == SAFETY_DEGRADED)
                                ? ROVER_FLOAT_SPEED : g_forwardSpeedMps;
        // linear снижается по закону 1 - errNorm^2, где errNorm = absErr/45°.
        const float errNorm = absErr / 45.0f;
        const float slowFactor = (errNorm >= 1.0f) ? 0.0f
                                : (1.0f - errNorm * errNorm);
        // Ramp-up на первых ~15 см сегмента: после corner handoff heading
        // ещё качается, и если сразу дать full speed на первых сантиметрах,
        // Stanley врезает жёсткий steer и cross-track сразу уходит в 0.2м.
        // smoothstep 0->1 по along/RAMP_UP_M, не трогает ни turn, ни angular.
        //
        // FLOOR: DEBUG_RAMP_MIN_FACTOR — иначе на самом старте сегмента
        // along=0 → smoothstep(0)=0 → linear=0 → ровер не едет → along
        // не растёт → rampFactor=0 навсегда (deadlock). Минимум 0.45
        // даёт ~0.05 м/с на старте (g_forwardSpeedMps=0.11 для LTEST).
        float rampFactor = 1.0f;
        if (along < DEBUG_RAMP_UP_M) {
            const float u = along / DEBUG_RAMP_UP_M;     // 0..1
            rampFactor = u * u * (3.0f - 2.0f * u);       // smoothstep
            if (rampFactor < DEBUG_RAMP_MIN_FACTOR) {
                rampFactor = DEBUG_RAMP_MIN_FACTOR;
            }
        }
        // Сохраняем в debug-поля ДО обёртки в crosstrack-guard — чтобы
        // по логу можно было отличить "slowFactor=0" от "rampFactor=0"
        // от "мотор не поехал по gate" (три разные причины нуля).
        g_follow.slowFactorDbg = slowFactor;
        g_follow.rampFactorDbg = rampFactor;
        g_follow.linearPreClamp = speedBase * slowFactor * rampFactor;
        linear = g_follow.linearPreClamp;

        // Stanley: heading gain + atan(k_ct * cross / (k_soft + |v|))
        // + демпфирование по yawRate (rad/s).
        const float yawRateRadps =
            g_imu.yawRateDps() * (float)M_PI / 180.0f *
            (IMU_YAW_RATE_SIGN == 0 ? 1 : (IMU_YAW_RATE_SIGN > 0 ? 1.0f : -1.0f));
        const float v = linear;
        const float crossTerm = atanf(ROVER_K_CROSSTRACK * cross /
                                       (ROVER_STANLEY_SOFT_SPEED + fabsf(v)));
        angular = ROVER_K_HEADING * headingRad + crossTerm
                  - ROVER_K_D_YAWRATE * yawRateRadps;
        const float angClamp = ROVER_V2_MAX_CORRECTION_RADPS;
        if (angular >  angClamp) angular =  angClamp;
        if (angular < -angClamp) angular = -angClamp;
    }

    // === Crosstrack guard ===
    //   > SOFT_M  — снижаем скорость пропорционально превышению;
    //   > HARD_M  — fault (crosstrack_exceeded), без терминального recovery-стэка.
    const float absCross = fabsf(g_follow.crossTrack);
    if (absCross > ROVER_CROSSTRACK_HARD_M) {
        followerFault("crosstrack_exceeded");
        return;
    }
    if (absCross > ROVER_CROSSTRACK_SOFT_M && linear > 0.0f) {
        const float over = absCross - ROVER_CROSSTRACK_SOFT_M;
        const float span = ROVER_CROSSTRACK_HARD_M - ROVER_CROSSTRACK_SOFT_M;
        const float factor = 1.0f - clampf(over / span, 0.0f, 1.0f);
        linear *= factor;
    }

    g_follow.linearMps = linear;
    g_follow.angularRadps = angular;

    if (!checkFollowerProgress(e, linear)) return;

    g_follow.linearPostClamp = linear;
    g_follow.headingValidDbg = e.headingValid;
    g_follow.headingAgeDbg   = e.headingAgeMs;
    g_follow.pvtAgeDbg       = e.pvtAgeMs;
    g_follow.serialMotionDbg = g_serialMotion.active;

    const bool navOk   = g_ws.navRequested();
    const bool routeOk = g_route.isRunning();
    const bool safeOk  = g_safety.allowMotion();
    const bool canDrive = navOk && routeOk && safeOk;

    if (canDrive) {
        // SAFETY NET: даже если rampFactor/slowFactor дали 0 на самом
        // старте сегмента, и пока робот реально едет к target (а не
        // завершил), даём ненулевую скорость. Иначе ramp начинается с
        // 0 → linear=0 → робот не едет → along не растёт → rampFactor
        // остаётся 0 (deadlock). Это последний рубеж: основной floor
        // rampFactor стоит выше через DEBUG_RAMP_MIN_FACTOR; этот
        // слой — для случаев, когда slowFactor*=0 при absErr>=45°, но
        // робот уже середине сегмента и должен ехать дальше.
        if (linear <= 0.0f && !g_follow.turnInPlaceActive &&
            g_route.isRunning() && g_follow.wpIdx < g_route.count()) {
            const float absHErr = fabsf(headingErr);
            // Только при "маленькой" heading-ошибке — иначе это уже
            // turn-in-place, который смотрит отдельно.
            if (absHErr < ROVER_V2_TURN_IN_PLACE_DEG &&
                g_follow.distToTarget > (g_arrivalRadiusM + 0.02f)) {
                // 0.06 м/с — гарантированно выше физического порога
                // гусениц, чтобы ramp разморозился. После первого
                // тика along > 0, smoothstep наберёт нормальное
                // значение и этот слой не понадобится.
                linear = 0.06f;
                const float headingRadLocal = headingErr * (float)M_PI / 180.0f;
                angular = clampf(ROVER_K_HEADING * headingRadLocal,
                                 -ROVER_V2_MAX_CORRECTION_RADPS,
                                 ROVER_V2_MAX_CORRECTION_RADPS);
            }
        }
        g_follow.speedBlockedReason = nullptr;
        g_follow.linearMps = linear;
        g_follow.angularRadps = angular;
        g_motor.setLinearAngularSpeed(linear, angular, true);
    } else {
        // Раньше это был silent zero — сейчас фиксируем конкретную причину
        // и логируем не чаще раза в 500мс, чтобы не флудить Serial.
        const char* why;
        if      (!navOk)   why = "nav_not_requested";
        else if (!routeOk) why = "route_not_running";
        else if (!safeOk)  why = "safety_disallows_motion";
        else               why = "unknown";
        g_follow.speedBlockedReason = why;
        g_follow.linearMps = 0;
        g_follow.angularRadps = 0;
        g_motor.stopImmediately();

        const uint32_t now = millis();
        if (now - g_follow.lastBlockedLogMs > 500u) {
            g_follow.lastBlockedLogMs = now;
            Serial.printf("[FOLLOWER-BLOCKED] reason=%s navReq=%d routeRun=%d safeOK=%d "
                          "safetyLevel=%d safetyReason=%s "
                          "linearPreClamp=%.3f linearPostClamp=%.3f "
                          "slowFactor=%.2f rampFactor=%.2f "
                          "headingErr=%.1f absErr=%.1f cross=%.3f "
                          "headingValid=%d headingAge=%u pvtAge=%u "
                          "serialMotion=%d\n",
                          why, (int)navOk, (int)routeOk, (int)safeOk,
                          (int)g_safety.level(),
                          g_safety.reason() ? g_safety.reason() : "-",
                          (double)g_follow.linearPreClamp,
                          (double)g_follow.linearPostClamp,
                          (double)g_follow.slowFactorDbg,
                          (double)g_follow.rampFactorDbg,
                          (double)headingErr, (double)absErr,
                          (double)g_follow.crossTrack,
                          (int)g_follow.headingValidDbg,
                          (unsigned)g_follow.headingAgeDbg,
                          (unsigned)g_follow.pvtAgeDbg,
                          (int)g_follow.serialMotionDbg);
        }
    }
}

// ============================================================================
// RTK motion alignment
// ============================================================================
//
// Purpose: replace mandatory IMU absolute heading with a forward drive +
// RTK displacement method. The rover drives straight (angular=0) at a low
// speed for up to ALIGN_MAX_DIST_M or ALIGN_TIMEOUT_MS, whichever first,
// while recording (x,y) samples. The final heading is computed from
// windowed-mean start and end points (first ALIGN_START_WINDOW and last
// ALIGN_END_WINDOW valid samples), not from a single start/end snapshot, so
// RTK jitter (centimetre-scale fluctuations between PVTs) does not flip the
// resulting heading by tens of degrees.
//
// This runs as a small state machine in stepAlign(), driven by main loop().
// The Serial/WS command just transitions IDLE → RUNNING and returns
// immediately. loop() advances the state until OK/ERR, at which point
// followup code can read g_heading.

static constexpr float ALIGN_FORWARD_MPS       = 0.18f;
// We always stop at 1.5 m (or earlier) so an over-shoot doesn't push the
// robot outside the test field, but OK is also allowed to fire earlier if
// we have enough high-confidence travel.
static constexpr float ALIGN_MAX_DIST_M        = 2.5f;
static constexpr float ALIGN_MIN_TRAVEL_M      = 1.0f;   // accept-OK travel
static constexpr uint32_t ALIGN_TIMEOUT_MS     = 15000u;
// Below this we still accept but tag as not_enough_motion in diagnostics.
static constexpr float ALIGN_MIN_DIST_HARD     = 0.75f;
// Soft tolerance: 0.10 m is the project's nominal FIXED gate; for alignment
// we are OK with up to 0.10 m before we even start.
static constexpr float ALIGN_START_MAX_HACC    = 0.10f;
// Looser PVT staleness threshold for alignment. The rover may sample
// PVT at 1 Hz with ~1.1 s inter-arrival jitter; the navigation gate
// (SAFE_PVT_AGE_MS = 1000) is too tight for alignment and was killing
// otherwise healthy runs.
static constexpr uint32_t ALIGN_MAX_PVT_AGE_MS = 2500u;
// Per-sample variance inside the stationary start/end window. 0.07 m
// RMS is the upper bound; a real FIXED RTK at 1.4 cm horizontal accuracy
// typically shows 0.01..0.03 m RMS over a 0.5 s window. Bigger than
// 0.07 m indicates multipath / mode flip / unhealthy solution.
static constexpr float ALIGN_JITTER_MAX_M      = 0.07f;
// Sample-window size for the mean. 5 at ~10 Hz PVT ≈ 0.5 s of data.
static constexpr int   ALIGN_START_WINDOW       = 5;
static constexpr int   ALIGN_END_WINDOW         = 5;
// Cross-run segment-heading stability. We compute atan2 between windows
// spaced ≥ ALIGN_SEG_MIN_DIST_M apart and look at the population stddev.
// 20 deg is loose on purpose; on a 1.5 m run with ~0.5 m spaced segments
// this only flags truly wild jitter.
static constexpr float ALIGN_HEADING_STD_MAX_DEG = 20.0f;
static constexpr float ALIGN_SEG_MIN_DIST_M     = 0.30f;

struct AlignSample {
    float x;
    float y;
    float hAcc;
    uint32_t tMs;
};

// Phase machine for the alignment run. The previous version mixed
// stationary and moving samples into the same "start/end" window which
// spuriously counted real robot motion as RTK jitter and aborted
// otherwise healthy runs. The new version keeps three separate windows:
//
//   1. startSamples — filled ONLY during START_SETTLE, while the rover
//      is stationary. Their RMS radius is the genuine RTK jitter.
//   2. movingSamples — ring of the most recent N samples taken during
//      DRIVE. Used purely for progress / segment-stability debug; never
//      counted toward jitter.
//   3. endSamples — filled ONLY during END_SETTLE, after the rover has
//      stopped moving. Their RMS radius is the genuine end-of-run jitter.
//
// Final heading = atan2(mean(end) - mean(start)) over stationary-only
// windows, immune to motion displacement and to long alignments.
enum class AlignPhase : uint8_t {
    START_SETTLE = 0,
    DRIVE,
    END_SETTLE,
};

static constexpr int   ALIGN_WINDOW_CAP        = 10;
static constexpr int   ALIGN_MOVING_CAP        = 32;

struct AlignRun {
    bool active = false;
    AlignPhase phase = AlignPhase::START_SETTLE;
    uint32_t phaseStartedAtMs = 0;
    uint32_t startedAtMs = 0;

    // Phase 1: stationary start window.
    AlignSample startSamples[ALIGN_WINDOW_CAP];
    int   startSampleCount = 0;
    bool  startLogged = false;

    // Phase 2: moving debug ring (most recent N during DRIVE).
    AlignSample movingSamples[ALIGN_MOVING_CAP];
    int   movingSampleCount = 0;
    int   movingSampleHead  = 0;
    int   driveSampleCount = 0;  // total PVT samples seen during DRIVE

    // Phase 3: stationary end window.
    AlignSample endSamples[ALIGN_WINDOW_CAP];
    int   endSampleCount = 0;
    int   endSampleHead  = 0;
    bool  endLogged = false;

    // De-dup of PVT updates. We track pvtAgeMs every loop tick and push
    // a new sample only when pvtAgeMs decreased since the previous tick,
    // i.e. when the parser delivered a fresh PVT. We do NOT tie this to
    // the last pushed sample: an earlier implementation updated
    // lastSamplePvtAgeMs only on push, so a stuck age=0 (no new PVT
    // arriving) kept the gate open and then locked it shut.
    uint32_t lastSeenPvtAgeMs = 0xFFFFFFFFu;
    // Monotonic counter of samples pushed this run. Used for the
    // per-sample debug log.
    uint32_t samplePushCount = 0;
    // Alive-watchdog: момент последнего успешного push'а PVT. Если в
    // DRIVE-фазе новых PVT не было дольше ALIGN_DRIVE_ALIVE_MS — ровер
    // может ехать по инерции прошлой команды, поэтому abort.
    uint32_t lastSamplePushedMs = 0;

    bool abortedByStop = false;
    bool abortedByRtk = false;
    bool fromWebSocket = false;
    bool originAutoSet = false;
} g_align;

static bool rtkAcceptableForAlign(const Estimate& e) {
    const bool rtkOk =
        (e.sol == SOL_FIXED && e.hAcc <= ALIGN_START_MAX_HACC) ||
        (e.sol == SOL_FLOAT && e.hAcc <= 0.05f);
    return rtkOk && e.lat != 0.0f && e.lon != 0.0f && e.pvtAgeMs <= ALIGN_MAX_PVT_AGE_MS;
}

// Begin a new alignment run. Returns true if we transitioned to RUNNING and
// the caller can expect g_heading to be updated once stepAlign() finishes.
static bool autoAlignHeadingBegin(bool fromWebSocket) {
    // Busy-guard: AUTO_ALIGN конкурирует за моторы с другими debug
    // motion. Нельзя стартовать новый поверх активного.
    if (g_serialMotion.active) {
        g_heading.lastAlignError = "busy_serial_motion";
        g_heading.alignState = AlignState::ERR;
        Serial.printf("[BUSY] AUTO_ALIGN rejected: serialMotionSource=%s active\n",
                      serialMotionSourceName(g_serialMotion.source));
        return false;
    }
    const auto& e0 = g_est.get();
    if (!rtkAcceptableForAlign(e0)) {
        g_heading.lastAlignError = "rtk_lost";
        g_heading.alignState = AlignState::ERR;
        return false;
    }

    // If no map origin is installed (operator is running a pure Serial
    // debug session, no app, no route uploaded), create a temporary
    // local origin from the live RTK so alignment can run. This does NOT
    // touch any saved map origin. Once a real ROUTE_BEGIN arrives the
    // StateEstimator will be re-anchored to the map's lat/lon and the
    // temporary anchor becomes irrelevant. Avoid logging it as a "map
    // origin" to keep diagnostics honest.
    g_align.originAutoSet = false;
    if (!e0.originSet) {
        if (!g_est.setOrigin(e0.lat, e0.lon)) {
            g_heading.lastAlignError = "no_origin";
            g_heading.alignState = AlignState::ERR;
            return false;
        }
        g_align.originAutoSet = true;
        Serial.println("[ALIGN-RTK] origin not set; using current RTK position as temporary align origin");
    }

    const auto& e = g_est.get();
    g_align.active = true;
    g_align.abortedByStop = false;
    g_align.abortedByRtk = false;
    g_align.fromWebSocket = fromWebSocket;
    g_align.startedAtMs = millis();
    if (!fromWebSocket) {
        // Allow motors to spin without a connected WebSocket client.
        // The flag is cleared inside serialMotionTick (alignment finish
        // -> abort/end handles it via `g_align.active = false`, and the
        // tick uses g_align.active as one of the auto-clear signals).
        serialMotionBegin(SerialMotionSource::AUTO_ALIGN, ALIGN_TIMEOUT_MS + 1000u);
    }
    g_align.phase = AlignPhase::START_SETTLE;
    g_align.phaseStartedAtMs = millis();
    g_align.startSampleCount = 0;
    g_align.startLogged = false;
    g_align.movingSampleCount = 0;
    g_align.movingSampleHead = 0;
    g_align.driveSampleCount = 0;
    g_align.endSampleCount = 0;
    g_align.endSampleHead = 0;
    g_align.endLogged = false;
    g_align.lastSeenPvtAgeMs = 0xFFFFFFFFu;
    g_align.samplePushCount = 0;
    g_align.lastSamplePushedMs = 0;
    g_heading.alignStartedAtMs = g_align.startedAtMs;
    g_heading.alignState = AlignState::RUNNING;
    g_heading.lastAlignError = nullptr;
    Serial.printf("[ALIGN-RTK] start x=%.3f y=%.3f hAcc=%.3f source=%s\n",
                  (double)e.x, (double)e.y, (double)e.hAcc,
                  fromWebSocket ? "websocket" : "serial");
    return true;
}

// ============================================================================
// Alignment sample-window helpers
// ============================================================================
//
// The first ALIGN_WINDOW_CAP valid PVT samples are recorded verbatim into
// g_align.startSamples; the most recent ALIGN_WINDOW_CAP valid samples are
// kept in a ring buffer in g_align.endSamples. The full chronological log
// in g_align.samples (size ALIGN_SAMPLE_CAP) is only used for segment-
// heading stability analysis and is permitted to wrap on long alignments.
//
// Final heading is computed as
//   startMean = mean(startSamples[startSampleCount])
//   endMean   = mean(endSamples[endSampleCount])
//   headingDeg = atan2(endMean.x - startMean.x, endMean.y - startMean.y)
// so a 2 cm RTK jitter on any single PVT cannot bend the result by tens of
// degrees, regardless of alignment duration.

static int alignEndSampleIndex(int logicalIdx) {
    // logicalIdx ∈ [0, endSampleCount); oldest end-sample is at
    // (endSampleHead - endSampleCount) mod ALIGN_WINDOW_CAP, wrapped.
    const int oldest = (g_align.endSampleHead - g_align.endSampleCount);
    int i = (oldest + logicalIdx) % ALIGN_WINDOW_CAP;
    if (i < 0) i += ALIGN_WINDOW_CAP;
    return i;
}

static int alignMovingIndex(int logicalIdx) {
    // Chronological index in the DRIVE ring buffer; oldest entry at
    // (movingSampleHead - movingSampleCount) mod ALIGN_MOVING_CAP.
    const int oldest = (g_align.movingSampleHead - g_align.movingSampleCount);
    int i = (oldest + logicalIdx) % ALIGN_MOVING_CAP;
    if (i < 0) i += ALIGN_MOVING_CAP;
    return i;
}

static void alignStartMean(float& outX, float& outY, float& outHAcc, int& outN) {
    outX = 0; outY = 0; outHAcc = 0;
    outN = g_align.startSampleCount;
    if (outN <= 0) return;
    for (int i = 0; i < outN; ++i) {
        outX    += g_align.startSamples[i].x;
        outY    += g_align.startSamples[i].y;
        outHAcc += g_align.startSamples[i].hAcc;
    }
    const float inv = 1.0f / (float)outN;
    outX *= inv; outY *= inv; outHAcc *= inv;
}

static void alignEndMean(float& outX, float& outY, float& outHAcc, int& outN) {
    outX = 0; outY = 0; outHAcc = 0;
    outN = g_align.endSampleCount;
    if (outN <= 0) return;
    for (int i = 0; i < outN; ++i) {
        const int idx = alignEndSampleIndex(i);
        outX    += g_align.endSamples[idx].x;
        outY    += g_align.endSamples[idx].y;
        outHAcc += g_align.endSamples[idx].hAcc;
    }
    const float inv = 1.0f / (float)outN;
    outX *= inv; outY *= inv; outHAcc *= inv;
}

// RMS radius of a stationary window around its mean. Real FIXED RTK
// at ~1.4 cm horizontal accuracy shows 0.01..0.03 m RMS over a 0.5 s
// stationary window. Anything > ALIGN_JITTER_MAX_M means multipath /
// mode flip / unhealthy solution and we abort.
static float alignStartRmsRadius() {
    const int n = g_align.startSampleCount;
    if (n <= 0) return 0;
    float mx = 0, my = 0;
    for (int i = 0; i < n; ++i) {
        mx += g_align.startSamples[i].x;
        my += g_align.startSamples[i].y;
    }
    mx /= (float)n; my /= (float)n;
    float s = 0;
    for (int i = 0; i < n; ++i) {
        const float dx = g_align.startSamples[i].x - mx;
        const float dy = g_align.startSamples[i].y - my;
        s += dx * dx + dy * dy;
    }
    return sqrtf(s / (float)n);
}

static float alignEndRmsRadius() {
    const int n = g_align.endSampleCount;
    if (n <= 0) return 0;
    float mx = 0, my = 0;
    for (int i = 0; i < n; ++i) {
        const int idx = alignEndSampleIndex(i);
        mx += g_align.endSamples[idx].x;
        my += g_align.endSamples[idx].y;
    }
    mx /= (float)n; my /= (float)n;
    float s = 0;
    for (int i = 0; i < n; ++i) {
        const int idx = alignEndSampleIndex(i);
        const float dx = g_align.endSamples[idx].x - mx;
        const float dy = g_align.endSamples[idx].y - my;
        s += dx * dx + dy * dy;
    }
    return sqrtf(s / (float)n);
}

// Standard deviation (degrees) of segment headings between DRIVE samples
// spaced at least ALIGN_SEG_MIN_DIST_M apart. Returns -1 when not enough
// data. Uses circular stddev so wrap-around at 360° is OK.
static float alignSegmentHeadingStdDeg() {
    const int n = g_align.movingSampleCount;
    if (n < 4) return -1.0f;
    const int stride = n / 4;
    if (stride < 2) return -1.0f;
    float headings[16];
    int hcount = 0;
    for (int a = 0; a + stride < n && hcount < 16; a += stride) {
        const int b = a + stride;
        const int idxA = alignMovingIndex(a);
        const int idxB = alignMovingIndex(b);
        const float dx = g_align.movingSamples[idxB].x - g_align.movingSamples[idxA].x;
        const float dy = g_align.movingSamples[idxB].y - g_align.movingSamples[idxA].y;
        if (sqrtf(dx * dx + dy * dy) < ALIGN_SEG_MIN_DIST_M) continue;
        headings[hcount++] = ImuMath::rtkForwardHeadingDeg(dx, dy);
    }
    if (hcount < 2) return -1.0f;
    float sx = 0, sy = 0;
    for (int i = 0; i < hcount; ++i) {
        const float rad = headings[i] * (float)M_PI / 180.0f;
        sx += cosf(rad);
        sy += sinf(rad);
    }
    const float R = sqrtf(sx * sx + sy * sy) / (float)hcount;
    if (R >= 1.0f) return 0.0f;
    return acosf(R) * 180.0f / (float)M_PI;
}

// Push a fresh PVT sample into the window that matches the current
// phase. START_SETTLE fills startSamples, DRIVE fills the moving debug
// ring, END_SETTLE fills endSamples. Caller is responsible for ensuring
// this is a NEW PVT, not a duplicate loop tick — see the
// `lastSeenPvtAgeMs` de-dup in stepAlign().
static void alignPushSample(float x, float y, float hAcc, uint32_t tMs) {
    switch (g_align.phase) {
        case AlignPhase::START_SETTLE: {
            if (g_align.startSampleCount < ALIGN_WINDOW_CAP) {
                g_align.startSamples[g_align.startSampleCount] = {x, y, hAcc, tMs};
                g_align.startSampleCount++;
            }
            if (!g_align.startLogged && g_align.startSampleCount >= ALIGN_WINDOW_CAP) {
                g_align.startLogged = true;
                Serial.printf("[ALIGN-RTK] start window filled n=%d jitter=%.3f\n",
                              g_align.startSampleCount,
                              (double)alignStartRmsRadius());
            }
            break;
        }
        case AlignPhase::DRIVE: {
            const int slot = g_align.movingSampleHead % ALIGN_MOVING_CAP;
            g_align.movingSamples[slot] = {x, y, hAcc, tMs};
            g_align.movingSampleHead = (g_align.movingSampleHead + 1) % ALIGN_MOVING_CAP;
            if (g_align.movingSampleCount < ALIGN_MOVING_CAP) {
                g_align.movingSampleCount++;
            }
            g_align.driveSampleCount++;
            break;
        }
        case AlignPhase::END_SETTLE: {
            const int slot = g_align.endSampleHead % ALIGN_WINDOW_CAP;
            g_align.endSamples[slot] = {x, y, hAcc, tMs};
            g_align.endSampleHead = (g_align.endSampleHead + 1) % ALIGN_WINDOW_CAP;
            if (g_align.endSampleCount < ALIGN_WINDOW_CAP) {
                g_align.endSampleCount++;
            }
            if (!g_align.endLogged && g_align.endSampleCount >= ALIGN_WINDOW_CAP) {
                g_align.endLogged = true;
                Serial.printf("[ALIGN-RTK] end window filled n=%d jitter=%.3f\n",
                              g_align.endSampleCount,
                              (double)alignEndRmsRadius());
            }
            break;
        }
    }
    // Per-sample debug line — fires once per fresh PVT, not once per
    // loop tick. Useful when diagnosing "samples = 1, timeout" without
    // having to enable the [NAVV2] stream.
    g_align.samplePushCount++;
    Serial.printf("[ALIGN-RTK] sample pushed phase=%d n=%u age=%u x=%.3f y=%.3f hAcc=%.3f\n",
                  (int)g_align.phase,
                  (unsigned)g_align.samplePushCount,
                  (unsigned)g_align.lastSeenPvtAgeMs,
                  (double)x, (double)y, (double)hAcc);
}

// Helper: derive mean+jitter of the start window without leaving phase
// START_SETTLE. Used for the early-jitter reject inside START_SETTLE.
static void alignStartCheck(float& outX, float& outY, float& outHAcc,
                            float& outJitter, int& outN) {
    alignStartMean(outX, outY, outHAcc, outN);
    outJitter = alignStartRmsRadius();
}

static void alignEndCheck(float& outX, float& outY, float& outHAcc,
                          float& outJitter, int& outN) {
    alignEndMean(outX, outY, outHAcc, outN);
    outJitter = alignEndRmsRadius();
}

// Phase-aware abort helper used by stepAlign and the loop-time safety
// check. Caller has already filled g_heading.lastAlignError with `err`.
static void alignAbort(const char* err, const Estimate& e, uint32_t nowMs) {
    g_align.active = false;
    g_motor.stopImmediately();
    g_heading.alignState = AlignState::ERR;
    g_heading.lastAlignError = err;
    g_heading.lastAlignHAcc = e.hAcc;
    Serial.printf("[ALIGN-RTK] abort: %s reason=%s level=%d "
                  "pvtAge=%u maxPvtAge=%u hAcc=%.3f rtkStatus=%d rtcmAge=%u phase=%d\n",
                  err,
                  g_safety.reason(),
                  (int)g_safety.level(),
                  (unsigned)e.pvtAgeMs,
                  (unsigned)ALIGN_MAX_PVT_AGE_MS,
                  (double)e.hAcc,
                  (int)e.sol,
                  (unsigned)g_rtcm.transportAgeMs(nowMs),
                  (int)g_align.phase);
    Serial.printf("AUTO_ALIGN_HEADING_BY_RTK,ERR,%s,%s\n",
                  err, g_safety.reason());
}

// Finalize the run, fill g_heading.lastAlign* and either mark OK or ERR.
// Returns true if the run completed OK (heading trusted).
static bool alignFinalize(const Estimate& e, uint32_t now, uint32_t elapsedMs,
                          const char*& errOut) {
    errOut = nullptr;
    const int minRequired = 5;
    if (g_align.startSampleCount < minRequired || g_align.endSampleCount < minRequired) {
        g_heading.lastAlignError = "not_enough_samples";
        g_heading.lastAlignDist = 0; g_heading.lastAlignDx = 0; g_heading.lastAlignDy = 0;
        alignAbort("not_enough_samples", e, now);
        errOut = "not_enough_samples";
        return false;
    }

    float sx = 0, sy = 0, ex = 0, ey = 0, shAcc = 0, ehAcc = 0;
    float jitStart = 0, jitEnd = 0;
    int startN = 0, endN = 0;
    alignStartCheck(sx, sy, shAcc, jitStart, startN);
    alignEndCheck(ex, ey, ehAcc, jitEnd, endN);

    const float dx = ex - sx;
    const float dy = ey - sy;
    const float dist = sqrtf(dx * dx + dy * dy);
    if (dist < ALIGN_MIN_DIST_HARD) {
        g_heading.lastAlignError = "not_enough_motion";
        g_heading.lastAlignDist = dist; g_heading.lastAlignDx = dx; g_heading.lastAlignDy = dy;
        g_heading.lastAlignHAcc = (shAcc + ehAcc) * 0.5f;
        alignAbort("not_enough_motion", e, now);
        errOut = "not_enough_motion";
        return false;
    }
    if (dist < ALIGN_MIN_TRAVEL_M) {
        g_heading.lastAlignError = "not_enough_motion";
        g_heading.lastAlignDist = dist; g_heading.lastAlignDx = dx; g_heading.lastAlignDy = dy;
        g_heading.lastAlignHAcc = (shAcc + ehAcc) * 0.5f;
        alignAbort("not_enough_motion", e, now);
        errOut = "not_enough_motion";
        return false;
    }

    const float stdDev = alignSegmentHeadingStdDeg();
    if (stdDev >= 0.0f && stdDev > ALIGN_HEADING_STD_MAX_DEG) {
        g_heading.lastAlignError = "unstable_heading";
        g_heading.lastAlignDist = dist; g_heading.lastAlignDx = dx; g_heading.lastAlignDy = dy;
        g_heading.lastAlignHAcc = (shAcc + ehAcc) * 0.5f;
        alignAbort("unstable_heading", e, now);
        errOut = "unstable_heading";
        return false;
    }

    const float headingDeg = ImuMath::rtkForwardHeadingDeg(dx, dy);
    g_heading.lastAlignHeading = headingDeg;
    g_heading.lastAlignDist = dist;
    g_heading.lastAlignDx = dx;
    g_heading.lastAlignDy = dy;
    g_heading.lastAlignHAcc = (shAcc + ehAcc) * 0.5f;
    g_heading.lastAlignError = nullptr;
    g_heading.alignState = AlignState::OK;
    // Promote to "PLUS_IMU": the RTK motion gave us an absolute heading
    // baseline, and from now on we let the BNO085 gyro / estimator
    // track rotation. HeadingMgr.headingDeg is updated each tick from
    // the live estimator (see loop() plumbing) so the operator-facing
    // heading rotates with the chassis.
    g_heading.source = HeadingSource::RTK_MOTION_ALIGNED_PLUS_IMU;
    g_heading.headingDeg = headingDeg;
    g_heading.trusted = true;
    g_heading.trustedAtMs = now;
    g_est.seedHeadingDeg(headingDeg, ImuYawSource::ROTATION_VECTOR);
    g_align.abortedByStop = false;

    Serial.printf("[ALIGN-RTK] startMean=(%.3f,%.3f) endMean=(%.3f,%.3f) "
                  "dist=%.3f dx=%.3f dy=%.3f hAccMean=%.3f "
                  "jitterStart=%.3f jitterEnd=%.3f method=stationary_start_end\n",
                  (double)sx, (double)sy, (double)ex, (double)ey,
                  (double)dist, (double)dx, (double)dy,
                  (double)((shAcc + ehAcc) * 0.5f),
                  (double)jitStart, (double)jitEnd);
    Serial.printf("[ALIGN-RTK] OK heading=%.1f elapsed=%u\n",
                  (double)headingDeg, (unsigned)elapsedMs);
    return true;
}

// Called from main loop() once per iteration. Advances the alignment state
// machine. Sets `stopMotorsOut` to true when the run is finished so the
// caller should zero motor commands next tick regardless of branch.
static void stepAlign(bool& stopMotorsOut, const char*& errOut) {
    stopMotorsOut = false;
    errOut = nullptr;
    if (g_heading.alignState != AlignState::RUNNING || !g_align.active) return;

    const auto& e = g_est.get();
    const uint32_t now = millis();

    // RTK lost/stale → abort, from any phase.
    if (!rtkAcceptableForAlign(e)) {
        g_align.abortedByRtk = true;
        alignAbort("rtk_lost", e, now);
        stopMotorsOut = true;
        errOut = "rtk_lost";
        return;
    }

    // Hard overall timeout. We let finalize() decide the error string,
    // so a timeout that left us short of motion is logged as
    // `not_enough_motion` rather than `timeout`.
    const uint32_t elapsed = now - g_align.startedAtMs;
    if (elapsed > ALIGN_TIMEOUT_MS) {
        g_align.active = false;
        if (!alignFinalize(e, now, elapsed, errOut)) {
            // alignFinalize already logged + set state.
            Serial.println("[ALIGN-RTK] abort: timeout");
            if (g_heading.lastAlignError == nullptr) {
                g_heading.lastAlignError = "timeout";
            }
            errOut = g_heading.lastAlignError ? g_heading.lastAlignError : "timeout";
        }
        stopMotorsOut = true;
        return;
    }

    // De-dup: push only when a fresh PVT is observed.
    //
    // The Estimate's pvtAgeMs resets to ~0 each time the parser delivers
    // a new PVT; on subsequent loop ticks it grows monotonically. We
    // record `lastSeenPvtAgeMs` EVERY loop (regardless of whether we push)
    // so that "fresh PVT" = "pvtAgeMs dropped since we last observed it"
    // is detected even on the second and subsequent PVTs of the run.
    const uint32_t age = e.pvtAgeMs;
    bool newPvt = false;
    if (g_align.lastSeenPvtAgeMs == 0xFFFFFFFFu) {
        // Very first tick of the run: accept regardless of age.
        newPvt = true;
    } else if (age < g_align.lastSeenPvtAgeMs) {
        newPvt = true;
    }
    g_align.lastSeenPvtAgeMs = age;

    if (!newPvt) {
        // No new PVT this tick. Раньше код продолжал setLinearAngularSpeed(...)
        // в DRIVE — если PVT реально пропал, ровер продолжал ехать по инерции
        // прошлой команды. Теперь: если новых PVT не было дольше ALIVE_MS —
        // abort. Это решает "ровер ехал молча после последнего sample n=22
        // и не останавливался" (наблюдалось 2026-07-06 в полевом тесте).
        if (g_align.phase == AlignPhase::DRIVE) {
            // Грубая защита: если цикл без свежего PVT длиннее ALIVE_MS —
            // гарантированно abort. Мотор стопит alignAbort() сам.
            constexpr uint32_t ALIGN_DRIVE_ALIVE_MS = 2000u;
            if (now - g_align.lastSamplePushedMs > ALIGN_DRIVE_ALIVE_MS) {
                alignAbort("pvt_dropped_in_drive", e, now);
                stopMotorsOut = true;
                errOut = "pvt_dropped_in_drive";
                return;
            }
            g_motor.setLinearAngularSpeed(ALIGN_FORWARD_MPS, 0.0f, true);
        }
        return;
    }
    // Новый PVT пришёл — фиксируем момент для alive-watchdog'а выше.
    g_align.lastSamplePushedMs = now;
    alignPushSample(e.x, e.y, e.hAcc, now);

    // Phase transitions and per-phase behaviour.
    switch (g_align.phase) {
        case AlignPhase::START_SETTLE: {
            if (g_align.startSampleCount >= ALIGN_WINDOW_CAP) {
                float sx = 0, sy = 0, shAcc = 0, jit = 0;
                int n = 0;
                alignStartCheck(sx, sy, shAcc, jit, n);
                if (jit > ALIGN_JITTER_MAX_M) {
                    alignAbort("rtk_jitter_too_high", e, now);
                    stopMotorsOut = true;
                    errOut = "rtk_jitter_too_high";
                    return;
                }
                Serial.printf("[ALIGN-RTK] phase=START_SETTLE done jitter=%.3f -> DRIVE\n",
                              (double)jit);
                g_align.phase = AlignPhase::DRIVE;
                g_align.phaseStartedAtMs = now;
                Serial.println("[ALIGN-RTK] phase=DRIVE");
            }
            // Stay stationary: don't drive the motor while collecting
            // the start window.
            return;
        }
        case AlignPhase::DRIVE: {
            // Check distance from the start window mean. We have at
            // least ALIGN_WINDOW_CAP stationary start samples so the
            // mean is meaningful.
            float sx = 0, sy = 0, shAcc = 0, jit = 0;
            int n = 0;
            alignStartCheck(sx, sy, shAcc, jit, n);
            const float dx = e.x - sx;
            const float dy = e.y - sy;
            const float dist = sqrtf(dx * dx + dy * dy);
            if (dist >= ALIGN_MAX_DIST_M) {
                Serial.printf("[ALIGN-RTK] phase=DRIVE dist=%.3f -> END_SETTLE\n",
                              (double)dist);
                g_align.phase = AlignPhase::END_SETTLE;
                g_align.phaseStartedAtMs = now;
                g_align.endSampleCount = 0;
                g_align.endSampleHead = 0;
                g_align.endLogged = false;
                Serial.println("[ALIGN-RTK] phase=END_SETTLE");
                // Stop the motors immediately; the END_SETTLE window is
                // stationary.
                g_motor.stopImmediately();
                stopMotorsOut = true;
                return;
            }
            g_motor.setLinearAngularSpeed(ALIGN_FORWARD_MPS, 0.0f, true);
            return;
        }
        case AlignPhase::END_SETTLE: {
            if (g_align.endSampleCount >= ALIGN_WINDOW_CAP) {
                float ex = 0, ey = 0, ehAcc = 0, jit = 0;
                int n = 0;
                alignEndCheck(ex, ey, ehAcc, jit, n);
                if (jit > ALIGN_JITTER_MAX_M) {
                    alignAbort("rtk_jitter_too_high", e, now);
                    stopMotorsOut = true;
                    errOut = "rtk_jitter_too_high";
                    return;
                }
                Serial.printf("[ALIGN-RTK] phase=END_SETTLE done jitter=%.3f -> FINALIZE\n",
                              (double)jit);
                alignFinalize(e, now, elapsed, errOut);
                stopMotorsOut = true;
                return;
            }
            // Stay stationary while the end window fills.
            return;
        }
    }
}

static void clearHeadingTrust() {
    g_heading.trusted = false;
    g_heading.source = HeadingSource::NONE;
    g_heading.headingDeg = 0.0f;
    g_heading.trustedAtMs = 0;
    g_heading.lastAlignError = nullptr;
    // Leave alignState / lastAlignHeading intact for diagnostics.
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(200);
    Serial.println("\n[ROVER] boot");

    pinMode(PIN_RELAY_ATTACH, OUTPUT); digitalWrite(PIN_RELAY_ATTACH, LOW);
    pinMode(PIN_RELAY_MOUNT,  OUTPUT); digitalWrite(PIN_RELAY_MOUNT,  LOW);

    // Мотор ИНИЦИАЛИЗИРУЕМ ПЕРВЫМ (как в sound.ino): Serial2.begin + первый пакет 0,0
    // ДО любого WiFi/IMU/GNSS — чтобы плата hoverboard сразу увидела валидный протокол
    // и не уходила в защитный писк «нет сигнала».
    g_motor.begin(MotorSerial, ROVER_WHEELBASE_M, PIN_MOTOR_RX, PIN_MOTOR_TX);
    g_motor.stopImmediately();
    // КЛЮЧЕВОЕ: поток команд 50Гц ведёт отдельная задача на ядре 0. Иначе блокирующие
    // init ниже (g_gnss.begin до 1.5с ретраев, g_imu.begin I2C-скан, WiFi) РВУТ поток
    // на главном ядре → плата hoverboard видит пропажу сигнала после первого пакета и
    // пищит. Задача шлёт ровный 50Гц независимо от любых блокировок setup()/loop().
    g_motor.startTxTask();
    Serial.printf("[ROVER] motor Serial2 ready RX=%d TX=%d @%d (50Hz TX task on core0)\n",
                  PIN_MOTOR_RX, PIN_MOTOR_TX, HOVER_BAUD);

    connectWiFi();

    g_imu.begin(PIN_IMU_SDA, PIN_IMU_SCL);
    g_gnss.begin(F9pSerial, GNSS_ROVER);
    g_est.begin();
    float startupHeading = 0.0f;
    if (waitForInitialAbsoluteHeading(IMU_STARTUP_WAIT_MS, startupHeading)) {
        g_est.seedHeadingDeg(startupHeading, g_imu.yawSource());
        Serial.printf("[ROVER] heading seed: %.1f° source=%d abs=1 raw=%.1f mount=%.1f sign=%.1f mag=%d acc=%.2f state=%d\n",
                      (double)startupHeading, (int)g_imu.yawSource(),
                      (double)g_imu.rawYawDeg(),
                      (double)IMU_ROT_YAW_OFFSET_DEG,
                      (double)IMU_ROT_YAW_SIGN,
                      g_imu.yawFromMag() ? 1 : 0,
                      (double)g_imu.yawAccRad(),
                      (int)g_imu.headingState());
    } else {
        Serial.println("[IMU] absolute heading unavailable; game vector is relative; route start blocked");
    }
    g_route.begin();
    g_safety.begin();

    g_rtcm.begin(g_gnss, BASE_IP, RTCM_TCP_PORT, RTCM_UDP_PORT);
    g_ws.begin(g_est, g_imu, g_gnss, g_rtcm, g_route, g_motor, g_safety, WS_PORT);

    Serial.println("[ROVER] ready");
}

void loop() {
    uint32_t now = millis();
    static uint32_t lastBat = 0;

    // Простые текстовые команды из Serial — для отладки без приложения.
    //   CAL\n   — текущее направление носа = "0°". После этого робот поедет туда,
    //              куда сейчас смотрит носом, при waypoint (x=0, y=+).
    //   GO / GO_FORWARD[,m] — ехать m метров по текущему абсолютному heading.
    //   GO_NORTH[,m]        — ехать к географическому северу (y+), не "от носа".
    //   STOP\n  — стоп моторов немедленно.
    //   LOG,0\n — выключить периодический лог (чтобы терминал не летал).
    //   LOG,1\n — включить обратно (200мс).
    if (Serial.available()) {
        char buf[32];
        size_t n = Serial.readBytesUntil('\n', buf, sizeof(buf) - 1);
        if (n > 0) {
            buf[n] = 0;
            if (strcmp(buf, "CAL") == 0) {
                roverdbg::handleCal();
            } else if (strcmp(buf, "IMU_STATUS") == 0) {
                Serial.println(roverdbg::imuStatusLine());
            } else if (strcmp(buf, "IMU_CAL_START") == 0 || strcmp(buf, "CAL") == 0) {
                Serial.println(roverdbg::imuCalStartLine());
            } else if (strcmp(buf, "IMU_CAL_SAVE") == 0) {
                Serial.println(roverdbg::imuCalSaveLine());
            } else if (strcmp(buf, "IMU_CAL_STATUS") == 0) {
                Serial.println(roverdbg::imuCalStatusLine());
            } else if (strcmp(buf, "IMU_CAL_CLEAR") == 0) {
                Serial.println(roverdbg::imuCalClearLine());
            } else if (strcmp(buf, "IMU_TARE_YAW") == 0) {
                Serial.println(roverdbg::imuTareYawLine());
            } else if (strcmp(buf, "IMU_TARE_PERSIST") == 0) {
                Serial.println(roverdbg::imuTarePersistLine());
            } else if (strncmp(buf, "IMU_SET_TRUE_HEADING,", 21) == 0) {
                float deg = atof(buf + 21);
                Serial.println(roverdbg::imuSetTrueHeadingLine(deg));
            } else if (strcmp(buf, "IMU_CLEAR_HEADING_CORRECTION") == 0) {
                Serial.println(roverdbg::imuClearHeadingCorrectionLine());
            } else if (strcmp(buf, "IMU_HEADING_TEST") == 0) {
                Serial.println(roverdbg::imuHeadingTestLine());
            } else if (strcmp(buf, "IMU_TRUST_CURRENT_HEADING_ONCE") == 0) {
                Serial.println(roverdbg::imuTrustCurrentHeadingOnceLine());
            } else if (strcmp(buf, "IMU_CLEAR_MANUAL_HEADING_TRUST") == 0) {
                Serial.println(roverdbg::imuClearManualHeadingTrustLine());
            } else if (strcmp(buf, "AUTO_ALIGN_HEADING_BY_RTK") == 0) {
                Serial.println(roverdbg::autoAlignHeadingByRtkLine());
            } else if (strcmp(buf, "HEADING_STATUS") == 0) {
                Serial.println(roverdbg::headingStatusLine());
            } else if (strcmp(buf, "CLEAR_HEADING_TRUST") == 0) {
                Serial.println(roverdbg::clearHeadingTrustLine());
            } else if (strcmp(buf, "NAV_START_AUTO_ALIGN") == 0) {
                Serial.println(roverdbg::navStartAutoAlignLine());
            } else if (strcmp(buf, "GO") == 0) {
                roverdbg::handleGoForward(ROVER_GO_DEFAULT_DISTANCE_M);
            } else if (strncmp(buf, "GO_FORWARD", 10) == 0) {
                float distance = ROVER_GO_DEFAULT_DISTANCE_M;
                if (buf[10] == ',') distance = atof(buf + 11);
                roverdbg::handleGoForward(distance);
            } else if (strncmp(buf, "GO_NORTH", 8) == 0) {
                float distance = ROVER_GO_DEFAULT_DISTANCE_M;
                if (buf[8] == ',') distance = atof(buf + 9);
                roverdbg::handleGoNorth(distance);
            } else if (strncmp(buf, "GO_L_SHAPE_DEBUG,", 17) == 0) {
                float firstM = 1.0f, turnDeg = 90.0f, secondM = 1.0f;
                const char* p = buf + 17;
                firstM = atof(p);
                p = strchr(p, ',');
                if (p) { turnDeg = atof(++p); p = strchr(p, ','); }
                if (p) { secondM = atof(++p); }
                Serial.println(roverdbg::handleGoLShapeDebugLine(firstM, turnDeg, secondM));
            } else if (strncmp(buf, "GO_SQUARE_DEBUG,", 16) == 0) {
                const float sideM = atof(buf + 16);
                Serial.println(roverdbg::handleGoSquareDebugLine(sideM));
            } else if (strncmp(buf, "GO_L_SHAPE", 10) == 0) {
                float firstM = 1.0f, turnDeg = 90.0f, secondM = 1.0f;
                const char* p = buf + 10;
                if (*p == ',') {
                    firstM = atof(++p);
                    p = strchr(p, ',');
                    if (p) turnDeg = atof(++p);
                    p = p ? strchr(p, ',') : nullptr;
                    if (p) secondM = atof(++p);
                }
                roverdbg::handleGoLShape(firstM, turnDeg, secondM);
            } else if (strcmp(buf, "STOP") == 0) {
                roverdbg::handleStopLine();
            } else if (strcmp(buf, "IMU_ZERO") == 0) {
                Serial.println(roverdbg::imuZeroLine());
            } else if (strcmp(buf, "IMU_DIAG") == 0) {
                Serial.println(roverdbg::imuDiagLine());
            } else if (strncmp(buf, "LOG,", 4) == 0) {
                int v = atoi(buf + 4);
                g_logEnabled = (v != 0);
                Serial.printf("[LOG] periodic log %s\n", g_logEnabled ? "ON (200ms)" : "OFF");
            } else if (buf[0] != 0) {
                Serial.printf("[?] unknown: '%s' (try CAL / GO / STOP / LOG,0 / LOG,1)\n", buf);
            }
        }
    }

    g_imu.loop();
    g_gnss.loop();
    g_rtcm.loop();
    // g_motor.loop() НЕ зовём — поток команд ведёт TX-задача на ядре 0 (startTxTask).

    // WiFi TX-power оставляем 19.5 dBm постоянно. Раньше снижали до 11 dBm чтобы не
    // наводить помехи на UART2 → hoverboard, но с тех пор TX-задача мотора переехала
    // на ядро 0 и помехи ушли. 11 dBm даёт нестабильную связь → реконнекты WebSocket.
    // Если hoverboard снова запищит — вернём динамическое снижение, но это не наш кейс.

    if (g_gnss.consumeFreshPvt()) {
        g_est.onPvt(now,
            g_gnss.lastLatE7(), g_gnss.lastLonE7(), g_gnss.lastHeightMm(),
            g_gnss.lastHAccMm(), g_gnss.lastVAccMm(),
            g_gnss.lastGSpeedMmps(), g_gnss.lastHeadMotDe5(),
            g_gnss.lastFixType(), g_gnss.lastCarrierSol(), g_gnss.lastDiffSoln(),
            g_gnss.lastNumSv(), g_gnss.lastPDop(),
            g_gnss.lastHeadAccDe5());
    }
    // RTCM транспорт → возраст в эстимейторе (раньше est.rtcmAgeMs зеркалил
    // возраст PVT и телеметрия врала). Отмечаем каждый принятый UDP-пакет.
    {
        static uint32_t s_lastRtcmPkts = 0;
        if (g_rtcm.packets() != s_lastRtcmPkts) {
            s_lastRtcmPkts = g_rtcm.packets();
            g_est.onRtcmInfo(now, (int)g_gnss.rtcm().lastType,
                             (int)g_gnss.rtcm().msgCount,
                             (int)g_gnss.rtcm().crcFail);
        }
    }
    g_est.onImu(now, g_imu.yawRateDps(), g_imu.fresh() && g_imu.ageMs(now) < SAFE_IMU_AGE_MS,
                g_imu.yawDeg(), g_imu.yawAbsoluteValid(),
                g_imu.yawAccRad(), g_imu.yawSource(), g_imu.yawIsAbsolute());
    // Hoverboard feedback → EKF predict. TX-задача на ядре 0 обновляет _fb в Motor;
    // здесь читаем актуальные обороты. Если feedback ещё не пришёл, шлёт 0/0.
    g_est.onHoverboardFeedback(now, g_motor.speedLeftMeas(), g_motor.speedRightMeas());
    g_est.tick(now);

    // Refresh the operator-facing trusted heading from the live
    // estimator whenever we are in the RTK_MOTION_ALIGNED_PLUS_IMU mode.
    // BNO085 gyro integration keeps `headingFiltDeg` rotating with the
    // chassis after the one-shot RTK seed, so we mirror it into
    // g_heading.headingDeg each tick. `trustedAtMs` is bumped so the
    // freshness metric reflects real life rather than the alignment
    // moment. `lastAlignHeading` keeps the original RTK seed for
    // diagnostics (HEADING_STATUS field).
    if (g_heading.trusted &&
        g_heading.source == HeadingSource::RTK_MOTION_ALIGNED_PLUS_IMU) {
        const auto& eLive = g_est.get();
        if (eLive.headingValid) {
            g_heading.headingDeg = eLive.headingFiltDeg;
            g_heading.trustedAtMs = now;
        }
    }

    // === Safety tick BEFORE the alignment block ===
    //
    // Order matters: the alignment kill-switches read g_safety.level(),
    // and Serial AUTO_ALIGN_HEADING_BY_RTK relies on safety clearing
    // `ws_disconnected` via serialDebugMotion. We must run safety.tick()
    // first so the alignment block sees the up-to-date level. Otherwise
    // a fresh Serial AUTO_ALIGN would inherit the previous tick's
    // ESTOP `ws_disconnected` and abort immediately as `safety`.

    SafetyInput si;
    si.wsConnected  = g_ws.isConnected();
    si.lastWsRxMs   = g_ws.lastRxMs();
    si.lastCmdMs    = g_ws.lastCmdMs();
    si.navRequested = g_ws.navRequested();
    si.sol          = g_est.get().sol;
    si.numSv        = g_est.get().numSv;
    si.pDop         = g_est.get().pDop;
    si.hAcc         = g_est.get().hAcc;
    si.pvtAgeMs     = g_est.get().pvtAgeMs;
    si.rtcmAgeMs    = g_rtcm.transportAgeMs(now);
    si.acceptedPositionAgeMs = g_est.get().acceptedPositionAgeMs;
    si.headingAgeMs = g_est.get().headingAgeMs;
    si.rejectedPositionFixes = g_est.get().rejectedPositionFixes;
    si.originLocked = g_est.get().originSet;
    si.routeReady   = g_route.isReady();

    // Serial-issued motion override. We consider a session
    // "serial motion" when ANY of:
    //   - Serial AUTO_ALIGN is running (and not from WebSocket)
    //   - a Serial GO_FORWARD / GO_NORTH test command is active
    // When true, ws_disconnected is downgraded to OK in Safety::tick().
    // WebSocket-issued alignment is excluded so the WS link still
    // gates its own runs.
    const bool serialAlignActive =
        g_align.active &&
        g_heading.alignState == AlignState::RUNNING &&
        !g_align.fromWebSocket;
    si.serialDebugMotion = serialAlignActive || g_serialMotion.active;

    // rotatingInPlace: turn-in-place активен в follower'е, ИЛИ идёт
    // угловой поворот (precision corner turn в LTEST/SQTEST имеет
    // большую угловую скорость даже когда motor cmd=0 — гусеницы
    // инерциально катятся). hAcc в эти моменты кратко прыгает до
    // 2-3 см (нормальный F9P jitter на подвижной платформе, не потеря
    // FIXED) — Safety делает короткий tolerance на spike.
    //
    // Доступ к g_precision/PrecisionPhase идёт через roverdbg::, потому
    // что они объявлены в namespace roverdbg (см. ниже по файлу).
    si.rotatingInPlace = g_follow.turnInPlaceActive ||
        roverdbg::precisionIsRotating() ||
        fabsf(g_imu.yawRateDps()) > 5.0f;

    // Relaxed PVT staleness threshold while alignment is running (in
    // either Serial or WebSocket mode). Normal navigation keeps the
    // strict SAFE_PVT_AGE_MS so a stale estimator never silently drives.
    const bool alignmentRunning = (g_align.active &&
        g_heading.alignState == AlignState::RUNNING);
    si.maxPvtAgeMs = alignmentRunning
        ? ALIGN_MAX_PVT_AGE_MS
        : SAFE_PVT_AGE_MS;

    // Heading trust gate. Accept any of:
    //   - BNO085 absolute yaw OK (computed via canUseAbsoluteYawForNav)
    //   - manual trust flag set by IMU_TRUST_CURRENT_HEADING_ONCE
    //   - RTK-motion alignment completed this boot AND estimator heading
    //     is still fresh
    // `headingUsesImu` is true only when the trusted heading source IS
    // the IMU. RTK-motion aligned navigation does not depend on BNO085
    // being live, so imu_stale is not fatal there.
    {
        const auto& e = g_est.get();
        const bool imuAbsOk = ImuMath::canUseAbsoluteYawForNav(
                g_imu.headingState(),
                g_imu.yawAbsoluteValid(),
                g_imu.yawAccRad(),
                g_imu.yawAgeMs(now),
                SAFE_IMU_AGE_MS);
        const bool manualTrust = g_imu.manualYawTrusted();
        // RTK-motion aligned heading is treated as trusted when the
        // estimator heading is still fresh — both RTK_MOTION_ALIGNED
        // (one-shot, only valid before any chassis rotation) and
        // RTK_MOTION_ALIGNED_PLUS_IMU (live, with BNO085 gyro
        // integration through the estimator).
        const bool rtkAlignFresh =
            g_heading.trusted &&
            (g_heading.source == HeadingSource::RTK_MOTION_ALIGNED ||
             g_heading.source == HeadingSource::RTK_MOTION_ALIGNED_PLUS_IMU) &&
            e.headingValid &&
            e.headingAgeMs <= SAFE_HEADING_AGE_MS;
        si.headingTrustedForNav = imuAbsOk || manualTrust || rtkAlignFresh;
        // `headingUsesImu` is true only when navigation depends on
        // BNO085 being live right now. RTK_MOTION_ALIGNED_PLUS_IMU
        // also depends on BNO085 (for gyro integration), so it counts.
        si.headingUsesImu       = imuAbsOk || manualTrust ||
                                  (g_heading.trusted &&
                                   g_heading.source == HeadingSource::RTK_MOTION_ALIGNED_PLUS_IMU);
    }
    g_safety.tick(now, si, g_est, g_imu);

    // RTK motion alignment state machine. Runs AFTER Safety::tick so
    // it sees the up-to-date level — in particular, serialDebugMotion
    // has already cleared the previous ws_disconnected ESTOP before this
    // block decides whether to drive.
    //
    // Kill conditions:
    //   - safety ESTOP / HOLD (always, except ws_disconnected when
    //     alignment was started from Serial — that case is allowed to
    //     keep driving under serialDebugMotion, and Safety::tick()
    //     already downgraded it to OK before we got here)
    //   - STOP (any source, the explicit STOP command sets
    //     navRequested=false). WebSocket-issued alignment also aborts
    //     here on WS disconnect.
    if (g_heading.alignState == AlignState::RUNNING) {
        bool stopMotors = false;
        const char* alignErr = nullptr;
        // WebSocket-issued alignment: drop the run if the WS link
        // dropped (navRequested was cleared by WS_DISCONNECT).
        if (!g_ws.navRequested() && g_align.fromWebSocket) {
            stopMotors = true;
            alignErr = "ws_disconnected";
        } else if (g_safety.level() == SAFETY_ESTOP || g_safety.level() == SAFETY_HOLD) {
            // Any non-cleared safety fault kills the run. Note that
            // ws_disconnected has already been downgraded to OK by
            // Safety::tick() when serialDebugMotion=true, so this
            // branch will not trigger on a Serial alignment.
            stopMotors = true;
            alignErr = "safety";
        }
        if (stopMotors) {
            const uint32_t nowAbort = millis();
            alignAbort(alignErr, g_est.get(), nowAbort);
        } else {
            stepAlign(stopMotors, alignErr);
            if (stopMotors) g_motor.stopImmediately();
        }
    }

    // Operational origin is supplied by ROUTE_BEGIN from the saved map.
    // Current-position auto-origin exists only inside explicit GO_* test commands.
    if (!g_safety.allowMotion()) {
        digitalWrite(PIN_RELAY_ATTACH, LOW);
        digitalWrite(PIN_RELAY_MOUNT, LOW);
        g_motor.stopImmediately();
    }

    if (!g_ws.navRequested()) {
        g_follow.reset();
    }

    if (g_ws.navRequested() && g_route.isRunning()) {
        if (!g_follow.running) {
            g_follow.reset();
            g_follow.running = true;
        }
        stepFollower();
    }

    // Auto-clear the serialDebugMotion override once the test drive
    // completes (arrival / safety fault / timeout / STOP).
    serialMotionTick();
    roverdbg::lShapeOnTick();
    roverdbg::precisionDebugOnTick();

    NavStateOut nso;
    if (g_follow.faultReason != nullptr) {
        nso.state = NavStateOut::ERROR;
        nso.errorReason = g_follow.faultReason;
    } else if (g_safety.level() == SAFETY_ESTOP || g_safety.level() == SAFETY_HOLD) {
        nso.state = NavStateOut::ERROR;
        nso.errorReason = g_safety.reason();
    } else if (!g_route.isRunning() && g_route.isReady() && g_follow.arrived) {
        nso.state = NavStateOut::ARRIVED;
    } else if (g_route.isPaused()) {
        nso.state = NavStateOut::PAUSED;
    } else if (g_route.isRunning()) {
        nso.state = (g_follow.distToTarget < 0.30f) ? NavStateOut::APPROACHING : NavStateOut::RUNNING;
    } else {
        nso.state = NavStateOut::IDLE;
    }
    nso.wpIdx = g_follow.wpIdx;
    nso.wpTotal = g_route.count();
    nso.distToWp = g_follow.distToTarget;
    nso.headingErr = g_follow.headingErr;
    nso.crossTrack = g_follow.crossTrack;
    nso.lastLeftPwm  = g_motor.currentLeftPwm();
    nso.lastRightPwm = g_motor.currentRightPwm();
    g_ws.markNavUpdate(nso);

    g_ws.markTelemetryTick();
    g_ws.loop();

    if (now - lastBat > 1000) {
        lastBat = now;
        int pct = g_motor.batteryPercent();   // -1 если нет фидбэка от платы
        if (pct >= 0) g_ws.sendBattery(pct);
    }

    // Лог 200мс — видно каждый PVT. Поля подобраны под диагноз "почему не едет":
    //   sol / hAcc            — RTK фикс и точность
    //   head / imuYaw / mag   — heading (для проверки знака и стабильности)
    //   spd / pvtAge / rtcmAge / imuAge — свежесть данных
    //   wp / dWp / hErr / ct  — что Stanley пытается сделать
    //   motL / motR / sp / st — что мотор реально получил
    //   safety / reason       — почему может стоять
    //   fault                 — был ли fault на текущем WP
    static uint32_t lastLog = 0;
    if (g_logEnabled && now - lastLog > 200) {
        lastLog = now;
        const auto& e = g_est.get();
        const char* navState =
            g_follow.faultReason ? "ERROR" :
            g_route.isPaused() ? "PAUSED" :
            g_route.isRunning() ? "RUNNING" :
            g_follow.arrived ? "ARRIVED" : "IDLE";
        Serial.printf("[NAVV2] timestamp=%lu lat=%.8f lon=%.8f x=%.3f y=%.3f "
                      "imuYaw=%.1f imuRawYaw=%.1f imuState=%d imuSource=%d imuAbs=%d "
                      "imuMag=%d imuMagNorm=%.2f imuMagX=%.2f imuMagY=%.2f imuMagZ=%.2f "
                      "imuAcc=%.2f estimatorHeading=%.1f "
                      "headingUsed=%d absYaw=%.1f absYawValid=%d "
                      "target_x=%.3f target_y=%.3f target_heading=%.1f "
                      "heading_error=%.1f distance=%.3f "
                      "left_cmd=%d right_cmd=%d waypoint_index=%d/%d "
                      "nav_state=%s rtk_status=%d hAcc=%.3f "
                      "origin_lat=%.8f origin_lon=%.8f "
                      "speed=%.2f pvtAge=%u rtcmAge=%lu imuAge=%u "
                      "safety=%d reason=%s fault=%s "
                      "manualYawTrusted=%d manualYawTrustedHeading=%.1f "
                      "headingTrusted=%d headingSource=%s headingDeg=%.1f "
                      "alignState=%s lastAlignHeading=%.1f lastAlignDist=%.3f "
                      "wsTelSent=%u wsTelDropped=%u "
                      "serialMotion=%d serialMotionSource=%s wsConn=%d safety=%d reason=%s "
                      "imuAbsAge=%u imuRelYawAge=%u gyroAge=%u "
                      "odoL=%d odoR=%d gpsCourse=%.1f gpsCourseAcc=%.1f gpsCourseUsed=%d\n",
            (unsigned long)now, e.lat, e.lon, e.x, e.y,
            g_imu.yawDeg(), g_imu.rawYawDeg(), (int)g_imu.headingState(), (int)g_imu.yawSource(), g_imu.yawAbsoluteValid() ? 1 : 0,
            g_imu.yawFromMag() ? 1 : 0, g_imu.magNorm(),
            g_imu.magX(), g_imu.magY(), g_imu.magZ(),
            g_imu.yawAccRad(), e.headingFiltDeg,
            e.headingUsedByEstimator ? 1 : 0, e.absYawDeg, e.absYawValid ? 1 : 0,
            g_follow.targetX, g_follow.targetY, g_follow.targetHeadingDeg,
            g_follow.headingErr, g_follow.distToTarget,
            g_motor.currentLeftPwm(), g_motor.currentRightPwm(),
            g_follow.wpIdx, g_route.count(), navState, (int)e.sol, e.hAcc,
            e.originLat, e.originLon,
            e.speedMps, e.pvtAgeMs, g_rtcm.transportAgeMs(now), g_imu.ageMs(now),
            (int)g_safety.level(), g_safety.reason(),
            g_follow.faultReason ? g_follow.faultReason : "-",
            g_imu.manualYawTrusted() ? 1 : 0,
            (double)g_imu.manualTrustedHeadingDeg(),
            g_heading.trusted ? 1 : 0,
            headingSourceName(g_heading.source),
            (double)g_heading.headingDeg,
            alignStateName(g_heading.alignState),
            (double)g_heading.lastAlignHeading,
            (double)g_heading.lastAlignDist,
            (unsigned)g_ws.wsTelemetrySent(),
            (unsigned)g_ws.wsTelemetryDropped(),
            g_serialMotion.active ? 1 : 0,
            serialMotionSourceName(g_serialMotion.source),
            g_ws.isConnected() ? 1 : 0,
            (int)g_safety.level(),
            g_safety.reason(),
            (unsigned)g_imu.absYawAgeMs(now),
            (unsigned)g_imu.relYawAgeMs(now),
            (unsigned)g_imu.gyroAgeMs(now),
            // Одометрия сырьём: проверить знаки (вперёд → оба положительные?)
            // и масштаб kMeasToMps по фактической скорости.
            g_motor.speedLeftMeas(), g_motor.speedRightMeas(),
            (double)e.headingDeg,          // сырой GPS-курс
            (double)e.gpsCourseAccDeg,     // headAcc, deg
            e.gpsCourseUsed ? 1 : 0);      // курс скорректирован этим PVT
    }
}

// =============================================================================
// roverdbg:: — мосты из WsServer для отладочных команд (CAL / GO / LOG).
// Чтобы те же команды работали и из Serial Monitor, и из WebSocket-терминала
// приложения — без этого GO/CAL/LOG от приложения отвечают ERR,UNKNOWN.
// =============================================================================
namespace roverdbg {

static bool s_imuZeroed = false;
static float s0Yaw = 0, s0Raw = 0, s0Game = 0, s0Rot = 0, s0Geo = 0;
static float s0Gx = 0, s0Gy = 0, s0Gz = 0;
static float s0Mxy = 0, s0Myx = 0, s0Mxz = 0, s0Mzx = 0, s0Myz = 0, s0Mzy = 0;
static uint32_t s0CntMag = 0, s0CntGeo = 0, s0CntRot = 0, s0CntGame = 0, s0CntGyro = 0;

static float normDeg360Dbg(float d) {
    while (d < 0.0f) d += 360.0f;
    while (d >= 360.0f) d -= 360.0f;
    return d;
}

static float pairHeadingDeg(float a, float b) {
    return normDeg360Dbg(atan2f(b, a) * 180.0f / M_PI);
}

static void currentMagPairs(float& mxy, float& myx, float& mxz,
                            float& mzx, float& myz, float& mzy) {
    float mx = g_imu.magX();
    float my = g_imu.magY();
    float mz = g_imu.magZ();
    mxy = pairHeadingDeg(mx, my);
    myx = pairHeadingDeg(my, mx);
    mxz = pairHeadingDeg(mx, mz);
    mzx = pairHeadingDeg(mz, mx);
    myz = pairHeadingDeg(my, mz);
    mzy = pairHeadingDeg(mz, my);
}

// IMPORTANT: This used to be `handleCal()` and was gated on
// `IMU_ABSOLUTE_OK`. That was wrong — calibration is exactly what you have
// to run when the BNO085 is still ABSOLUTE_UNCALIBRATED, otherwise it can
// never become ABSOLUTE_OK. The old "[CAL] refusing: absolute heading
// unavailable" gate is gone.
//
// The legacy command name `CAL` is kept as an alias for `IMU_CAL_START`
// (BNO085 dynamic calibration) — that's what the user actually expects.
// A separate, strictly-gated helper `handleHeadingSeed()` exists for
// the operation that needs a valid absolute heading: reseeding the
// EKF from IMU. That's exposed via Serial only as `CAL_HEADING_SEED`,
// never as `CAL`.
bool handleCal() {
    // Calibration start is allowed from stale / unreliable absolute-yaw
    // states because calibration is exactly the mechanism that may recover
    // them. Navigation remains gated elsewhere (NAV_START / follower /
    // goPrecheck) until absolute yaw is actually valid.
    //
    // The only real failure mode here is "BNO085 never produced any data
    // at all" — i.e. sensor not initialised / not detected on I2C.
    // A 1000 ms "freshness" gate is too tight for this command: the
    // operator may legitimately issue IMU_CAL_START while acc is still
    // ramping up, or right after a brief I2C hiccup. We use a soft 5 s
    // bound so that an actual unresponsive sensor is still caught, but a
    // sensor that has `fresh=1` and just had a stale frame (~1 s) is not
    // refused with "IMU not responding".
    const uint32_t now = millis();
    const uint32_t age = g_imu.ageMs(now);
    constexpr uint32_t CAL_SOFT_STALE_MS = 5000u;

    if (!g_imu.hasData() || (!g_imu.fresh() && age > CAL_SOFT_STALE_MS)) {
        Serial.printf("[IMU-CAL] refusing: imu_not_initialized state=%s source=%s age=%u fresh=%d\n",
                      ImuMath::headingStateName(g_imu.headingState()),
                      ImuMath::yawSourceName(g_imu.yawSource()),
                      (unsigned)age, g_imu.fresh() ? 1 : 0);
        return false;
    }

    Serial.printf("[IMU-CAL] start requested state=%s source=%s acc=%.2f age=%u fresh=%d\n",
                  ImuMath::headingStateName(g_imu.headingState()),
                  ImuMath::yawSourceName(g_imu.yawSource()),
                  g_imu.yawAccRad(),
                  (unsigned)age, g_imu.fresh() ? 1 : 0);

    String err;
    if (!g_imu.startCalibration(&err)) {
        Serial.printf("[IMU-CAL] dynamic calibration failed: %s\n", err.c_str());
        return false;
    }
    Serial.println("[IMU-CAL] dynamic calibration started");
    return true;
}

// Strictly-gated: only valid when absolute yaw is already good. This is
// the operation `CAL` USED to mean; renamed so it's not accidentally
// confused with BNO085 dynamic calibration.
bool handleHeadingSeed() {
    if (!g_imu.yawAbsoluteValid() || g_imu.headingState() != ImuHeadingState::IMU_ABSOLUTE_OK) {
        Serial.printf("[CAL_HEADING_SEED] refusing: absolute heading unavailable state=%d source=%d acc=%.2f\n",
                      (int)g_imu.headingState(), (int)g_imu.yawSource(), g_imu.yawAccRad());
        return false;
    }
    float cur = g_imu.yawDeg();
    g_est.seedHeadingDeg(cur, g_imu.yawSource());
    g_follow.reset();
    Serial.printf("[CAL_HEADING_SEED] estimator reseeded from absolute imuYaw=%.1f source=%d\n",
                  (double)cur, (int)g_imu.yawSource());
    return true;
}

bool handleCalStart() {
    return handleCal();
}

bool handleCalSave() {
    String err;
    Serial.printf("[IMU-CAL] save requested state=%s acc=%.2f\n",
                  ImuMath::headingStateName(g_imu.headingState()),
                  g_imu.yawAccRad());
    if (!g_imu.saveCalibration(&err)) {
        Serial.printf("[IMU-CAL] DCD save failed: %s\n", err.c_str());
        return false;
    }
    Serial.println("[IMU-CAL] DCD save OK");
    return true;
}

bool handleGo() {
    return handleGoForward(ROVER_GO_DEFAULT_DISTANCE_M);
}

static bool goPrecheck() {
    // Heading gate. We accept a run when ANY of:
    //   - BNO085 absolute yaw OK (preferred path, calls
    //     canUseAbsoluteYawForNav below)
    //   - IMU_TRUST_CURRENT_HEADING_ONCE was issued this boot
    //   - RTK motion alignment succeeded this boot and the estimator
    //     is still tracking the heading (RTK_MOTION_ALIGNED or
    //     RTK_MOTION_ALIGNED_PLUS_IMU)
    // The "IMU dead" hard-gate that used to live here is gone: the BNO085
    // is allowed to be stale when we are running on RTK-aligned heading,
    // because the failure mode it protected against (heading frozen at
    // a stale value, Stanley turning the rover the wrong way) is no
    // longer relevant — RTK alignment already gave us an absolute
    // heading, and the BNO085 only contributes relative yaw on top.
    uint32_t tNow = millis();
    const bool imuAbsOk = ImuMath::canUseAbsoluteYawForNav(
            g_imu.headingState(),
            g_imu.yawAbsoluteValid(),
            g_imu.yawAccRad(),
            g_imu.yawAgeMs(tNow),
            SAFE_IMU_AGE_MS);
    const bool manualTrust = g_imu.manualYawTrusted();
    const auto& eGate = g_est.get();
    const bool rtkAlignFresh =
        g_heading.trusted &&
        (g_heading.source == HeadingSource::RTK_MOTION_ALIGNED ||
         g_heading.source == HeadingSource::RTK_MOTION_ALIGNED_PLUS_IMU) &&
        eGate.headingValid &&
        eGate.headingAgeMs <= SAFE_HEADING_AGE_MS;

    if (!imuAbsOk && !manualTrust && !rtkAlignFresh) {
        // Pick the most informative refusal reason.
        if (eGate.headingValid && eGate.headingAgeMs > SAFE_HEADING_AGE_MS) {
            Serial.printf("[GO] refusing: heading_stale ageMs=%u\n",
                          (unsigned)eGate.headingAgeMs);
        } else if (g_imu.ageMs(tNow) > SAFE_IMU_AGE_MS) {
            // Relative IMU path stale — distinct from the legacy
            // "IMU dead" wording because here we only care that
            // nothing else can rescue us. The estimator is allowed
            // to keep gyro integration via the gyro path for short
            // windows without BNO085 packet updates.
            Serial.printf("[GO] refusing: relative_imu_stale ageMs=%u\n",
                          (unsigned)g_imu.ageMs(tNow));
        } else {
            Serial.printf("[GO] refusing: absolute yaw unavailable state=%d source=%d acc=%.2f\n",
                          (int)g_imu.headingState(), (int)g_imu.yawSource(), g_imu.yawAccRad());
        }
        return false;
    }
    if (!imuAbsOk && (manualTrust || rtkAlignFresh)) {
        // Informative warning — we are starting without BNO085 absolute.
        if (rtkAlignFresh) {
            Serial.println("[NAV] WARNING using RTK-aligned heading (BNO085 absolute not required)");
        } else if (manualTrust) {
            Serial.println("[NAV] WARNING using manually trusted IMU heading; BNO absolute yaw is not valid");
        }
    }
    if (!eGate.headingValid) {
        Serial.println("[GO] refusing: estimator heading is not seeded");
        return false;
    }
    // Диагностика перед запуском — что может блокировать движение.
    const auto& e = g_est.get();
    uint32_t now = millis();
    Serial.printf("[GO] precheck: sol=%d hAcc=%.3f imuYaw=%.1f source=%d abs=%d state=%d mag=%d acc=%.2f "
                  "imuAge=%u pvtAge=%u rtcmAge=%lu motFB=%d "
                  "headingValid=%d wsConn=%d navReq=%d safety=%d reason=%s\n",
                  (int)e.sol, e.hAcc, g_imu.yawDeg(),
                  (int)g_imu.yawSource(), g_imu.yawAbsoluteValid() ? 1 : 0,
                  (int)g_imu.headingState(), g_imu.yawFromMag() ? 1 : 0, g_imu.yawAccRad(),
                  g_imu.ageMs(now), e.pvtAgeMs,
                  g_rtcm.transportAgeMs(now),
                  g_motor.haveFeedback() ? 1 : 0,
                  e.headingValid ? 1 : 0,
                  g_ws.isConnected() ? 1 : 0,
                  g_ws.navRequested() ? 1 : 0,
                  (int)g_safety.level(), g_safety.reason());

    return true;
}

// Busy-guard: любая debug-motion команда обязана отказаться, если
// уже идёт другой serial/debug autonomous motion. Иначе пользователь
// может запустить SQTEST поверх оборванного LTEST и получить два
// активных follower'а одновременно.
static bool serialMotionBusy() {
    return g_serialMotion.active;
}

bool handleGoForward(float distanceM) {
    if (serialMotionBusy()) {
        Serial.printf("[BUSY] GO_FORWARD rejected: serialMotionSource=%s active\n",
                      serialMotionSourceName(g_serialMotion.source));
        return false;
    }
    if (!goPrecheck()) return false;
    return startGoRoute("GO_FORWARD", distanceM, true);
}

bool handleGoNorth(float distanceM) {
    if (serialMotionBusy()) {
        Serial.printf("[BUSY] GO_NORTH rejected: serialMotionSource=%s active\n",
                      serialMotionSourceName(g_serialMotion.source));
        return false;
    }
    if (!goPrecheck()) return false;
    return startGoRoute("GO_NORTH", distanceM, false);
}

// GO_L_SHAPE,<first_m>,<turn_deg>,<second_m>
// Drives first_m meters from the current position along the current
// trusted heading (HeadingMgr.headingDeg), turns by turn_deg in place
// (heading is just rotated — no physical rotation step), then drives
// second_m meters along the new heading. Both segments are uploaded
// as a 2-waypoint route with a generous safety boundary. Like
// GO_FORWARD, this runs even without WebSocket via the serialMotion
// override.
bool handleGoLShape(float firstM, float turnDeg, float secondM) {
    if (!isfinite(firstM) || firstM <= 0.0f || firstM > 5.0f ||
        !isfinite(secondM) || secondM <= 0.0f || secondM > 5.0f ||
        !isfinite(turnDeg) || turnDeg < -360.0f || turnDeg > 360.0f) {
        Serial.printf("[GO_L_SHAPE] refusing: bad args first=%.2f turn=%.1f second=%.2f\n",
                      (double)firstM, (double)turnDeg, (double)secondM);
        return false;
    }
    if (!goPrecheck()) {
        Serial.println("[GO_L_SHAPE] refusing: goPrecheck failed");
        return false;
    }
    if (!g_est.get().originSet) {
        Serial.println("[GO_L_SHAPE] refusing: no origin");
        return false;
    }
    const auto& e = g_est.get();
    const float h0 = g_heading.trusted ? g_heading.headingDeg : e.headingFiltDeg;
    const float h1 = NavMath::normalizeDeg360(h0 + turnDeg);

    const float h0Rad = h0 * (float)M_PI / 180.0f;
    const float h1Rad = h1 * (float)M_PI / 180.0f;
    const float p0x = e.x;
    const float p0y = e.y;
    const float p1x = p0x + firstM * sinf(h0Rad);
    const float p1y = p0y + firstM * cosf(h0Rad);
    const float p2x = p1x + secondM * sinf(h1Rad);
    const float p2y = p1y + secondM * cosf(h1Rad);

    Serial.printf("[GO_L_SHAPE] first=%.2f turn=%.1f second=%.2f heading0=%.1f h1=%.1f "
                  "p0=(%.3f,%.3f) p1=(%.3f,%.3f) p2=(%.3f,%.3f)\n",
                  (double)firstM, (double)turnDeg, (double)secondM,
                  (double)h0, (double)h1,
                  (double)p0x, (double)p0y,
                  (double)p1x, (double)p1y,
                  (double)p2x, (double)p2y);

    g_follow.reset();
    if (!g_route.beginUpload(2, e.originLat, e.originLon)) {
        Serial.println("[GO_L_SHAPE] refusing: route begin failed");
        return false;
    }
    g_route.addWaypoint(0, p1x, p1y);
    g_route.addWaypoint(1, p2x, p2y);
    // Safety boundary: rectangle covering both segments with margin.
    const float margin = 0.75f;
    const float minX = fminf(fminf(p0x, p1x), p2x) - margin;
    const float maxX = fmaxf(fmaxf(p0x, p1x), p2x) + margin;
    const float minY = fminf(fminf(p0y, p1y), p2y) - margin;
    const float maxY = fmaxf(fmaxf(p0y, p1y), p2y) + margin;
    g_route.beginBoundary(4);
    g_route.addBoundaryPoint(0, minX, minY);
    g_route.addBoundaryPoint(1, maxX, minY);
    g_route.addBoundaryPoint(2, maxX, maxY);
    g_route.addBoundaryPoint(3, minX, maxY);
    g_route.endBoundary();
    g_route.beginForbidden(0, nullptr);
    g_route.endForbidden();
    g_route.endUpload();
    g_route.start();
    g_ws.requestDebugNavigation();
    serialMotionBegin(SerialMotionSource::GO_L_SHAPE, SERIAL_L_SHAPE_TIMEOUT_MS);
    return true;
}

static constexpr float DEBUG_PRECISION_RADIUS_M = 0.25f;
static constexpr float DEBUG_PRECISION_FINAL_RADIUS_M = 0.20f;
static constexpr float DEBUG_PRECISION_SPEED_MPS = 0.11f;
static constexpr float DEBUG_TURN_TOLERANCE_DEG = 7.0f;
static constexpr uint32_t DEBUG_SETTLE_MS = 300u;
static constexpr uint32_t DEBUG_TURN_HOLD_MS = 400u;
static constexpr uint32_t DEBUG_POST_TURN_SETTLE_MS = 800u;
// DEBUG_RAMP_UP_M — в RtkConfig.h (нужен stepFollower'у ниже по файлу).

// === Precision debug turn PWM profile ===
//
// Танковая платформа на малых радиусах перелетает target heading на
// полной угловой скорости (cmdL=-9 cmdR=9 = 1.5 wheels/sec). Делаем
// ступенчатый профиль, чтобы робот не «проскакивал» цель и плавно
// вставал в tolerance.
static constexpr int PRECISION_TURN_PWM_MAX = 6;   // abs(err) > 60°
static constexpr int PRECISION_TURN_PWM_MID = 5;   // 25..60°
static constexpr int PRECISION_TURN_PWM_LOW = 4;   // 10..25°
static constexpr int PRECISION_TURN_PWM_MIN = 3;   // < 10° (creep)
static constexpr uint32_t SERIAL_SQUARE_TIMEOUT_MS = 70000u;

enum class PrecisionPhase : uint8_t {
    IDLE,
    DRIVE,
    SETTLE,
    TURN,
    POST_TURN_SETTLE,
};

struct PrecisionDebugMotion {
    bool active = false;
    bool square = false;
    int segment = 0;
    int segmentCount = 0;
    float px[5]{};
    float py[5]{};
    float heading[4]{};
    float finalHeading = 0;
    PrecisionPhase phase = PrecisionPhase::IDLE;
    uint32_t phaseStartedMs = 0;
    uint32_t headingOkSinceMs = 0;
    int lastTurnPwm = 0;     // последняя PWM-ступень поворота (для лога при смене)
    const char* tag = "LTEST";
    // Опорная точка нового сегмента, фиксируется ЗДЕСЬ когда ровер
    // реально стоит после POST_TURN_SETTLE. Передаётся в uploadPrecisionSegment
    // и прибивается к g_follow.startX/startY напрямую — иначе Stanley бы
    // захватывал её на первом тике follower'а когда ровер ещё чуть-чуть
    // дрейфует, и первые 1-2 sample'а уже давали cross-track 0.2м.
    float segmentStartX = 0;
    float segmentStartY = 0;
};

static PrecisionDebugMotion g_precision;

bool precisionDebugActive() {
    return g_precision.active;
}

// True если идёт corner turn / settle / post-turn-settle в LTEST/SQTEST.
// В эти моменты hAcc кратко прыгает — Safety делает tolerance на spike.
bool precisionIsRotating() {
    if (!g_precision.active) return false;
    return g_precision.phase == PrecisionPhase::TURN ||
           g_precision.phase == PrecisionPhase::SETTLE ||
           g_precision.phase == PrecisionPhase::POST_TURN_SETTLE;
}

static bool uploadPrecisionSegment(int segment) {
    const auto& e = g_est.get();
    if (!e.originSet) return false;
    // Если в фазе POST_TURN_SETTLE уже зафиксировали опорную точку
    // (g_precision.segmentStartX/Y), используем ЕЁ как start of segment.
    // Иначе fallback на текущую позицию (первый сегмент LTEST/SQTEST).
    const bool haveAnchor = (segment > 0) || g_precision.segmentStartX != 0.0f ||
                            g_precision.segmentStartY != 0.0f;
    const float sx = haveAnchor ? g_precision.segmentStartX : e.x;
    const float sy = haveAnchor ? g_precision.segmentStartY : e.y;
    const float tx = g_precision.px[segment + 1];
    const float ty = g_precision.py[segment + 1];
    const float margin = 0.75f;
    const float minX = fminf(sx, tx) - margin;
    const float maxX = fmaxf(sx, tx) + margin;
    const float minY = fminf(sy, ty) - margin;
    const float maxY = fmaxf(sy, ty) + margin;

    g_follow.reset();
    // Если есть опорная точка — прибиваем её в follower заранее:
    // g_follow.startCaptured=true и g_follow.startX/startY заданы явно,
    // stepFollower() не будет "первого тика" захвата.
    if (haveAnchor && segment > 0) {
        g_follow.startX = sx;
        g_follow.startY = sy;
        g_follow.startCaptured = true;
    }
    if (!g_route.beginUpload(1, e.originLat, e.originLon)) return false;
    g_route.addWaypoint(0, tx, ty);
    g_route.beginBoundary(4);
    g_route.addBoundaryPoint(0, minX, minY);
    g_route.addBoundaryPoint(1, maxX, minY);
    g_route.addBoundaryPoint(2, maxX, maxY);
    g_route.addBoundaryPoint(3, minX, maxY);
    g_route.endBoundary();
    g_route.beginForbidden(0, nullptr);
    g_route.endForbidden();
    g_route.endUpload();
    g_route.start();
    g_ws.requestDebugNavigation();
    g_precision.phase = PrecisionPhase::DRIVE;
    g_precision.phaseStartedMs = millis();
    // anchor уже использован — обнуляем, чтобы следующий сегмент
    // (если это не corner handoff) считал свой собственный anchor.
    g_precision.segmentStartX = 0;
    g_precision.segmentStartY = 0;

    if (g_precision.square) {
        Serial.printf("[SQTEST] DRIVE_SEGMENT corner=%d start=(%.3f,%.3f) "
                      "target=(%.3f,%.3f)\n",
                      segment + 1,
                      (double)sx, (double)sy, (double)tx, (double)ty);
    } else {
        Serial.printf("[LTEST] DRIVE_SEGMENT logicalSegment=%d start=(%.3f,%.3f) "
                      "target=(%.3f,%.3f)\n",
                      segment,
                      (double)sx, (double)sy, (double)tx, (double)ty);
    }
    return true;
}

static float precisionTargetHeading() {
    if (g_precision.segment < g_precision.segmentCount - 1) {
        return g_precision.heading[g_precision.segment + 1];
    }
    return g_precision.finalHeading;
}

// Forward decl для фикса WP1_REACHED внутри finishPrecisionTurn.
static void lShapeRecordWp1();

static void beginPrecisionTurn(bool finalTurn) {
    const uint32_t now = millis();
    const float target = precisionTargetHeading();
    g_precision.phase = PrecisionPhase::TURN;
    g_precision.phaseStartedMs = now;
    g_precision.headingOkSinceMs = 0;
    if (g_precision.square) {
        Serial.printf("[SQTEST] CORNER_TURN corner=%d target=%.1f tolerance=%.1f\n",
                      g_precision.segment + 1,
                      (double)target,
                      (double)DEBUG_TURN_TOLERANCE_DEG);
    } else if (finalTurn) {
        Serial.printf("[LTEST] FINAL_TURN target=%.1f tolerance=%.1f\n",
                      (double)target,
                      (double)DEBUG_TURN_TOLERANCE_DEG);
    } else {
        Serial.printf("[LTEST] CORNER_TURN corner=%d target=%.1f tolerance=%.1f\n",
                      g_precision.segment + 1,
                      (double)target,
                      (double)DEBUG_TURN_TOLERANCE_DEG);
    }
}

static void finishPrecisionTurn(bool finalTurn, float err) {
    const auto& e = g_est.get();
    g_motor.stopImmediately();
    if (g_precision.square) {
        // SQTEST: фиксируем turnErr сегмента (g_precision.segment = только
        // что завершённый corner). corner N=segment+1 (1..4).
        if (g_precision.segment >= 0 && g_precision.segment < 4) {
            g_sqtest.segTurnErr[g_precision.segment] = err;
        }
        Serial.printf("[SQTEST] CORNER_TURN_DONE corner=%d heading=%.1f err=%+.1f\n",
                      g_precision.segment + 1,
                      (double)e.headingFiltDeg,
                      (double)err);
    } else if (finalTurn) {
        Serial.printf("[LTEST] FINAL_TURN_DONE heading=%.1f err=%+.1f\n",
                      (double)e.headingFiltDeg,
                      (double)err);
    } else {
        Serial.printf("[LTEST] CORNER_TURN_DONE corner=%d heading=%.1f err=%+.1f\n",
                      g_precision.segment + 1,
                      (double)e.headingFiltDeg,
                      (double)err);
    }

    // DRIVE_SEGMENT_DONE — фиксируем длину/cross-track сегмента, который
    // только что закончился. logicalSegment = g_precision.segment (старый,
    // ещё не инкрементированный).
    if (!g_precision.square) {
        const int doneSeg = g_precision.segment;
        if (doneSeg == 0) {
            Serial.printf("[LTEST] DRIVE_SEGMENT_DONE logicalSegment=0 "
                          "chord=%.3f pathLen=%.3f maxCross=%.3f\n",
                          (double)g_ltest.firstPlan,
                          (double)g_ltest.pathLen1,
                          (double)g_ltest.maxCross1);
        } else {
            Serial.printf("[LTEST] DRIVE_SEGMENT_DONE logicalSegment=1 "
                          "chord=%.3f pathLen=%.3f maxCross=%.3f\n",
                          (double)g_ltest.secondPlan,
                          (double)g_ltest.pathLen2,
                          (double)g_ltest.maxCross2);
        }
    }

    if (finalTurn) {
        // Последний сегмент завершён — фиксируем WP1, если это L-shape и
        // мы только что прошли logical segment 0 (один раз за тест).
        if (!g_precision.square) {
            lShapeRecordWp1();
        }
        g_precision.active = false;
        serialMotionEnd("arrived");
        return;
    }

    // Переход segment -> segment+1. До инкремента — это та точка, где
    // logical segment 0 (p0->p1) только что завершён и робот фактически
    // стоит на p1. Это идеальное место для WP1_REACHED в LTEST.
    if (!g_precision.square && g_precision.segment == 0) {
        lShapeRecordWp1();
    }

    g_precision.segment++;
    if (!uploadPrecisionSegment(g_precision.segment)) {
        g_precision.active = false;
        serialMotionEnd("route_begin_failed");
    }
}

// LTEST DRIVE PATH TRACE
//
// Этот trace вызывается из stepFollower() раз в секунду во время LTEST
// DRIVE-фазы и показывает ВСЕ поля, которые нужны для понимания, почему
// linear = 0: navRequested / routeRunning / serialMotion / allowMotion /
// speedBase / linearPreClamp / linearPostClamp / slowFactor / rampFactor /
// turnInPlaceActive / headingErr / cross / sol / hAcc / pvtAge /
// headingValid / headingAge / speedBlockedReason.
//
// Если строки НЕТ — stepFollower() вообще не вызывается (это будет
// означать что LTEST DRIVE идёт по другому пути).
// Если строка есть и все gate=true, но linPre=0 — баг в slow/ramp/turn
// расчёте. Если linPre != 0 а linPost=0 — баг в crosstrack-soft/hard gate.
// Если оба > 0, но мотор шлёт 0 — баг в Motor.cpp (не тот путь).
void ltestDriveTraceTick() {
    if (!g_precision.active) return;
    if (g_precision.phase != PrecisionPhase::DRIVE) return;
    static uint32_t s_lastMs = 0;
    const uint32_t now = millis();
    if ((now - s_lastMs) < 1000u) return;
    s_lastMs = now;

    const auto& e = g_est.get();
    const bool navOk   = g_ws.navRequested();
    const bool routeOk = g_route.isRunning();
    const int  pauseOk = g_route.isPaused() ? 0 : 1;
    const bool safeOk  = g_safety.allowMotion();
    const int  canDrive = (navOk && routeOk && (pauseOk != 0) && safeOk) ? 1 : 0;
    const float speedBase = (g_safety.level() == SAFETY_DEGRADED)
                            ? ROVER_FLOAT_SPEED : g_forwardSpeedMps;
    const char* block = g_follow.speedBlockedReason ? g_follow.speedBlockedReason : "-";
    Serial.printf("[LTEST-DRIVE-TRACE] t=%u seg=%d/%d navReq=%d routeRun=%d "
                  "paused=%d serialMotion=%d allowMotion=%d canDrive=%d "
                  "speedBase=%.2f linPre=%.3f linPost=%.3f slow=%.2f ramp=%.2f "
                  "turnIP=%d hErr=%+.1f cross=%+.3f "
                  "sol=%d hAcc=%.3f pvtAge=%u rtcmAge=%lu eHValid=%d eHAge=%u "
                  "block=%s\n",
        (unsigned)now,
        g_precision.segment, g_precision.segmentCount,
        (int)navOk, (int)routeOk, pauseOk,
        (int)g_serialMotion.active, (int)safeOk, canDrive,
        (double)speedBase,
        (double)g_follow.linearPreClamp,
        (double)g_follow.linearPostClamp,
        (double)g_follow.slowFactorDbg,
        (double)g_follow.rampFactorDbg,
        (int)g_follow.turnInPlaceActive,
        (double)g_follow.headingErr,
        (double)g_follow.crossTrack,
        (int)e.sol, (double)e.hAcc,
        (unsigned)e.pvtAgeMs,
        (unsigned long)g_rtcm.transportAgeMs(now),
        (int)e.headingValid,
        (unsigned)e.headingAgeMs,
        block);
}

// Гасит g_precision.active. Страховка от "LTEST-DRIVE-TRACE продолжает
// писать после safety/stop". Вызывается ТОЛЬКО из serialMotionEnd().
void precisionResetForSafetyEnd() {
    if (!g_precision.active) return;
    g_precision.active = false;
    g_precision.phase = PrecisionPhase::IDLE;
    g_precision.lastTurnPwm = 0;
    // motorStop НЕ делаем: serialMotionEnd() уже зовёт g_motor.stopImmediately()
    // через resetFollowerConfig() flow выше. Здесь только precision-state.
    Serial.println("[PRECISION-RESET] g_precision.active=false (safety/stop end)");
}

void precisionDebugOnTick() {
    if (!g_precision.active || !g_serialMotion.active) return;
    const uint32_t now = millis();
    const auto& e = g_est.get();

    if (g_precision.phase == PrecisionPhase::DRIVE && g_follow.arrived) {
        g_motor.stopImmediately();
        const bool finalSegment = g_precision.segment >= g_precision.segmentCount - 1;
        g_precision.phase = PrecisionPhase::SETTLE;
        g_precision.phaseStartedMs = now;
        if (g_precision.square) {
            // SQTEST: фиксируем фактический угол и метрики предыдущего
            // сегмента (только если это не первый corner = старт).
            const int segIdx = g_precision.segment;   // 0..3 = сегмент
            const int cornerIdx = segIdx + 1;          // 1..4 = corner
            if (segIdx >= 0 && segIdx < 4) {
                // cornerActualX/Y заполняется corner cornerIdx
                // (corner 1 = конец сегмента 0 и т.д.)
                g_sqtest.cornerActualX[segIdx] = e.x;
                g_sqtest.cornerActualY[segIdx] = e.y;
                const float cx = g_sqtest.pX[segIdx + 1];
                const float cy = g_sqtest.pY[segIdx + 1];
                const float err = lshapeDist2d(e.x, e.y, cx, cy);
                g_sqtest.cornerErr[segIdx] = err;
                // segment metrics: previous segment
                g_sqtest.segMaxCross[segIdx] = g_sqtest.maxCross;
                g_sqtest.segPathLen[segIdx] = g_sqtest.pathLen;
                Serial.printf("[SQTEST] CORNER_REACHED corner=%d actual=(%.3f,%.3f) "
                              "planned=(%.3f,%.3f) err=%.3f heading=%.1f\n",
                              cornerIdx,
                              (double)e.x, (double)e.y,
                              (double)cx, (double)cy,
                              (double)err, (double)e.headingFiltDeg);
                Serial.printf("[SQTEST] SEGMENT_DONE idx=%d chord=%.3f path=%.3f maxCross=%.3f\n",
                              cornerIdx, (double)g_sqtest.sidePlan,
                              (double)g_sqtest.segPathLen[segIdx],
                              (double)g_sqtest.segMaxCross[segIdx]);
            }
        } else if (finalSegment) {
            Serial.printf("[LTEST] FINAL_SETTLE pos=(%.3f,%.3f) heading=%.1f settleMs=%u\n",
                          (double)e.x, (double)e.y, (double)e.headingFiltDeg,
                          (unsigned)DEBUG_SETTLE_MS);
        } else {
            Serial.printf("[LTEST] CORNER_SETTLE corner=%d pos=(%.3f,%.3f) heading=%.1f settleMs=%u\n",
                          g_precision.segment + 1,
                          (double)e.x, (double)e.y, (double)e.headingFiltDeg,
                          (unsigned)DEBUG_SETTLE_MS);
        }
        return;
    }

    if (g_precision.phase == PrecisionPhase::SETTLE) {
        g_motor.stopImmediately();
        if ((now - g_precision.phaseStartedMs) >= DEBUG_SETTLE_MS) {
            const bool finalTurn = g_precision.segment >= g_precision.segmentCount - 1;
            beginPrecisionTurn(finalTurn);
        }
        return;
    }

    if (g_precision.phase == PrecisionPhase::TURN) {
        const bool finalTurn = g_precision.segment >= g_precision.segmentCount - 1;
        const float target = precisionTargetHeading();
        const float err = NavMath::wrapDeg180(target - e.headingFiltDeg);
        const float absErr = fabsf(err);
        if (absErr <= DEBUG_TURN_TOLERANCE_DEG) {
            g_motor.setLinearAngularSpeed(0, 0, true);
            if (g_precision.headingOkSinceMs == 0) {
                g_precision.headingOkSinceMs = now;
                Serial.printf("[PRECISION_TURN] target=%.1f heading=%.1f err=%+.1f "
                              "pwm=0 (in tolerance, starting stability hold)\n",
                              (double)target, (double)e.headingFiltDeg,
                              (double)err);
            }
            if ((now - g_precision.headingOkSinceMs) >= DEBUG_TURN_HOLD_MS) {
                Serial.printf("[PRECISION_TURN] %s target=%.1f heading=%.1f err=%+.1f "
                              "stableMs=%u\n",
                              (finalTurn ? "FINAL_TURN_STABLE"
                                         : (g_precision.square
                                                ? "CORNER_TURN_STABLE"
                                                : "CORNER_TURN_STABLE")),
                              (double)target, (double)e.headingFiltDeg,
                              (double)err,
                              (unsigned)(now - g_precision.headingOkSinceMs));
                // Переход в POST_TURN_SETTLE: motors=0, ждём ещё 300 мс
                // чтобы механика успокоилась перед стартом следующего
                // сегмента / поворота.
                g_precision.phase = PrecisionPhase::POST_TURN_SETTLE;
                g_precision.phaseStartedMs = now;
                g_motor.stopImmediately();
            }
            return;
        }
        g_precision.headingOkSinceMs = 0;
        // Ступенчатый PWM профиль по абсолютной ошибке угла. cmdL/cmdR
        // — это PWM в единицах мотора; в Motor::setLinearAngularSpeed()
        // они нормируются на 0..1 через PWM/ROVER_INPUT_DIV.
        int pwm;
        if (absErr > 60.0f)        pwm = PRECISION_TURN_PWM_MAX;
        else if (absErr > 25.0f)   pwm = PRECISION_TURN_PWM_MID;
        else if (absErr > 10.0f)   pwm = PRECISION_TURN_PWM_LOW;
        else                       pwm = PRECISION_TURN_PWM_MIN;
        // Преобразуем PWM в angular rad/s: используем тот же калибровочный
        // знаменатель, что и ROVER_V2_TURN_RADPS (≈1.5 рад/с при PWM=9).
        const float turnGain = ROVER_V2_TURN_RADPS / 9.0f;
        const float angular = (err >= 0.0f ? 1.0f : -1.0f) * turnGain * (float)pwm;
        g_motor.setLinearAngularSpeed(0, angular, true);
        // Лог не на каждом тике — только когда PWM-ступень меняется.
        if (pwm != g_precision.lastTurnPwm) {
            Serial.printf("[PRECISION_TURN] target=%.1f heading=%.1f err=%+.1f pwm=%d\n",
                          (double)target, (double)e.headingFiltDeg,
                          (double)err, pwm);
            g_precision.lastTurnPwm = pwm;
        }
    }

    if (g_precision.phase == PrecisionPhase::POST_TURN_SETTLE) {
        g_motor.stopImmediately();
        const uint32_t dwell = now - g_precision.phaseStartedMs;
        if (dwell == 0) {
            const char* tag = g_precision.square ? "SQTEST" : "LTEST";
            const int cornerIdx = g_precision.segment + 1;
            Serial.printf("[%s] POST_TURN_SETTLE corner=%d settleMs=%u\n",
                          tag, cornerIdx,
                          (unsigned)DEBUG_POST_TURN_SETTLE_MS);
        }
        if (dwell >= DEBUG_POST_TURN_SETTLE_MS) {
            const bool finalTurn = g_precision.segment >= g_precision.segmentCount - 1;
            if (finalTurn) {
                if (!g_precision.square) lShapeRecordWp1();
                g_precision.active = false;
                serialMotionEnd("arrived");
                return;
            }
            if (!g_precision.square && g_precision.segment == 0) {
                lShapeRecordWp1();
            }
            g_precision.segment++;
            g_precision.lastTurnPwm = 0;
            // Фиксируем ОПОРНУЮ точку нового сегмента ЗДЕСЬ, когда
            // ровер уже стоит. uploadPrecisionSegment ниже возьмёт её
            // вместо того, чтобы ждать первого PVT follower'а.
            g_precision.segmentStartX = e.x;
            g_precision.segmentStartY = e.y;
            if (!uploadPrecisionSegment(g_precision.segment)) {
                g_precision.active = false;
                serialMotionEnd("route_begin_failed");
            }
        }
        return;
    }
}

// LTEST DRIVE PATH TRACE — defined inside namespace roverdbg below.
// (g_precision / PrecisionPhase are file-local and only visible inside
// the namespace, so the body cannot live in global scope.)

static bool startPrecisionLShapeDebug() {
    g_precision.active = true;
    g_precision.square = false;
    g_precision.segmentCount = 2;
    g_precision.tag = "LTEST";
    g_precision.px[0] = g_ltest.p0x;
    g_precision.py[0] = g_ltest.p0y;
    g_precision.px[1] = g_ltest.p1x;
    g_precision.py[1] = g_ltest.p1y;
    g_precision.px[2] = g_ltest.p2x;
    g_precision.py[2] = g_ltest.p2y;
    g_precision.heading[0] = g_ltest.h0;
    g_precision.heading[1] = g_ltest.h1;
    g_precision.finalHeading = g_ltest.h1;
    serialMotionBegin(SerialMotionSource::GO_L_SHAPE, SERIAL_L_SHAPE_TIMEOUT_MS);
    if (!uploadPrecisionSegment(0)) {
        g_precision.active = false;
        serialMotionEnd("route_begin_failed");
        return false;
    }
    return true;
}

static bool startPrecisionSquareDebug(float sideM) {
    if (!goPrecheck()) return false;
    const auto& e0 = g_est.get();
    if (!e0.originSet) return false;
    const float h0 = NavMath::normalizeDeg360(
        g_heading.trusted ? g_heading.headingDeg : e0.headingFiltDeg);
    g_sqtest = SquareTestStats{};
    g_sqtest.active = true;
    g_sqtest.debug = true;
    g_sqtest.sidePlan = sideM;
    g_sqtest.h0 = h0;
    g_sqtest.startedMs = millis();
    g_sqtest.lastPrintMs = g_sqtest.startedMs;
    g_sqtest.startX = e0.x;
    g_sqtest.startY = e0.y;
    g_sqtest.headingStart = h0;
    g_sqtest.lastX = e0.x;
    g_sqtest.lastY = e0.y;
    g_sqtest.pX[0] = e0.x;
    g_sqtest.pY[0] = e0.y;

    float x = e0.x;
    float y = e0.y;
    for (int i = 0; i < 4; ++i) {
        const float h = NavMath::normalizeDeg360(h0 + 90.0f * i);
        const float hr = h * (float)M_PI / 180.0f;
        x += sideM * sinf(hr);
        y += sideM * cosf(hr);
        g_sqtest.pX[i + 1] = x;
        g_sqtest.pY[i + 1] = y;
    }

    Serial.printf("[SQTEST] START side=%.2f\n", (double)sideM);
    Serial.printf("[SQTEST] PLAN h0=%.1f p0=(%.3f,%.3f) p1=(%.3f,%.3f) p2=(%.3f,%.3f) p3=(%.3f,%.3f) p4=(%.3f,%.3f)\n",
                  (double)h0,
                  (double)g_sqtest.pX[0], (double)g_sqtest.pY[0],
                  (double)g_sqtest.pX[1], (double)g_sqtest.pY[1],
                  (double)g_sqtest.pX[2], (double)g_sqtest.pY[2],
                  (double)g_sqtest.pX[3], (double)g_sqtest.pY[3],
                  (double)g_sqtest.pX[4], (double)g_sqtest.pY[4]);

    setFollowerConfig(DEBUG_PRECISION_RADIUS_M, DEBUG_PRECISION_FINAL_RADIUS_M, DEBUG_PRECISION_SPEED_MPS);
    Serial.printf("[SQTEST] CONFIG arrivalRadius=%.2f finalRadius=%.2f speed=%.2f turnToleranceDeg=%.1f\n",
                  (double)g_arrivalRadiusM, (double)g_finalArrivalRadiusM,
                  (double)g_forwardSpeedMps, (double)DEBUG_TURN_TOLERANCE_DEG);

    g_precision = PrecisionDebugMotion{};
    g_precision.active = true;
    g_precision.square = true;
    g_precision.segmentCount = 4;
    g_precision.tag = "SQTEST";
    for (int i = 0; i < 5; ++i) {
        g_precision.px[i] = g_sqtest.pX[i];
        g_precision.py[i] = g_sqtest.pY[i];
    }
    for (int i = 0; i < 4; ++i) {
        g_precision.heading[i] = NavMath::normalizeDeg360(h0 + 90.0f * i);
    }
    g_precision.finalHeading = h0;
    serialMotionBegin(SerialMotionSource::GO_SQUARE, SERIAL_SQUARE_TIMEOUT_MS);
    if (!uploadPrecisionSegment(0)) {
        g_precision.active = false;
        serialMotionEnd("route_begin_failed");
        return false;
    }
    return true;
}

String handleGoSquareDebugLine(float sideM) {
    // Busy-guard: нельзя стартовать новый debug motion поверх активного.
    if (g_serialMotion.active) {
        Serial.printf("[BUSY] GO_SQUARE_DEBUG rejected: serialMotionSource=%s active\n",
                      serialMotionSourceName(g_serialMotion.source));
        return String("ERR,GO_SQUARE_DEBUG,busy_serial_motion");
    }
    if (!isfinite(sideM) || sideM <= 0.05f || sideM > 5.0f) {
        Serial.printf("[GO_SQUARE_DEBUG] refusing: bad side=%.2f\n", (double)sideM);
        return String("ERR,GO_SQUARE_DEBUG,bad_args");
    }
    if (!startPrecisionSquareDebug(sideM)) {
        g_sqtest.active = false;
        g_precision.active = false;
        resetFollowerConfig();
        return String("ERR,GO_SQUARE_DEBUG,refused");
    }
    return String("OK,GO_SQUARE_DEBUG");
}

void squareFinish(const char* reason) {
    if (!g_sqtest.active) return;
    g_sqtest.active = false;
    g_precision.active = false;
    g_sqtest.lastResultReason = reason ? reason : "unknown";
    const auto& e = g_est.get();
    g_sqtest.finishX = e.x;
    g_sqtest.finishY = e.y;
    g_sqtest.headingFinish = e.headingFiltDeg;
    g_sqtest.finishedMs = millis();
    const float finalErr = lshapeDist2d(g_sqtest.finishX, g_sqtest.finishY,
                                        g_sqtest.pX[4], g_sqtest.pY[4]);
    // closeLoop: расстояние от finish обратно к start. Для хорошего
    // замкнутого квадрата должно быть близко к 0.
    const float closeLoopErr = lshapeDist2d(g_sqtest.finishX, g_sqtest.finishY,
                                            g_sqtest.startX, g_sqtest.startY);
    const float headingErr = NavMath::wrapDeg180(g_sqtest.headingFinish - g_sqtest.h0);
    const uint32_t durationMs = g_sqtest.finishedMs - g_sqtest.startedMs;
    Serial.printf("[SQTEST] SUMMARY result=%s durationMs=%u side=%.3f finalErr=%.3f closeLoopErr=%.3f headingErr=%+.1f totalPath=%.3f maxCross=%.3f maxHeadingErr=%.1f\n",
                  reason ? reason : "unknown", (unsigned)durationMs,
                  (double)g_sqtest.sidePlan, (double)finalErr,
                  (double)closeLoopErr, (double)headingErr,
                  (double)g_sqtest.pathLen,
                  (double)g_sqtest.maxCross, (double)g_sqtest.maxHeadingErr);
    // Per-segment summary.
    for (int i = 0; i < 4; ++i) {
        const char* tag = (i == 0) ? "SEG1" : (i == 1) ? "SEG2" :
                         (i == 2) ? "SEG3" : "SEG4";
        const float chordErr = g_sqtest.segPathLen[i] - g_sqtest.sidePlan;
        Serial.printf("[SQTEST] SUMMARY_%s planned=%.3f path=%.3f chordErr=%+.3f maxCross=%.3f cornerErr=%.3f turnErr=%+.1f\n",
                      tag,
                      (double)g_sqtest.sidePlan,
                      (double)g_sqtest.segPathLen[i],
                      (double)chordErr,
                      (double)g_sqtest.segMaxCross[i],
                      (double)g_sqtest.cornerErr[i],
                      isnan(g_sqtest.segTurnErr[i]) ? 0.0f
                                                     : (double)g_sqtest.segTurnErr[i]);
    }
    Serial.printf("[SQTEST] SUMMARY closeLoopErr=%.3f finalHeadingErr=%+.1f\n",
                  (double)closeLoopErr, (double)headingErr);
    Serial.printf("[SQTEST_CSV] result=%s,side=%.3f,finalErr=%.3f,closeLoopErr=%.3f,headingErr=%+.1f,totalPath=%.3f,maxCross=%.3f,maxHeadingErr=%.1f,durationMs=%u,seg1Path=%.3f,seg2Path=%.3f,seg3Path=%.3f,seg4Path=%.3f,seg1Cross=%.3f,seg2Cross=%.3f,seg3Cross=%.3f,seg4Cross=%.3f\n",
                  reason ? reason : "unknown",
                  (double)g_sqtest.sidePlan,
                  (double)finalErr,
                  (double)closeLoopErr,
                  (double)headingErr,
                  (double)g_sqtest.pathLen,
                  (double)g_sqtest.maxCross,
                  (double)g_sqtest.maxHeadingErr,
                  (unsigned)durationMs,
                  (double)g_sqtest.segPathLen[0],
                  (double)g_sqtest.segPathLen[1],
                  (double)g_sqtest.segPathLen[2],
                  (double)g_sqtest.segPathLen[3],
                  (double)g_sqtest.segMaxCross[0],
                  (double)g_sqtest.segMaxCross[1],
                  (double)g_sqtest.segMaxCross[2],
                  (double)g_sqtest.segMaxCross[3]);
}

// GO_L_SHAPE_DEBUG,<first>,<turn>,<second>
// Same drive as GO_L_SHAPE but with full telemetry capture. Pops up a
// `LShapeTestStats` record, prints START/PLAN, periodic every 200 ms,
// WP1_REACHED on waypoint transition, and SUMMARY_* / CSV / VERDICT
// at the end (arrived / stopped / timeout / safety).
String handleGoLShapeDebugLine(float firstM, float turnDeg, float secondM) {
    // Parse log fires before validation so the operator can see what
    // the dispatcher actually picked up.
    Serial.printf("[GO_L_SHAPE_DEBUG] parsed first=%.2f turn=%.1f second=%.2f\n",
                  (double)firstM, (double)turnDeg, (double)secondM);
    // Busy-guard: нельзя стартовать новый debug motion поверх активного.
    if (g_serialMotion.active) {
        Serial.printf("[BUSY] GO_L_SHAPE_DEBUG rejected: serialMotionSource=%s active\n",
                      serialMotionSourceName(g_serialMotion.source));
        return String("ERR,GO_L_SHAPE_DEBUG,busy_serial_motion");
    }
    // Validate args before touching g_ltest, route or serialMotion.
    if (!isfinite(firstM) || firstM <= 0.05f || firstM > 5.0f ||
        !isfinite(secondM) || secondM <= 0.05f || secondM > 5.0f ||
        !isfinite(turnDeg) || fabsf(turnDeg) < 5.0f || fabsf(turnDeg) > 180.0f) {
        Serial.printf("[GO_L_SHAPE_DEBUG] refusing: bad args first=%.2f turn=%.1f second=%.2f\n",
                      (double)firstM, (double)turnDeg, (double)secondM);
        return String("ERR,GO_L_SHAPE_DEBUG,bad_args");
    }

    // Reset stats, then delegate to handleGoLShape after capturing
    // the planner's plan output.
    g_ltest = LShapeTestStats{};
    g_ltest.debug = true;
    g_ltest.firstPlan  = firstM;
    g_ltest.turnPlan   = turnDeg;
    g_ltest.secondPlan = secondM;
    g_ltest.startedMs  = millis();
    g_ltest.lastPrintMs= g_ltest.startedMs;

    const auto& e0 = g_est.get();
    g_ltest.startX = e0.x;
    g_ltest.startY = e0.y;
    g_ltest.headingStart = NavMath::normalizeDeg360(
        g_heading.trusted ? g_heading.headingDeg : e0.headingFiltDeg);
    g_ltest.h0 = g_ltest.headingStart;
    g_ltest.h1 = NavMath::normalizeDeg360(g_ltest.h0 + turnDeg);
    const float h0R = g_ltest.h0 * (float)M_PI / 180.0f;
    g_ltest.p0x = e0.x;
    g_ltest.p0y = e0.y;
    g_ltest.p1x = g_ltest.p0x + firstM  * sinf(h0R);
    g_ltest.p1y = g_ltest.p0y + firstM  * cosf(h0R);
    const float h1R = g_ltest.h1 * (float)M_PI / 180.0f;
    g_ltest.p2x = g_ltest.p1x + secondM * sinf(h1R);
    g_ltest.p2y = g_ltest.p1y + secondM * cosf(h1R);
    g_ltest.lastX = e0.x;
    g_ltest.lastY = e0.y;
    g_ltest.lastWpIndex = -1;

    Serial.printf("[LTEST] START first=%.2f turn=%.1f second=%.2f\n",
                  (double)firstM, (double)turnDeg, (double)secondM);
    Serial.printf("[LTEST] PLAN h0=%.1f h1=%.1f p0=(%.3f,%.3f) p1=(%.3f,%.3f) p2=(%.3f,%.3f)\n",
                  (double)g_ltest.h0, (double)g_ltest.h1,
                  (double)g_ltest.p0x, (double)g_ltest.p0y,
                  (double)g_ltest.p1x, (double)g_ltest.p1y,
                  (double)g_ltest.p2x, (double)g_ltest.p2y);

    // Tight precision mode for the duration of this test. Restored by
    // serialMotionEnd (which calls resetFollowerConfig) so the override
    // does not leak into the next regular route.
    setFollowerConfig(DEBUG_PRECISION_RADIUS_M, DEBUG_PRECISION_FINAL_RADIUS_M, DEBUG_PRECISION_SPEED_MPS);
    Serial.printf("[LTEST] CONFIG arrivalRadius=%.2f finalRadius=%.2f speed=%.2f turnToleranceDeg=%.1f\n",
                  (double)g_arrivalRadiusM, (double)g_finalArrivalRadiusM,
                  (double)g_forwardSpeedMps, (double)DEBUG_TURN_TOLERANCE_DEG);

    if (!startPrecisionLShapeDebug()) {
        g_ltest.active = false;
        g_ltest.lastResultReason = "go_l_shape_refused";
        g_precision.active = false;
        resetFollowerConfig();
        return String("ERR,GO_L_SHAPE_DEBUG,go_l_shape_refused");
    }
    g_ltest.active = true;
    return String("OK,GO_L_SHAPE_DEBUG");
}

void lShapeFinish(const char* reason) {
    if (!g_ltest.active) return;
    g_ltest.active = false;
    g_ltest.lastResultReason = reason ? reason : "unknown";
    const auto& eE = g_est.get();
    g_ltest.finishX = eE.x;
    g_ltest.finishY = eE.y;
    g_ltest.headingFinish = eE.headingFiltDeg;
    g_ltest.finishedMs = millis();

    const bool hasWp1 = !isnan(g_ltest.wp1X) && !isnan(g_ltest.wp1Y);
    const float ax = g_ltest.startX;
    const float ay = g_ltest.startY;
    const float wx = hasWp1 ? g_ltest.wp1X : g_ltest.p1x;
    const float wy = hasWp1 ? g_ltest.wp1Y : g_ltest.p1y;
    const float fx = g_ltest.finishX;
    const float fy = g_ltest.finishY;
    const float p2x = g_ltest.p2x;
    const float p2y = g_ltest.p2y;

    const float seg1Chord = lshapeDist2d(ax, ay, wx, wy);
    const float seg2Chord = lshapeDist2d(wx, wy, fx, fy);
    const float finalError = lshapeDist2d(fx, fy, p2x, p2y);
    const float seg1Err = seg1Chord - g_ltest.firstPlan;
    const float seg2Err = seg2Chord - g_ltest.secondPlan;
    const float turnActual = NavMath::wrapDeg180(g_ltest.headingFinish - g_ltest.headingStart);
    const float turnError  = NavMath::wrapDeg180(turnActual - g_ltest.turnPlan);
    const float totalPath  = g_ltest.pathLen;
    const float maxCrossAll = (g_ltest.maxCross1 > g_ltest.maxCross2)
                                ? g_ltest.maxCross1 : g_ltest.maxCross2;
    const uint32_t durationMs = g_ltest.finishedMs - g_ltest.startedMs;

    // Verdict (tightened thresholds: this is a precision test, not a
    // casual drive). The follower still uses the global arrival radius,
    // so the chord values carry the underlying early-stop imprecision.
    // We acknowledge that with the relaxed chord window 0.85..1.15.
    const bool wp1OK = hasWp1;
    const bool chord1OK = seg1Chord >= 0.85f && seg1Chord <= 1.15f;
    const bool chord2OK = seg2Chord >= 0.85f && seg2Chord <= 1.15f;
    const bool finalOK = finalError <= 0.20f;
    const bool turnOK  = fabsf(turnError) <= 12.0f;
    const bool crossOK = maxCrossAll <= 0.25f;
    const bool okAll = wp1OK && chord1OK && chord2OK && finalOK && turnOK && crossOK &&
                     String(reason) != "timeout";
    const bool warnAll = wp1OK &&
        (finalError <= 0.35f) && (fabsf(turnError) <= 20.0f) &&
        (maxCrossAll <= 0.40f) &&
        String(reason) != "timeout";
    const char* verdict = "BAD";
    const char* vReason = "thresholds";
    if (okAll)            { verdict = "OK";  vReason = "-"; }
    else if (warnAll)     { verdict = "WARN"; vReason = reason; }
    else if (!wp1OK)      { vReason = "waypoint_missed"; }
    else if (String(reason) == "timeout") { vReason = "timeout"; }
    else { vReason = reason; }

    Serial.printf("[LTEST] SUMMARY result=%s durationMs=%u\n",
                  reason ? reason : "unknown", (unsigned)durationMs);
    Serial.printf("[LTEST] SUMMARY_CONFIG arrivalRadius=%.2f finalRadius=%.2f speed=%.2f\n",
                  (double)g_arrivalRadiusM, (double)g_finalArrivalRadiusM,
                  (double)g_forwardSpeedMps);
    Serial.printf("[LTEST] SUMMARY_PLAN first=%.3f turn=%.1f second=%.3f total=%.3f h0=%.1f h1=%.1f\n",
                  (double)g_ltest.firstPlan, (double)g_ltest.turnPlan,
                  (double)g_ltest.secondPlan,
                  (double)(g_ltest.firstPlan + g_ltest.secondPlan),
                  (double)g_ltest.h0, (double)g_ltest.h1);
    Serial.printf("[LTEST] SUMMARY_POINTS p0=(%.3f,%.3f) p1=(%.3f,%.3f) p2=(%.3f,%.3f)\n",
                  (double)g_ltest.p0x, (double)g_ltest.p0y,
                  (double)g_ltest.p1x, (double)g_ltest.p1y,
                  (double)g_ltest.p2x, (double)g_ltest.p2y);
    Serial.printf("[LTEST] SUMMARY_ACTUAL start=(%.3f,%.3f) wp1=(%.3f,%.3f) finish=(%.3f,%.3f)\n",
                  (double)ax, (double)ay,
                  (double)wx, (double)wy,
                  (double)fx, (double)fy);
    Serial.printf("[LTEST] SUMMARY_SEG1 planned=%.3f chord=%.3f path=%.3f errChord=%+.3f maxCross=%.3f\n",
                  (double)g_ltest.firstPlan,
                  (double)seg1Chord,
                  (double)g_ltest.pathLen1,
                  (double)seg1Err,
                  (double)g_ltest.maxCross1);
    Serial.printf("[LTEST] SUMMARY_SEG2 planned=%.3f chord=%.3f path=%.3f errChord=%+.3f maxCross=%.3f finalErr=%.3f\n",
                  (double)g_ltest.secondPlan,
                  (double)seg2Chord,
                  (double)g_ltest.pathLen2,
                  (double)seg2Err,
                  (double)g_ltest.maxCross2,
                  (double)finalError);
    Serial.printf("[LTEST] SUMMARY_TURN planned=%.1f actual=%+.1f err=%+.1f "
                  "headingStart=%.1f headingWp1=%.1f headingFinish=%.1f maxHeadingErr=%.1f\n",
                  (double)g_ltest.turnPlan,
                  (double)turnActual, (double)turnError,
                  (double)g_ltest.headingStart,
                  (double)hasWp1 ? (double)g_ltest.headingWp1 : 0.0,
                  (double)g_ltest.headingFinish,
                  (double)g_ltest.maxHeadingErr);
    Serial.printf("[LTEST] SUMMARY_QUALITY totalPath=%.3f maxCross=%.3f maxHeadingErr=%.1f "
                  "maxHAcc=%.3f maxPvtAge=%u maxRtcmAge=%u maxHeadingAge=%u "
                  "maxRelYawAge=%u maxGyroAge=%u\n",
                  (double)totalPath, (double)maxCrossAll,
                  (double)g_ltest.maxHeadingErr,
                  (double)g_ltest.maxHAcc,
                  (unsigned)g_ltest.maxPvtAge,
                  (unsigned)g_ltest.maxRtcmAge,
                  (unsigned)g_ltest.maxHeadingAge,
                  (unsigned)g_ltest.maxRelYawAge,
                  (unsigned)g_ltest.maxGyroAge);
    Serial.printf("[LTEST_CSV] result=%s,arrivalRadius=%.2f,finalRadius=%.2f,followSpeed=%.2f,"
                  "firstPlan=%.3f,firstChord=%.3f,firstPath=%.3f,"
                  "firstErr=%+.3f,secondPlan=%.3f,secondChord=%.3f,secondPath=%.3f,"
                  "secondErr=%+.3f,turnPlan=%.1f,turnActual=%+.1f,turnErr=%+.1f,"
                  "finalErr=%.3f,totalPath=%.3f,maxCross1=%.3f,maxCross2=%.3f,"
                  "maxCross=%.3f,maxHeadingErr=%.1f,durationMs=%u,"
                  "maxHAcc=%.3f,maxPvtAge=%u,maxRtcmAge=%u,maxHeadingAge=%u,"
                  "maxRelYawAge=%u,maxGyroAge=%u\n",
                  reason ? reason : "unknown",
                  (double)g_arrivalRadiusM, (double)g_finalArrivalRadiusM,
                  (double)g_forwardSpeedMps,
                  (double)g_ltest.firstPlan, (double)seg1Chord, (double)g_ltest.pathLen1, (double)seg1Err,
                  (double)g_ltest.secondPlan, (double)seg2Chord, (double)g_ltest.pathLen2, (double)seg2Err,
                  (double)g_ltest.turnPlan, (double)turnActual, (double)turnError,
                  (double)finalError,
                  (double)totalPath,
                  (double)g_ltest.maxCross1, (double)g_ltest.maxCross2,
                  (double)maxCrossAll,
                  (double)g_ltest.maxHeadingErr,
                  (unsigned)durationMs,
                  (double)g_ltest.maxHAcc,
                  (unsigned)g_ltest.maxPvtAge,
                  (unsigned)g_ltest.maxRtcmAge,
                  (unsigned)g_ltest.maxHeadingAge,
                  (unsigned)g_ltest.maxRelYawAge,
                  (unsigned)g_ltest.maxGyroAge);
    Serial.printf("[LTEST] VERDICT %s reason=%s finalError=%.3f turnError=%+.1f maxCross=%.3f wp1=%d reason=%s\n",
                  verdict, vReason,
                  (double)finalError, (double)turnError,
                  (double)maxCrossAll,
                  hasWp1 ? 1 : 0,
                  reason ? reason : "unknown");
}

// Per-tick update — call from loop() while g_ltest.active.
void lShapeOnTick() {
    if (!g_ltest.active) return;
    const auto& e = g_est.get();
    const uint32_t now = millis();
    const float x = e.x;
    const float y = e.y;

    // Logical-segment source of truth. Когда активен precision-flow,
    // каждый сегмент загружается отдельным одно-waypoint-маршрутом, и
    // g_follow.wpIdx всегда 0. Поэтому раньше WP1_REACHED никогда не
    // срабатывал (ждали перехода 0->1) — отсюда баг с wp1Err=0.865.
    //
    // Источник истины теперь — g_precision.segment:
    //   - 0 => logical segment p0->p1 (сегмент 1 в LTEST-нотации)
    //   - 1 => logical segment p1->p2 (сегмент 2 в LTEST-нотации)
    // На время settle/turn-фаз робот стоит, метрики не должны
    // скакать. Логируем только пока реально DRIVE.
    int segLogical = 0;          // 0 или 1 — logical precision segment
    bool drivingNow = false;
    if (g_precision.active && !g_precision.square) {
        segLogical = (g_precision.segment >= 1) ? 1 : 0;
        drivingNow = (g_precision.phase == PrecisionPhase::DRIVE);
    } else {
        // Legacy path: g_ltest был запущен без precision (если что-то
        // так пользуется). Fallback на старое правило wpIdx<=0.
        segLogical = (g_follow.wpIdx <= 0) ? 0 : 1;
        drivingNow = true;
    }

    // Path length accumulation. Reject > 0.5 m single-step jumps as
    // RTK jitter rather than real motion.
    if (drivingNow && !isnan(g_ltest.lastX) && !isnan(g_ltest.lastY)) {
        const float step = lshapeDist2d(g_ltest.lastX, g_ltest.lastY, x, y);
        if (step < 0.5f) {
            g_ltest.pathLen += step;
            if (segLogical == 0) g_ltest.pathLen1 += step;
            else                  g_ltest.pathLen2 += step;
        }
    }
    g_ltest.lastX = x;
    g_ltest.lastY = y;

    // Cross-track maxima per logical segment.
    if (drivingNow) {
        if (segLogical == 0) {
            const float cross = fabsf(lshapeSignedCross(
                g_ltest.p0x, g_ltest.p0y, g_ltest.p1x, g_ltest.p1y, x, y));
            if (cross > g_ltest.maxCross1) g_ltest.maxCross1 = cross;
        } else {
            const float cross = fabsf(lshapeSignedCross(
                g_ltest.p1x, g_ltest.p1y, g_ltest.p2x, g_ltest.p2y, x, y));
            if (cross > g_ltest.maxCross2) g_ltest.maxCross2 = cross;
        }
    }

    // Heading error: target heading from current (x,y) к текущей цели
    // сегмента (p1 для seg 0, p2 для seg 1), vs live heading.
    if (drivingNow) {
        const float tx = (segLogical == 0) ? g_ltest.p1x : g_ltest.p2x;
        const float ty = (segLogical == 0) ? g_ltest.p1y : g_ltest.p2y;
        const float targetH = NavMath::targetHeadingDeg(tx - x, ty - y);
        const float hErr = fabsf(NavMath::wrapDeg180(targetH - e.headingFiltDeg));
        if (hErr > g_ltest.maxHeadingErr) g_ltest.maxHeadingErr = hErr;
    }

    // RTK / IMU maxima.
    if (e.hAcc > g_ltest.maxHAcc)      g_ltest.maxHAcc    = e.hAcc;
    if (e.pvtAgeMs > g_ltest.maxPvtAge) g_ltest.maxPvtAge = e.pvtAgeMs;
    {
        const uint32_t ra = g_rtcm.transportAgeMs(now);
        if (ra > g_ltest.maxRtcmAge) g_ltest.maxRtcmAge = ra;
    }
    if (e.headingAgeMs > g_ltest.maxHeadingAge) g_ltest.maxHeadingAge = e.headingAgeMs;
    {
        const uint32_t ra = g_imu.relYawAgeMs(now);
        if (ra > g_ltest.maxRelYawAge) g_ltest.maxRelYawAge = ra;
        const uint32_t ga = g_imu.gyroAgeMs(now);
        if (ga > g_ltest.maxGyroAge)   g_ltest.maxGyroAge   = ga;
    }

    // NOTE: WP1_REACHED ловится НЕ тут (g_follow.wpIdx всегда 0 в
    // precision-flow), а в finishPrecisionTurn() при переходе segment
    // 0 -> 1. См. lShapeRecordWp1().

    g_ltest.lastWpIndex = g_follow.wpIdx;

    // Optional periodic debug line.
    if (g_ltest.debug && (now - g_ltest.lastPrintMs) >= 200u) {
        g_ltest.lastPrintMs = now;
        const float tx = (segLogical == 0) ? g_ltest.p1x : g_ltest.p2x;
        const float ty = (segLogical == 0) ? g_ltest.p1y : g_ltest.p2y;
        const float distT = lshapeDist2d(x, y, tx, ty);
        const float targetH = NavMath::targetHeadingDeg(tx - x, ty - y);
        const float hErr = NavMath::wrapDeg180(targetH - e.headingFiltDeg);
        const float spd  = g_follow.linearMps;
        const float crossNow = (segLogical == 0)
            ? lshapeSignedCross(g_ltest.p0x, g_ltest.p0y,
                                g_ltest.p1x, g_ltest.p1y, x, y)
            : lshapeSignedCross(g_ltest.p1x, g_ltest.p1y,
                                g_ltest.p2x, g_ltest.p2y, x, y);
        Serial.printf("[LTEST] T t=%u logicalSeg=%d driving=%d pos=(%.3f,%.3f) "
                      "target=(%.3f,%.3f) dist=%.3f cross=%.3f heading=%.1f "
                      "hErr=%+.1f speed=%.2f cmdL=%d cmdR=%d safety=%d reason=%s "
                      "hAcc=%.3f pvtAge=%u rtcmAge=%u headingAge=%u "
                      "relAge=%u gyroAge=%u\n",
                      (unsigned)(now - g_ltest.startedMs),
                      segLogical, (int)drivingNow,
                      (double)x, (double)y,
                      (double)tx, (double)ty,
                      (double)distT, (double)crossNow,
                      (double)e.headingFiltDeg,
                      (double)hErr, (double)spd,
                      g_motor.currentLeftPwm(), g_motor.currentRightPwm(),
                      (int)g_safety.level(),
                      g_safety.reason() ? g_safety.reason() : "-",
                      (double)e.hAcc,
                      (unsigned)e.pvtAgeMs,
                      (unsigned)g_rtcm.transportAgeMs(now),
                      (unsigned)e.headingAgeMs,
                      (unsigned)g_imu.relYawAgeMs(now),
                      (unsigned)g_imu.gyroAgeMs(now));
    }
}

// Запись фактической WP1-позиции и лог [LTEST] WP1_REACHED.
// Вызывается ИСКЛЮЧИТЕЛЬНО из finishPrecisionTurn() когда
// g_precision.segment инкрементируется 0 -> 1 (завершение logical
// segment p0->p1). Так WP1_REACHED срабатывает ровно один раз и
// именно в точке прибытия на p1, а не на финальной точке p2.
static void lShapeRecordWp1() {
    if (!g_ltest.active) return;
    if (!isnan(g_ltest.wp1X)) return; // уже зафиксирована
    const auto& e = g_est.get();
    const uint32_t now = millis();
    g_ltest.wp1X = e.x;
    g_ltest.wp1Y = e.y;
    g_ltest.headingWp1 = e.headingFiltDeg;
    g_ltest.wp1Ms = now;
    const float wx = g_ltest.wp1X, wy = g_ltest.wp1Y;
    const float p1x = g_ltest.p1x, p1y = g_ltest.p1y;
    const float wp1Err = lshapeDist2d(wx, wy, p1x, p1y);
    const float firstChord = lshapeDist2d(g_ltest.startX, g_ltest.startY, wx, wy);
    const float crossAt = lshapeSignedCross(
        g_ltest.p0x, g_ltest.p0y, g_ltest.p1x, g_ltest.p1y, wx, wy);
    Serial.printf("[LTEST] WP1_REACHED actual=(%.3f,%.3f) planned=(%.3f,%.3f) "
                  "wp1Err=%.3f firstChord=%.3f firstPath=%.3f cross=%+.3f "
                  "heading=%.1f\n",
                  (double)wx, (double)wy, (double)p1x, (double)p1y,
                  (double)wp1Err, (double)firstChord,
                  (double)g_ltest.pathLen1, (double)crossAt,
                  (double)g_ltest.headingWp1);
}

void setLogEnabled(bool enabled) {
    g_logEnabled = enabled;
    Serial.printf("[LOG] periodic log %s\n", enabled ? "ON (200ms)" : "OFF");
}

bool handleGoLShapeDebug(float firstM, float turnDeg, float secondM) {
    return handleGoLShapeDebugLine(firstM, turnDeg, secondM).startsWith("OK");
}

String diagLine() {
    const auto& e = g_est.get();
    uint32_t now = millis();
    char buf[320];
    snprintf(buf, sizeof(buf),
             "sol=%d hAcc=%.3f imu=%.1f mag=%d acc=%.2f imuAge=%u "
             "pvtAge=%u rtcmAge=%lu motFB=%d ws=%d safety=%d %s "
             "cmdL=%d cmdR=%d sp=%d st=%d "
             "wp=%d/%d dWp=%.2f hErr=%.1f ct=%.2f fault=%s",
             (int)e.sol, e.hAcc, g_imu.yawDeg(),
             g_imu.yawFromMag() ? 1 : 0, g_imu.yawAccRad(),
             g_imu.ageMs(now), e.pvtAgeMs,
             g_rtcm.transportAgeMs(now),
             g_motor.haveFeedback() ? 1 : 0,
             g_ws.isConnected() ? 1 : 0,
             (int)g_safety.level(), g_safety.reason(),
             g_motor.currentLeftPwm(), g_motor.currentRightPwm(),
             g_motor.lastSpeedCmd(), g_motor.lastSteerCmd(),
             g_follow.wpIdx, g_route.count(),
             g_follow.distToTarget, g_follow.headingErr, g_follow.crossTrack,
             g_follow.faultReason ? g_follow.faultReason : "-");
    return String(buf);
}

String imuZeroLine() {
    currentMagPairs(s0Mxy, s0Myx, s0Mxz, s0Mzx, s0Myz, s0Mzy);
    s0Yaw = g_imu.yawDeg();
    s0Raw = g_imu.rawYawDeg();
    s0Game = g_imu.gameYawDeg();
    s0Rot = g_imu.rotYawDeg();
    s0Geo = g_imu.geoYawDeg();
    s0Gx = g_imu.gyroIntXDeg();
    s0Gy = g_imu.gyroIntYDeg();
    s0Gz = g_imu.gyroIntZDeg();
    s0CntMag = g_imu.magCount();
    s0CntGeo = g_imu.geoCount();
    s0CntRot = g_imu.rotCount();
    s0CntGame = g_imu.gameCount();
    s0CntGyro = g_imu.gyroCount();
    s_imuZeroed = true;
    char buf[420];
    snprintf(buf, sizeof(buf),
             "IMU_ZERO,state=%d,source=%d,abs=%d,yaw=%.1f,raw=%.1f,game=%.1f,rot=%.1f,geo=%.1f,"
             "mag=%.2f/%.2f/%.2f,gint=%.1f/%.1f/%.1f,cnt=%lu/%lu/%lu/%lu/%lu",
             (int)g_imu.headingState(), (int)g_imu.yawSource(), g_imu.yawAbsoluteValid() ? 1 : 0,
             s0Yaw, s0Raw, s0Game, s0Rot, s0Geo,
             g_imu.magX(), g_imu.magY(), g_imu.magZ(), s0Gx, s0Gy, s0Gz,
             (unsigned long)s0CntMag, (unsigned long)s0CntGeo,
             (unsigned long)s0CntRot, (unsigned long)s0CntGame,
             (unsigned long)s0CntGyro);
    return String(buf);
}

String imuDiagLine() {
    if (!s_imuZeroed) {
        return String("IMU_DIAG,ERR,call IMU_ZERO first");
    }
    float mxy, myx, mxz, mzx, myz, mzy;
    currentMagPairs(mxy, myx, mxz, mzx, myz, mzy);
    char buf[760];
    snprintf(buf, sizeof(buf),
             "IMU_DIAG,"
             "state=%d,source=%d,abs=%d,dYaw=%.1f,dRaw=%.1f,dGame=%.1f,dRot=%.1f,dGeo=%.1f,"
             "dGx=%.1f,dGy=%.1f,dGz=%.1f,"
             "dMxy=%.1f,dMyx=%.1f,dMxz=%.1f,dMzx=%.1f,dMyz=%.1f,dMzy=%.1f,"
             "rate=%.1f/%.1f/%.1f,"
             "age=%u,cnt=%lu/%lu/%lu/%lu/%lu,"
             "nowYaw=%.1f,raw=%.1f,game=%.1f,rot=%.1f,geo=%.1f,mag=%.2f/%.2f/%.2f",
             (int)g_imu.headingState(), (int)g_imu.yawSource(), g_imu.yawAbsoluteValid() ? 1 : 0,
             wrapDeg180Local(g_imu.yawDeg() - s0Yaw),
             wrapDeg180Local(g_imu.rawYawDeg() - s0Raw),
             wrapDeg180Local(g_imu.gameYawDeg() - s0Game),
             wrapDeg180Local(g_imu.rotYawDeg() - s0Rot),
             wrapDeg180Local(g_imu.geoYawDeg() - s0Geo),
             g_imu.gyroIntXDeg() - s0Gx,
             g_imu.gyroIntYDeg() - s0Gy,
             g_imu.gyroIntZDeg() - s0Gz,
             wrapDeg180Local(mxy - s0Mxy),
             wrapDeg180Local(myx - s0Myx),
             wrapDeg180Local(mxz - s0Mxz),
             wrapDeg180Local(mzx - s0Mzx),
             wrapDeg180Local(myz - s0Myz),
             wrapDeg180Local(mzy - s0Mzy),
             g_imu.gyroXDps(), g_imu.gyroYDps(), g_imu.gyroZDps(),
             (unsigned)g_imu.ageMs(millis()),
             (unsigned long)(g_imu.magCount() - s0CntMag),
             (unsigned long)(g_imu.geoCount() - s0CntGeo),
             (unsigned long)(g_imu.rotCount() - s0CntRot),
             (unsigned long)(g_imu.gameCount() - s0CntGame),
             (unsigned long)(g_imu.gyroCount() - s0CntGyro),
             g_imu.yawDeg(), g_imu.rawYawDeg(), g_imu.gameYawDeg(),
             g_imu.rotYawDeg(), g_imu.geoYawDeg(),
             g_imu.magX(), g_imu.magY(), g_imu.magZ());
    return String(buf);
}

String imuStatusLine() {
    char buf[520];
    snprintf(buf, sizeof(buf),
             "IMU_STATUS,state=%d,stateName=%s,source=%d,sourceName=%s,abs=%d,disturbed=%d,acc=%.2f,"
             "yaw=%.1f,raw=%.1f,sourceYaw=%.1f,absYaw=%.1f,magNorm=%.2f,mag=%.2f/%.2f/%.2f,"
             "stability=%.1f,age=%u,headingAge=%u,gyroRate=%.1f,estUsed=%d",
             (int)g_imu.headingState(), ImuMath::headingStateName(g_imu.headingState()),
             (int)g_imu.yawSource(), ImuMath::yawSourceName(g_imu.yawSource()),
             g_imu.yawAbsoluteValid() ? 1 : 0, g_imu.magDisturbed() ? 1 : 0, g_imu.yawAccRad(),
             g_imu.yawDeg(), g_imu.rawYawDeg(), g_imu.sourceYawDeg(), g_imu.absYawDeg(),
             g_imu.magNorm(), g_imu.magX(), g_imu.magY(), g_imu.magZ(),
             g_imu.startupAbsDeltaDeg(), (unsigned)g_imu.ageMs(millis()),
             (unsigned)g_imu.yawAgeMs(millis()), g_imu.yawRateDps(),
             g_est.get().headingUsedByEstimator ? 1 : 0);
    return String(buf);
}

String imuCalStartLine() {
    String out;
    String err;
    // handleCalStart() does the soft gate (state/age), then drives the
    // async IMU_CAL_START. startCalibration() itself is non-blocking with
    // a 1500 ms hard cap, so this call returns in well under 2 seconds
    // even if sh2_startCal() hangs inside the BNO08x lib.
    if (handleCalStart()) {
        out = "IMU_CAL_START,OK";
    } else {
        // surface the real reason: timeout / busy / sh2 rc / not initialised
        switch (g_imu.lastCalStartState()) {
            case Imu::CalStartState::TIMEOUT:
                out = "IMU_CAL_START,ERR,timeout";
                break;
            case Imu::CalStartState::RUNNING:
                out = "IMU_CAL_START,ERR,calibration_already_running";
                break;
            case Imu::CalStartState::ERROR:
                out = "IMU_CAL_START,ERR,sh2_startCal_failed_rc=" + String(g_imu.lastCalStartRc());
                break;
            case Imu::CalStartState::IDLE:
            case Imu::CalStartState::OK:
            default:
                // Rejected at the gate (handleCal soft-gate). Fall back to
                // the descriptive reason the gate would have printed, but
                // in machine-readable form.
                out = "IMU_CAL_START,ERR,gate_refused";
                break;
        }
    }
    Serial.println(out);
    return out;
}

String imuCalSaveLine() {
    String out;
    if (handleCalSave()) {
        out = "IMU_CAL_SAVE,OK";
    } else {
        out = "IMU_CAL_SAVE,ERR,save_failed";
    }
    Serial.println(out);
    return out;
}

String imuCalStatusLine() {
    uint32_t now = millis();
    const char* calStateName = "idle";
    switch (g_imu.lastCalStartState()) {
        case Imu::CalStartState::IDLE:    calStateName = "idle"; break;
        case Imu::CalStartState::RUNNING: calStateName = "running"; break;
        case Imu::CalStartState::OK:      calStateName = "ok"; break;
        case Imu::CalStartState::ERROR:   calStateName = "error"; break;
        case Imu::CalStartState::TIMEOUT: calStateName = "timeout"; break;
    }
    char buf[384];
    snprintf(buf, sizeof(buf),
             "IMU_CAL_STATUS,calStartState=%s,calStartMs=%lu,calStartRc=%d,"
             "headingState=%s,yawSource=%s,abs=%d,acc=%.2f,"
             "magNorm=%.2f,magX=%.2f,magY=%.2f,magZ=%.2f,"
             "disturbed=%d,ageMs=%u,yawAgeMs=%u",
             calStateName,
             (unsigned long)g_imu.lastCalStartMs(),
             g_imu.lastCalStartRc(),
             ImuMath::headingStateName(g_imu.headingState()),
             ImuMath::yawSourceName(g_imu.yawSource()),
             g_imu.yawAbsoluteValid() ? 1 : 0,
             (double)g_imu.yawAccRad(),
             (double)g_imu.magNorm(),
             (double)g_imu.magX(), (double)g_imu.magY(), (double)g_imu.magZ(),
             g_imu.magDisturbed() ? 1 : 0,
             (unsigned)g_imu.ageMs(now),
             (unsigned)g_imu.yawAgeMs(now));
    Serial.println(buf);
    return String(buf);
}

String imuCalClearLine() {
    String err;
    if (g_imu.clearCalibration(&err)) {
        return String("IMU_CAL_CLEAR,OK");
    }
    return String("IMU_CAL_CLEAR,ERR,") + err;
}

String imuTareYawLine() {
    String err;
    if (g_imu.tareYaw(false, &err)) {
        return String("IMU_TARE_YAW,OK");
    }
    return String("IMU_TARE_YAW,ERR,") + err;
}

String imuTarePersistLine() {
    String err;
    if (g_imu.tareYaw(true, &err)) {
        return String("IMU_TARE_PERSIST,OK");
    }
    return String("IMU_TARE_PERSIST,ERR,") + err;
}

// IMU_SET_TRUE_HEADING,<deg>: set user heading correction so the current
// IMU robot heading matches the supplied true compass heading. Persistent
// across reboots.
String imuSetTrueHeadingLine(float trueHeadingDeg) {
    if (!isfinite(trueHeadingDeg)) {
        return String("IMU_SET_TRUE_HEADING,ERR,nan");
    }
    const float current = g_imu.yawDeg();
    const float base = g_imu.baseHeadingDeg();
    const float oldCorr = g_imu.userHeadingCorrectionDeg();
    const float delta = ImuMath::wrapDeg180(trueHeadingDeg - current);
    const float newCorr = ImuMath::wrapDeg180(oldCorr + delta);
    String err;
    if (!g_imu.applyAndSaveUserCorrection(newCorr, &err)) {
        return String("IMU_SET_TRUE_HEADING,ERR,") + err;
    }
    // If the absolute yaw is valid (not game-only), reseed estimator so
    // the filter immediately agrees with the corrected heading.
    if (g_imu.yawIsAbsolute() && g_imu.headingState() == ImuHeadingState::IMU_ABSOLUTE_OK) {
        g_est.seedHeadingDeg(g_imu.yawDeg(), g_imu.yawSource());
    }
    const float finalAfter = g_imu.yawDeg();
    char buf[256];
    snprintf(buf, sizeof(buf),
             "[IMU-CAL-HEADING] current=%.1f true=%.1f delta=%.1f base=%.1f userCorrection=%.1f final=%.1f saved=1",
             (double)current, (double)trueHeadingDeg, (double)delta,
             (double)base, (double)g_imu.userHeadingCorrectionDeg(),
             (double)finalAfter);
    Serial.println(buf);
    char out[200];
    snprintf(out, sizeof(out),
             "IMU_SET_TRUE_HEADING,OK,current=%.1f,true=%.1f,delta=%.1f,userCorrection=%.1f,final=%.1f",
             (double)current, (double)trueHeadingDeg, (double)delta,
             (double)g_imu.userHeadingCorrectionDeg(),
             (double)finalAfter);
    return String(out);
}

// IMU_CLEAR_HEADING_CORRECTION: zero the user correction (and persist).
String imuClearHeadingCorrectionLine() {
    String err;
    if (!g_imu.clearUserHeadingCorrection(&err)) {
        return String("IMU_CLEAR_HEADING_CORRECTION,ERR,") + err;
    }
    Serial.printf("[IMU-CAL-HEADING] cleared userCorrection=%.1f saved=1\n",
                  (double)g_imu.userHeadingCorrectionDeg());
    if (g_imu.yawIsAbsolute() && g_imu.headingState() == ImuHeadingState::IMU_ABSOLUTE_OK) {
        g_est.seedHeadingDeg(g_imu.yawDeg(), g_imu.yawSource());
    }
    return String("IMU_CLEAR_HEADING_CORRECTION,OK");
}

// IMU_HEADING_TEST: one-line dump of every heading ingredient.
String imuHeadingTestLine() {
    const float raw = g_imu.rawYawDeg();
    const float base = g_imu.baseHeadingDeg();
    const float finalH = g_imu.yawDeg();
    const uint32_t now = millis();
    char buf[512];
    snprintf(buf, sizeof(buf),
             "[IMU-HEADING-TEST] raw=%.1f base=%.1f final=%.1f source=%s state=%s abs=%d acc=%.2f sign=%.0f mount=%.1f staticAdjust=%.1f userCorrection=%.1f mag=%d norm=%.2f ageMs=%u manualYawTrusted=%d manualYawTrustedHeading=%.1f",
             (double)raw, (double)base, (double)finalH,
             ImuMath::yawSourceName(g_imu.yawSource()),
             ImuMath::headingStateName(g_imu.headingState()),
             g_imu.yawAbsoluteValid() ? 1 : 0,
             (double)g_imu.yawAccRad(),
             (double)IMU_ROT_YAW_SIGN,
             (double)IMU_ROT_YAW_OFFSET_DEG,
             (double)IMU_COMPASS_YAW_ADJUST_DEG,
             (double)g_imu.userHeadingCorrectionDeg(),
             g_imu.magDisturbed() ? 1 : 0,
             (double)g_imu.magNorm(),
             (unsigned)g_imu.ageMs(now),
             g_imu.manualYawTrusted() ? 1 : 0,
             (double)g_imu.manualTrustedHeadingDeg());
    Serial.println(buf);
    return String(buf);
}

// IMU_TRUST_CURRENT_HEADING_ONCE: debug-only override that lets the
// operator start navigation even when BNO085 hasn't reached ABSOLUTE_OK.
// Takes the current final yaw, marks it as manually trusted for this
// boot, and reseeds the StateEstimator. Not persisted; cleared by
// IMU_CLEAR_MANUAL_HEADING_TRUST or any reboot.
String imuTrustCurrentHeadingOnceLine() {
    const uint32_t now = millis();
    const uint32_t age = g_imu.ageMs(now);
    // Refuse on stale / no-data IMU so the operator can't seed garbage.
    if (!g_imu.hasData() || !g_imu.fresh() || age > 1500u) {
        char buf[160];
        snprintf(buf, sizeof(buf),
                 "[IMU-TRUST] refusing: imu_stale_or_no_data age=%u fresh=%d",
                 (unsigned)age, g_imu.fresh() ? 1 : 0);
        Serial.println(buf);
        return String("IMU_TRUST_CURRENT_HEADING_ONCE,ERR,imu_stale_or_no_data");
    }
    const float cur = g_imu.yawDeg();
    // Pass ROTATION_VECTOR as the source so seedHeadingDeg flips
    // absYawValid=true. The estimator will keep gyro-integration; the
    // manual trust flag is what lets precheck bypass the absYaw gate.
    g_est.seedHeadingDeg(cur, ImuYawSource::ROTATION_VECTOR);
    g_imu.setManualYawTrusted(true, cur);
    g_follow.reset();
    char buf[256];
    snprintf(buf, sizeof(buf),
             "[IMU-TRUST] trusted heading=%.1f source=%s state=%s acc=%.2f",
             (double)cur,
             ImuMath::yawSourceName(g_imu.yawSource()),
             ImuMath::headingStateName(g_imu.headingState()),
             g_imu.yawAccRad());
    Serial.println(buf);
    char out[256];
    snprintf(out, sizeof(out),
             "IMU_TRUST_CURRENT_HEADING_ONCE,OK,heading=%.1f,source=%s,state=%s,acc=%.2f",
             (double)cur,
             ImuMath::yawSourceName(g_imu.yawSource()),
             ImuMath::headingStateName(g_imu.headingState()),
             g_imu.yawAccRad());
    return String(out);
}

String imuClearManualHeadingTrustLine() {
    g_imu.clearManualYawTrusted();
    Serial.println("[IMU-TRUST] cleared");
    return String("IMU_CLEAR_MANUAL_HEADING_TRUST,OK");
}

// AUTO_ALIGN_HEADING_BY_RTK: drive ~1.5 m forward at low speed and compute
// heading from RTK displacement. The actual drive happens in main loop() via
// stepAlign(); this command just transitions IDLE → RUNNING and returns
// immediately so the Serial/WS handler does not block.
String autoAlignHeadingByRtkLine() {
    if (g_heading.alignState == AlignState::RUNNING) {
        return String("AUTO_ALIGN_HEADING_BY_RTK,ERR,already_running");
    }
    if (!autoAlignHeadingBegin(false /*fromWebSocket*/)) {
        // Begin() already filled g_heading.lastAlignError / state.
        char buf[96];
        snprintf(buf, sizeof(buf),
                 "AUTO_ALIGN_HEADING_BY_RTK,ERR,%s",
                 g_heading.lastAlignError ? g_heading.lastAlignError : "begin_failed");
        Serial.println(buf);
        return String(buf);
    }
    Serial.println("[ALIGN-RTK] request accepted; driving forward up to 1.5 m / 15 s");
    return String("AUTO_ALIGN_HEADING_BY_RTK,OK,driving");
}

// WebSocket-issued variant: the run is killed if the WS link drops so the
// app can react; we tag the run as "fromWebSocket" inside the alignment
// state.
String autoAlignHeadingByRtkLineWs() {
    if (g_heading.alignState == AlignState::RUNNING) {
        return String("AUTO_ALIGN_HEADING_BY_RTK,ERR,already_running");
    }
    if (!autoAlignHeadingBegin(true /*fromWebSocket*/)) {
        char buf[96];
        snprintf(buf, sizeof(buf),
                 "AUTO_ALIGN_HEADING_BY_RTK,ERR,%s",
                 g_heading.lastAlignError ? g_heading.lastAlignError : "begin_failed");
        Serial.println(buf);
        return String(buf);
    }
    Serial.println("[ALIGN-RTK] request accepted (ws); driving forward up to 1.5 m / 15 s");
    return String("AUTO_ALIGN_HEADING_BY_RTK,OK,driving");
}

// HEADING_STATUS: one-line dump of heading source / trust / align state.
String headingStatusLine() {
    const auto& e = g_est.get();
    const uint32_t now = millis();
    // headingAgeMs reflects the live freshness of the heading we are
    // actually using. For RTK_MOTION_ALIGNED_PLUS_IMU that's the
    // estimator age; for legacy one-shot RTK it falls back to
    // (now - trustedAtMs). For absolute / manual IMU trust it is the
    // BNO085 yaw age.
    uint32_t headingAgeMsOut = 0;
    switch (g_heading.source) {
        case HeadingSource::RTK_MOTION_ALIGNED_PLUS_IMU:
        case HeadingSource::RTK_MOTION_ALIGNED:
            headingAgeMsOut = e.headingValid ? e.headingAgeMs : 0;
            break;
        case HeadingSource::IMU_ABSOLUTE:
        case HeadingSource::IMU_MANUAL:
            headingAgeMsOut = g_imu.yawAgeMs(now);
            break;
        case HeadingSource::NONE:
        default:
            headingAgeMsOut = g_heading.trustedAtMs ? (now - g_heading.trustedAtMs) : 0;
            break;
    }
    const uint32_t absAge = g_imu.absYawAgeMs(now);
    const uint32_t relAge = g_imu.relYawAgeMs(now);
    const uint32_t gyroAge = g_imu.gyroAgeMs(now);
    char buf[512];
    snprintf(buf, sizeof(buf),
             "HEADING_STATUS,headingTrusted=%d,headingSource=%s,headingDeg=%.1f,"
             "headingAgeMs=%u,"
             "alignState=%s,alignStartedAtMs=%lu,"
             "lastAlignHeading=%.1f,lastAlignDist=%.3f,"
             "lastAlignDx=%.3f,lastAlignDy=%.3f,lastAlignHAcc=%.3f,"
             "lastAlignError=%s,"
             "estHeadingValid=%d,estHeadingFiltDeg=%.1f,estHeadingAgeMs=%u,estAbsYawValid=%d,"
             "imuYaw=%.1f,imuState=%s,imuAbs=%d,"
             "imuAbsAgeMs=%u,imuRelYawAgeMs=%u,gyroAgeMs=%u",
             g_heading.trusted ? 1 : 0,
             headingSourceName(g_heading.source),
             (double)g_heading.headingDeg,
             (unsigned)headingAgeMsOut,
             alignStateName(g_heading.alignState),
             (unsigned long)g_heading.alignStartedAtMs,
             (double)g_heading.lastAlignHeading,
             (double)g_heading.lastAlignDist,
             (double)g_heading.lastAlignDx,
             (double)g_heading.lastAlignDy,
             (double)g_heading.lastAlignHAcc,
             g_heading.lastAlignError ? g_heading.lastAlignError : "-",
             e.headingValid ? 1 : 0,
             (double)e.headingFiltDeg,
             (unsigned)e.headingAgeMs,
             e.absYawValid ? 1 : 0,
             (double)g_imu.yawDeg(),
             ImuMath::headingStateName(g_imu.headingState()),
             g_imu.yawAbsoluteValid() ? 1 : 0,
             (unsigned)absAge,
             (unsigned)relAge,
             (unsigned)gyroAge);
    Serial.println(buf);
    return String(buf);
}

// CLEAR_HEADING_TRUST: drop the trusted flag (does not touch RTK).
String clearHeadingTrustLine() {
    clearHeadingTrust();
    Serial.println("[HEADING-TRUST] cleared");
    return String("CLEAR_HEADING_TRUST,OK");
}

// NAV_START_AUTO_ALIGN: if trusted → start route. If not trusted but RTK
// good → begin alignment; main loop will drive and the next NAV_START
// command (from app) is expected after we report OK. If RTK bad → ERR.
// Note: this does NOT block on the alignment — alignment is async.
String navStartAutoAlignLine() {
    if (g_heading.trusted) {
        Serial.println("[NAV-AUTO-ALIGN] heading already trusted; sending NAV_START");
        return roverdbg::handleNavStartLine();
    }
    if (g_heading.alignState == AlignState::RUNNING) {
        return String("NAV_START_AUTO_ALIGN,ERR,already_aligning");
    }
    if (!autoAlignHeadingBegin(false /*fromWebSocket*/)) {
        char buf[96];
        snprintf(buf, sizeof(buf),
                 "NAV_START_AUTO_ALIGN,ERR,%s",
                 g_heading.lastAlignError ? g_heading.lastAlignError : "begin_failed");
        Serial.println(buf);
        return String(buf);
    }
    Serial.println("[NAV-AUTO-ALIGN] alignment started; NAV_START will be sent after OK");
    return String("NAV_START_AUTO_ALIGN,OK,alignment_started");
}

// WebSocket-issued variant of NAV_START_AUTO_ALIGN.
String navStartAutoAlignLineWs() {
    if (g_heading.trusted) {
        Serial.println("[NAV-AUTO-ALIGN] heading already trusted (ws); sending NAV_START");
        return roverdbg::handleNavStartLine();
    }
    if (g_heading.alignState == AlignState::RUNNING) {
        return String("NAV_START_AUTO_ALIGN,ERR,already_aligning");
    }
    if (!autoAlignHeadingBegin(true /*fromWebSocket*/)) {
        char buf[96];
        snprintf(buf, sizeof(buf),
                 "NAV_START_AUTO_ALIGN,ERR,%s",
                 g_heading.lastAlignError ? g_heading.lastAlignError : "begin_failed");
        Serial.println(buf);
        return String(buf);
    }
    Serial.println("[NAV-AUTO-ALIGN] alignment started (ws); NAV_START will be sent after OK");
    return String("NAV_START_AUTO_ALIGN,OK,alignment_started");
}

// Helper used by NAV_START_AUTO_ALIGN when heading is already trusted.
// Mirrors the WS NAV_START acceptance check, but goes through the Serial
// path so the same gates apply.
String handleNavStartLine() {
    // We delegate to the same logic WS uses: try to call _route->start
    // and _navRequested=true. The simplest path is to ask the WS layer.
    // Since we are on rover.cpp and have direct access to g_route/g_ws,
    // we run the same gate locally.
    const auto& e = g_est.get();
    uint32_t now = millis();
    if (!g_route.isReady()) {
        return String("ERR,NO_ROUTE");
    } else if (!e.originSet) {
        return String("ERR,NO_ORIGIN");
    } else if (e.sol != SOL_FIXED) {
        return String("ERR,RTK_NOT_FIXED");
    } else if (e.hAcc > SAFE_HACC_FIXED_M) {
        return String("ERR,HACC");
    } else if (e.pvtAgeMs > SAFE_PVT_AGE_MS ||
               e.acceptedPositionAgeMs > SAFE_ACCEPTED_POS_AGE_MS) {
        return String("ERR,POSITION_STALE");
    } else if (e.rejectedPositionFixes > SAFE_REJECTED_POSITION_FIXES_MAX) {
        return String("ERR,GPS_JUMP");
    } else if (!e.headingValid || e.headingAgeMs > SAFE_HEADING_AGE_MS) {
        return String("ERR,HEADING_STALE");
    } else {
        g_route.start();
        g_ws.setNavRequested(true);
        return String("OK,NAV_START");
    }
}

// STOP: drop the route, abort any running alignment, stop motors. Single
// source of truth for both Serial `STOP` and WebSocket `STOP` so the two
// surfaces cannot drift.
String handleStopLine() {
    if (g_heading.alignState == AlignState::RUNNING) {
        g_align.active = false;
        g_heading.alignState = AlignState::ERR;
        g_heading.lastAlignError = "stopped";
        g_heading.lastAlignDist = 0;
        g_heading.lastAlignDx = 0;
        g_heading.lastAlignDy = 0;
        Serial.println("[ALIGN-RTK] abort: stopped");
    }
    g_route.stop();
    g_motor.stopImmediately();
    // Clear the serialDebugMotion override so ws_disconnected can
    // once again become ESTOP for any following session.
    if (g_serialMotion.active) {
        serialMotionEnd("stopped");
    }
    Serial.println("[STOP] motors stopped");
    return String("OK STOP");
}

}  // namespace roverdbg
