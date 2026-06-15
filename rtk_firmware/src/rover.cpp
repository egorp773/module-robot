// rover.cpp - entry point. Rover side.
// Sunray-style: единый цикл с тасками по периоду, всё в loop().

#include <Arduino.h>
#include "RtkConfig.h"
#include "StateEstimator.h"
#include "Imu.h"
#include "Gnss.h"
#include "RtcmLink.h"
#include "Route.h"
#include "Motor.h"
#include "Safety.h"
#include "WsServer.h"
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

// --- waypoint follower (Sunray-style Stanley line tracker) ---
struct Follower {
    int   wpIdx = 0;
    bool  running = false;
    bool  paused  = false;
    float linearMps = 0;
    float angularRadps = 0;
    float headingErr = 0;
    float crossTrack = 0;
    float distToTarget = 0;
    // Поля разворота на месте удалены: GPS-only heading не позволяет разворот стоя
    // (курс набирается только в движении), поэтому всегда едем вперёд по дуге.
    bool  arrived = false;
    uint32_t arrivedSinceMs = 0;
    const char* faultReason = nullptr;
    bool progressInit = false;
    float progressX = 0;
    float progressY = 0;
    uint32_t progressSinceMs = 0;

    void reset() {
        wpIdx = 0; running = false; paused = false;
        linearMps = angularRadps = headingErr = crossTrack = distToTarget = 0;
        arrived = false; arrivedSinceMs = 0;
        faultReason = nullptr;
        progressInit = false;
        progressX = progressY = 0;
        progressSinceMs = 0;
    }
} g_follow;

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float wrapDeg180Local(float d) {
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

// Раньше прокачивала g_motor.loop() вручную во время ожидания. Теперь поток команд
// держит фоновая TX-задача на ядре 0 (g_motor.startTxTask), поэтому здесь просто ждём —
// дёргать loop() вручную НЕЛЬЗЯ: будет гонка двух нитей за один Serial2.
static void serviceMotorFor(uint32_t durationMs) {
    delay(durationMs);
}

static void followerFault(const char* reason) {
    g_follow.running = false;
    g_follow.arrived = false;
    g_follow.faultReason = reason;
    g_follow.progressInit = false;
    g_route.finish();
    g_motor.stopImmediately();
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

static void connectWiFi() {
    WiFi.mode(WIFI_STA);
    // ВАЖНО: НЕ снижать TX power здесь. 11dBm наводит помехи на UART2 к hoverboard
    // → плата видит мусор в протоколе → пищит. 11dBm включаем ПОЗЖЕ, после мотора и RTK.
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    IPAddress ip, gw, sn, dns;
    ip.fromString(ROVER_IP);
    gw.fromString("192.168.31.1");
    sn.fromString("255.255.255.0");
    dns = gw;
    WiFi.config(ip, gw, sn, dns);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    int t = 0;
    while (WiFi.status() != WL_CONNECTED && t < 30) {
        serviceMotorFor(500);
        Serial.print(".");
        t++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[ROVER] wifi connected ip=%s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\n[ROVER] wifi TIMEOUT");
    }
}

static void stepFollower() {
    if (!g_route.isRunning()) {
        g_follow.linearMps = g_follow.angularRadps = 0;
        g_motor.setLinearAngularSpeed(0, 0, true);
        return;
    }
    if (g_route.isPaused()) {
        g_motor.setLinearAngularSpeed(0, 0, true);
        return;
    }
    const auto& e = g_est.get();
    int total = g_route.count();
    NavPoint current{e.x, e.y};
    if (!g_route.positionAllowed(current, ROVER_BOUNDARY_TOLERANCE_M)) {
        followerFault("perimeter_violation");
        return;
    }
    if (total == 0 || g_follow.wpIdx >= total) {
        g_motor.setLinearAngularSpeed(0, 0, true);
        g_follow.running = false;
        g_follow.arrived = true;
        g_route.finish();
        return;
    }

    const Waypoint& wp = g_route.waypoint(g_follow.wpIdx);
    const Waypoint* prevWp = (g_follow.wpIdx > 0) ? &g_route.waypoint(g_follow.wpIdx - 1) : nullptr;
    float dx = wp.p.x - e.x;
    float dy = wp.p.y - e.y;
    float dist = sqrtf(dx*dx + dy*dy);

    // handoff distance: для промежуточных — короче, для финала — arrival radius
    bool finalWp = (g_follow.wpIdx + 1 >= total);
    float handoff;
    if (finalWp) {
        handoff = ROVER_ARRIVAL_RADIUS;
    } else {
        const Waypoint& next = g_route.waypoint(g_follow.wpIdx + 1);
        float segLen = sqrtf((next.p.x - wp.p.x)*(next.p.x - wp.p.x) +
                             (next.p.y - wp.p.y)*(next.p.y - wp.p.y));
        handoff = max(ROVER_HANDOFF_MIN_M, min(ROVER_HANDOFF_MAX_M, segLen * ROVER_HANDOFF_FRAC));
    }

    if (dist < handoff) {
        g_follow.wpIdx++;
        if (g_follow.wpIdx >= total) {
            g_follow.running = false;
            g_follow.arrived = true;
            g_route.finish();
            g_motor.setLinearAngularSpeed(0, 0, true);
            return;
        }
        stepFollower();
        return;
    }

    float lineDx = dx;
    float lineDy = dy;
    g_follow.crossTrack = 0;
    if (prevWp) {
        lineDx = wp.p.x - prevWp->p.x;
        lineDy = wp.p.y - prevWp->p.y;
        float lineLen = sqrtf(lineDx * lineDx + lineDy * lineDy);
        if (lineLen > 0.05f) {
            float rx = e.x - prevWp->p.x;
            float ry = e.y - prevWp->p.y;
            g_follow.crossTrack = (lineDx * ry - lineDy * rx) / lineLen;
            if (fabsf(g_follow.crossTrack) > ROVER_CROSSTRACK_HOLD_M) {
                followerFault("cross_track_limit");
                return;
            }
        } else {
            lineDx = dx;
            lineDy = dy;
        }
    }

    if (!g_route.segmentAllowed(current, wp.p, ROVER_BOUNDARY_TOLERANCE_M, ROVER_BOUNDARY_SAMPLE_M)) {
        followerFault("route_segment_blocked");
        return;
    }

    // Желаемый курс — вдоль активного сегмента пути (не только на waypoint).
    float desiredHeading = atan2f(lineDx, lineDy) * 180.0f / M_PI;
    if (desiredHeading < 0) desiredHeading += 360.0f;
    float headingErr = wrapDeg180Local(desiredHeading - e.headingFiltDeg);
    g_follow.headingErr = headingErr;

    // Базовая скорость по уровню безопасности (RTK-качество).
    float baseSpeed = ROVER_MAX_SPEED_MPS;
    if (g_safety.level() == SAFETY_DEGRADED) baseSpeed = ROVER_FLOAT_SPEED;
    else if (g_safety.level() == SAFETY_HOLD) baseSpeed = 0;
    else if (g_safety.level() == SAFETY_ESTOP) baseSpeed = 0;

    float absErr = fabsf(headingErr);

    // ===== GPS-only рулёжка: ВСЕГДА едем вперёд, поворот по ДУГЕ =====
    // Курс берётся только из GPS motion heading и обновляется лишь когда робот ДВИЖЕТСЯ.
    // Поэтому разворот на месте невозможен: стоя, робот не узнает, что повернулся, и
    // крутился бы вечно. Вместо этого всегда даём линейную скорость (робот едет),
    // а большую ошибку курса отрабатываем угловой скоростью — получается поворот по дуге,
    // во время которого GPS-курс набирается и ошибка естественно уменьшается.
    // (Эталон поведения: OpenMower/Sunray на одной GPS-антенне.)

    // Притормаживаем у цели — но НЕ до нуля, иначе курс перестанет набираться.
    if (dist < 0.30f)      baseSpeed *= 0.45f;
    else if (dist < 0.60f) baseSpeed *= 0.70f;
    // При большой ошибке курса едем медленнее (круче дуга), но сохраняем ход вперёд:
    // пол 0.35 гарантирует, что робот всё равно движется и GPS обновляет курс.
    baseSpeed *= clampf(1.0f - absErr / 120.0f, 0.35f, 1.0f);

    // Угловая скорость: ошибка курса (P-регулятор) + Stanley cross-track.
    float headingTerm = ROVER_K_HEADING * headingErr * M_PI / 180.0f;
    float crosstrackTerm = atan2f(ROVER_K_CROSSTRACK * g_follow.crossTrack,
                                  ROVER_STANLEY_SOFT_SPEED + fabsf(baseSpeed));
    float w = headingTerm + crosstrackTerm;
    w = clampf(w, -ROVER_MAX_ANGULAR_RADPS, ROVER_MAX_ANGULAR_RADPS);
    g_follow.linearMps   = baseSpeed;
    g_follow.angularRadps = w;
    g_follow.distToTarget = dist;

    if (!checkFollowerProgress(e, baseSpeed)) {
        return;
    }

    if (!g_ws.navRequested() || !g_route.isRunning()) {
        g_motor.stopImmediately();
        return;
    }

    if (g_safety.allowMotion()) {
        g_motor.setLinearAngularSpeed(baseSpeed, w, true);
    } else {
        g_motor.stopImmediately();
    }
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
    g_est.seedHeadingDeg(ROVER_INITIAL_HEADING_DEG);   // 176° — реальный старт-курс робота
    g_route.begin();
    g_safety.begin();

    g_rtcm.begin(g_gnss, BASE_IP, RTCM_TCP_PORT, RTCM_UDP_PORT);
    g_ws.begin(g_est, g_imu, g_gnss, g_rtcm, g_route, g_motor, g_safety, WS_PORT);

    Serial.println("[ROVER] ready");
}

void loop() {
    uint32_t now = millis();
    static uint32_t lastBat = 0;

    g_imu.loop();
    g_gnss.loop();
    g_rtcm.loop();
    // g_motor.loop() НЕ зовём — поток команд ведёт TX-задача на ядре 0 (startTxTask).

    // WiFi 11dBm включаем ПОЗЖЕ (через 3 сек), когда мотор уже стабильно шлёт — иначе
    // TX-помехи 11dBm наводят мусор на UART2 к hoverboard и она пищит.
    static uint32_t g_lowTxAt = 3000;
    if (g_lowTxAt != 0 && millis() > g_lowTxAt) {
        WiFi.setTxPower(WIFI_POWER_11dBm);
        g_lowTxAt = 0;
        Serial.println("[ROVER] wifi tx power -> 11dBm");
    }

    if (g_gnss.consumeFreshPvt()) {
        g_est.onPvt(now,
            g_gnss.lastLatE7(), g_gnss.lastLonE7(), g_gnss.lastHeightMm(),
            g_gnss.lastHAccMm(), g_gnss.lastVAccMm(),
            g_gnss.lastGSpeedMmps(), g_gnss.lastHeadMotDe5(),
            g_gnss.lastFixType(), g_gnss.lastCarrierSol(), g_gnss.lastDiffSoln(),
            g_gnss.lastNumSv(), g_gnss.lastPDop());
    }
    g_est.onImu(now, g_imu.yawRateDps(), g_imu.fresh() && g_imu.ageMs(now) < SAFE_IMU_AGE_MS);
    g_est.tick(now);

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
    si.acceptedPositionAgeMs = g_est.get().acceptedPositionAgeMs;
    si.headingAgeMs = g_est.get().headingAgeMs;
    si.rejectedPositionFixes = g_est.get().rejectedPositionFixes;
    si.originLocked = g_est.get().originSet;
    si.routeReady   = g_route.isReady();
    g_safety.tick(now, si, g_est, g_imu);
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

    static uint32_t lastLog = 0;
    if (now - lastLog > 5000) {
        lastLog = now;
        const auto& e = g_est.get();
        const char* src = (g_rtcm.source() == RTCM_UDP) ? "udp" : "none";
        Serial.printf("[ROVER] sol=%d sv=%d hAcc=%.3f head=%.1f gpsHead=%.1f yawRate=%.1f imuYaw=%.1f "
                      "spd=%.2f pvtAge=%u rtcmAge=%lu src=%s imuAge=%u bat=%d "
                      "motFB=%d motL=%d motR=%d sp=%d st=%d safety=%d reason=%s\n",
            (int)e.sol, e.numSv, e.hAcc, e.headingFiltDeg, e.headingDeg,
            g_imu.yawRateDps(), g_imu.yawDeg(), e.speedMps, e.pvtAgeMs,
            g_rtcm.transportAgeMs(now), src,
            g_imu.ageMs(now), g_motor.batteryPercent(),
            g_motor.haveFeedback() ? 1 : 0,
            g_motor.currentLeftPwm(), g_motor.currentRightPwm(),
            g_motor.lastSpeedCmd(), g_motor.lastSteerCmd(),
            (int)g_safety.level(), g_safety.reason());
    }
}
