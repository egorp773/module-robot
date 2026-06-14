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
    bool  turning = false;          // режим разворота на месте (с гистерезисом)
    int8_t turnSign = 0;
    uint32_t turningSinceMs = 0;
    uint32_t turnCooldownUntilMs = 0;
    bool  arrived = false;
    uint32_t arrivedSinceMs = 0;

    void reset() {
        wpIdx = 0; running = false; paused = false;
        linearMps = angularRadps = headingErr = crossTrack = distToTarget = 0;
        turning = false; turnSign = 0; turningSinceMs = 0; turnCooldownUntilMs = 0;
        arrived = false; arrivedSinceMs = 0;
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
    if (total == 0 || g_follow.wpIdx >= total) {
        g_motor.setLinearAngularSpeed(0, 0, true);
        g_follow.running = false;
        g_follow.arrived = true;
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
        } else {
            lineDx = dx;
            lineDy = dy;
        }
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

    float w = 0;
    float absErr = fabsf(headingErr);
    uint32_t nowMs = millis();

    // ----- РЕЖИМ 1: разворот на месте (курс сильно не туда) -----
    // Ключевое отличие от старого кода: гистерезис (входим >ENTER, выходим <EXIT — без дёрганья)
    // и угловая скорость, ПРОПОРЦИОНАЛЬНАЯ ошибке, но с ПОЛОМ выше трения гусениц.
    if (!g_follow.turning &&
        nowMs >= g_follow.turnCooldownUntilMs &&
        absErr > ROVER_TURN_IN_PLACE_ENTER_DEG) {
        g_follow.turning = true;
        g_follow.turnSign = (headingErr >= 0) ? 1 : -1;
        g_follow.turningSinceMs = nowMs;
    }
    if (g_follow.turning && absErr < ROVER_TURN_IN_PLACE_EXIT_DEG) {
        g_follow.turning = false;
        g_follow.turnSign = 0;
        g_follow.turningSinceMs = 0;
    }
    if (g_follow.turning &&
        g_follow.turningSinceMs != 0 &&
        (nowMs - g_follow.turningSinceMs) > ROVER_TURN_IN_PLACE_TIMEOUT_MS) {
        g_follow.turning = false;
        g_follow.turnSign = 0;
        g_follow.turningSinceMs = 0;
        g_follow.turnCooldownUntilMs = nowMs + ROVER_TURN_IN_PLACE_COOLDOWN_MS;
    }

    if (g_follow.turning) {
        baseSpeed = 0;
        // w пропорциональна ошибке, но не ниже MIN (пробить трение) и не выше MAX.
        float mag = ROVER_K_HEADING * (absErr * M_PI / 180.0f);
        mag = clampf(mag, ROVER_TURN_MIN_RADPS, ROVER_MAX_ANGULAR_RADPS);
        int8_t sign = g_follow.turnSign != 0 ? g_follow.turnSign : ((headingErr >= 0) ? 1 : -1);
        w = sign * mag;   // + = по часовой
    } else {
        // ----- РЕЖИМ 2: движение со Stanley-рулёжкой -----
        // Притормаживаем у цели и при большой ошибке курса (плавнее на змейке).
        if (dist < 0.30f)      baseSpeed *= 0.35f;
        else if (dist < 0.60f) baseSpeed *= 0.65f;
        baseSpeed *= clampf(1.0f - absErr / 90.0f, 0.25f, 1.0f);

        // Stanley: руль = ошибка курса + atan(k*crossTrack / (soft + v))
        float headingTerm = ROVER_K_HEADING * headingErr * M_PI / 180.0f;
        float crosstrackTerm = atan2f(ROVER_K_CROSSTRACK * g_follow.crossTrack,
                                      ROVER_STANLEY_SOFT_SPEED + fabsf(baseSpeed));
        w = headingTerm + crosstrackTerm;
    }
    w = clampf(w, -ROVER_MAX_ANGULAR_RADPS, ROVER_MAX_ANGULAR_RADPS);
    g_follow.linearMps   = baseSpeed;
    g_follow.angularRadps = w;
    g_follow.distToTarget = dist;

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
    si.originLocked = g_est.get().originSet;
    si.routeReady   = g_route.isReady();
    g_safety.tick(now, si, g_est, g_imu);

    if (g_ws.navRequested() && g_route.isRunning()) {
        if (!g_follow.running) {
            g_follow.reset();
            g_follow.running = true;
        }
        stepFollower();
    }

    NavStateOut nso;
    if (g_safety.level() == SAFETY_ESTOP) {
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
