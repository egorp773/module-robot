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
    bool  arrived = false;
    uint32_t arrivedSinceMs = 0;
    const char* faultReason = nullptr;
    bool progressInit = false;
    float progressX = 0;
    float progressY = 0;
    uint32_t progressSinceMs = 0;
    // Recovery counter: сколько fault подряд в одном WP без прогресса. После порога —
    // не финишируем маршрут, а переключаемся в degraded-режим и пробуем снова.
    int   faultCount = 0;
    int   lastFaultWp = -1;

    void reset() {
        wpIdx = 0; running = false; paused = false;
        linearMps = angularRadps = headingErr = crossTrack = distToTarget = 0;
        arrived = false; arrivedSinceMs = 0;
        faultReason = nullptr;
        progressInit = false;
        progressX = progressY = 0;
        progressSinceMs = 0;
        faultCount = 0;
        lastFaultWp = -1;
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

static bool waitForInitialImuHeading(uint32_t timeoutMs) {
    uint32_t start = millis();
    while ((uint32_t)(millis() - start) < timeoutMs) {
        g_imu.loop();
        uint32_t now = millis();
        if (g_imu.fresh() && g_imu.yawAgeMs(now) < SAFE_IMU_AGE_MS) {
            return true;
        }
        delay(10);
    }
    return false;
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

    // Hard guard: выход за boundary > HARD tolerance — это однозначно потеря позиции
    // (сдвиг origin, обрыв RTCM, накопление ошибки). После 3 попыток — ERROR.
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

    // ===== Активный сегмент =====
    // Сегмент = (prevWp, wp). Для первого WP prevWp = nullptr — это вырожденный случай
    // (старт маршрута, робот ещё в начале первого сегмента).
    float segDx, segDy, segLen;
    if (prevWp) {
        segDx = wp.p.x - prevWp->p.x;
        segDy = wp.p.y - prevWp->p.y;
    } else {
        // Старт: "сегмент" = направление на WP (для расчёта heading, без crosstrack).
        segDx = wp.p.x - current.x;
        segDy = wp.p.y - current.y;
    }
    segLen = sqrtf(segDx * segDx + segDy * segDy);

    // ===== Lookahead target на сегменте (Sunray LOOOKAHEAD) =====
    // Целевая точка = точка на сегменте на расстоянии lookahead ВПЕРЁД от проекции
    // робота на сегмент. Если проекция за началом — цепляем к началу; если за WP —
    // цепляемся к WP (для коротких сегментов).
    float t = 0.0f;
    if (prevWp && segLen > 0.05f) {
        t = ((current.x - prevWp->p.x) * segDx + (current.y - prevWp->p.y) * segDy) / (segLen * segLen);
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
    }
    // Точка проекции
    float projX, projY;
    if (prevWp) {
        projX = prevWp->p.x + segDx * t;
        projY = prevWp->p.y + segDy * t;
    } else {
        projX = current.x; projY = current.y;
    }
    // Lookahead target
    bool finalWp = (g_follow.wpIdx + 1 >= total);
    float maxLa = (prevWp && !finalWp) ? min(ROVER_LOOKAHEAD_M, segLen * 0.5f) : 0.0f;
    float laT = t + (maxLa / max(segLen, 0.05f));
    if (laT > 1.0f) laT = 1.0f;
    NavPoint lookAt;
    if (prevWp) {
        lookAt.x = prevWp->p.x + segDx * laT;
        lookAt.y = prevWp->p.y + segDy * laT;
    } else {
        lookAt = wp.p;
    }
    // Финальный WP: целимся прямо в WP (lookahead = 0)
    if (finalWp) {
        lookAt = wp.p;
    }

    // ===== Расстояние до WP (для handoff и телеметрии) =====
    float wdx = wp.p.x - current.x;
    float wdy = wp.p.y - current.y;
    float distToWp = sqrtf(wdx * wdx + wdy * wdy);
    g_follow.distToTarget = distToWp;

    // ===== Crosstrack (поперечное отклонение от линии сегмента) =====
    g_follow.crossTrack = 0;
    if (prevWp && segLen > 0.05f) {
        float rx = current.x - prevWp->p.x;
        float ry = current.y - prevWp->p.y;
        // Знак: положительный = слева от направления движения
        g_follow.crossTrack = (segDx * ry - segDy * rx) / segLen;
        if (fabsf(g_follow.crossTrack) > ROVER_CROSSTRACK_HARD_M) {
            // hard: 1.0 м — действительно потеряли путь, делаем recovery
            followerFault("cross_track_hard");
            return;
        }
    }

    // ===== Handoff: переключение на следующий WP =====
    float handoff;
    if (finalWp) {
        handoff = ROVER_ARRIVAL_RADIUS;
    } else if (prevWp) {
        handoff = max(ROVER_HANDOFF_MIN_M, min(ROVER_HANDOFF_MAX_M, segLen * ROVER_HANDOFF_FRAC));
    } else {
        // старт маршрута: пока нет prevWp, фиксированно ждём 0.5м отхода
        handoff = 0.5f;
    }
    if (distToWp < handoff) {
        g_follow.faultCount = 0;   // сброс — мы в норме
        g_follow.lastFaultWp = -1;
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

    if (!g_route.segmentAllowed(current, wp.p, ROVER_BOUNDARY_TOLERANCE_M, ROVER_BOUNDARY_SAMPLE_M)) {
        followerFault("route_segment_blocked");
        return;
    }

    // ===== Желаемый heading = направление на lookahead target =====
    // Это плавнее, чем "вдоль сегмента" — при подходе к финалу lookahead
    // автоматически сокращается, и headingErr естественно → 0.
    float ldx = lookAt.x - current.x;
    float ldy = lookAt.y - current.y;
    if (ldx*ldx + ldy*ldy < 1e-4f) {
        ldx = segDx; ldy = segDy;
    }
    float desiredHeading = atan2f(ldx, ldy) * 180.0f / M_PI;
    if (desiredHeading < 0) desiredHeading += 360.0f;
    float headingErr = wrapDeg180Local(desiredHeading - e.headingFiltDeg);
    g_follow.headingErr = headingErr;

    // ===== Скорость по safety + heading + crosstrack =====
    float baseSpeed = ROVER_MAX_SPEED_MPS;
    if (g_safety.level() == SAFETY_DEGRADED) baseSpeed = ROVER_FLOAT_SPEED;
    else if (g_safety.level() == SAFETY_HOLD) baseSpeed = 0;
    else if (g_safety.level() == SAFETY_ESTOP) baseSpeed = 0;

    float absErr = fabsf(headingErr);
    float absCt  = fabsf(g_follow.crossTrack);

    // GPS-only рулёжка: всегда едем вперёд, поворот по дуге (Stanley).
    // Притормаживаем у цели — но не до нуля, иначе GPS heading заморозится.
    if (distToWp < 0.30f)      baseSpeed *= 0.45f;
    else if (distToWp < 0.60f) baseSpeed *= 0.70f;

    // При большой heading-ошибке — медленнее (круче дуга), но пол 0.40 гарантирует
    // что робот движется и GPS-heading обновляется.
    baseSpeed *= clampf(1.0f - absErr / 120.0f, 0.40f, 1.0f);

    // При большом crosstrack — тоже медленнее (корректирующая дуга).
    if (absCt > ROVER_CROSSTRACK_SOFT_M) {
        baseSpeed *= clampf(1.0f - (absCt - ROVER_CROSSTRACK_SOFT_M) / 0.5f, 0.50f, 1.0f);
    }

    // Если уже были fault на этом WP — едем осторожнее.
    if (g_follow.faultCount > 0) {
        baseSpeed *= 0.6f;
    }

    // ===== Stanley =====
    // headingTerm — P по heading-ошибке (в рад/с)
    // crosstrackTerm — atan2(K*ct, soft + |v|) — пропорционально отклонению,
    //                  делённому на скорость (классический Stanley)
    float headingTerm = ROVER_K_HEADING * headingErr * M_PI / 180.0f;
    float crosstrackTerm = atan2f(ROVER_K_CROSSTRACK * g_follow.crossTrack,
                                  ROVER_STANLEY_SOFT_SPEED + fabsf(baseSpeed));
    float w = headingTerm + crosstrackTerm;
    w = clampf(w, -ROVER_MAX_ANGULAR_RADPS, ROVER_MAX_ANGULAR_RADPS);
    g_follow.linearMps   = baseSpeed;
    g_follow.angularRadps = w;

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
    if (waitForInitialImuHeading(1500)) {
        g_est.seedHeadingDeg(g_imu.yawDeg());
        Serial.printf("[ROVER] heading seed: %.1f° (IMU corrected, raw=%.1f rotOffset=%.1f corr=%.1f mag=%d acc=%.2f)\n",
                      (double)g_imu.yawDeg(), (double)g_imu.rawYawDeg(),
                      (double)IMU_ROT_YAW_OFFSET_DEG, (double)IMU_COMPASS_CORRECTION_DEG,
                      g_imu.yawFromMag() ? 1 : 0,
                      (double)g_imu.yawAccRad());
    } else {
        g_est.seedHeadingDeg(ROVER_INITIAL_HEADING_DEG);
        Serial.printf("[ROVER] heading seed: %.1f° (fallback, IMU unavailable)\n",
                      (double)ROVER_INITIAL_HEADING_DEG);
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
    //   GO\n    — CAL + загрузить маршрут из 2 точек: (0,0) → (0,3) + стартовать.
    //              Робот поедет ровно 3 метра туда, куда смотрит носом, и встанет.
    //              Идеальный тест для проверки heading + моторов без приложения.
    //   STOP\n  — стоп моторов немедленно.
    //   LOG,0\n — выключить периодический лог (чтобы терминал не летал).
    //   LOG,1\n — включить обратно (200мс).
    if (Serial.available()) {
        char buf[32];
        size_t n = Serial.readBytesUntil('\n', buf, sizeof(buf) - 1);
        if (n > 0) {
            buf[n] = 0;
            if (strcmp(buf, "CAL") == 0) {
                float cur = g_imu.yawDeg();
                g_est.seedHeadingDeg(0.0f);
                g_follow.reset();
                Serial.printf("[CAL] heading reseeded: imuYaw was %.1f → head=0\n",
                              (double)cur);
            } else if (strcmp(buf, "GO") == 0) {
                if (!g_est.get().originSet) {
                    Serial.println("[GO] refusing: no origin yet (wait for RTK FIX/FLOAT)");
                } else {
                    float cur = g_imu.yawDeg();
                    g_est.seedHeadingDeg(0.0f);
                    g_follow.reset();
                    g_route.beginUpload(2, g_est.get().originLat, g_est.get().originLon);
                    g_route.addWaypoint(0, 0.0f, 0.0f);
                    g_route.addWaypoint(1, 0.0f, 3.0f);
                    g_route.beginBoundary(4);
                    g_route.addBoundaryPoint(0, -2.0f, -2.0f);
                    g_route.addBoundaryPoint(1,  2.0f, -2.0f);
                    g_route.addBoundaryPoint(2,  2.0f,  5.0f);
                    g_route.addBoundaryPoint(3, -2.0f,  5.0f);
                    g_route.endBoundary();
                    g_route.beginForbidden(0, nullptr);
                    g_route.endForbidden();
                    g_route.endUpload();
                    g_route.start();
                    Serial.printf("[GO] CAL+route+start: imuYaw was %.1f → head=0, "
                                  "route: (0,0) → (0,3), speed=%.2f m/s\n",
                                  (double)cur, (double)ROVER_MAX_SPEED_MPS);
                }
            } else if (strcmp(buf, "STOP") == 0) {
                g_motor.stopImmediately();
                Serial.println("[STOP] motors stopped");
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
            g_gnss.lastNumSv(), g_gnss.lastPDop());
    }
    g_est.onImu(now, g_imu.yawRateDps(), g_imu.fresh() && g_imu.ageMs(now) < SAFE_IMU_AGE_MS,
                g_imu.yawDeg(), g_imu.yawAgeMs(now) < SAFE_IMU_AGE_MS,
                g_imu.yawAccRad());
    // Hoverboard feedback → EKF predict. TX-задача на ядре 0 обновляет _fb в Motor;
    // здесь читаем актуальные обороты. Если feedback ещё не пришёл, шлёт 0/0.
    g_est.onHoverboardFeedback(now, g_motor.speedLeftMeas(), g_motor.speedRightMeas());
    g_est.tick(now);

    // Auto-origin: если origin ещё не задан, но есть валидный fix — захватываем текущую
    // позицию как origin. Иначе est.x, est.y останутся 0,0 пока пользователь не отправит
    // маршрут, и карта в приложении будет показывать робота в (0,0) — "он в разных точках".
    // FIXED — идеал, FLOAT с hAcc<=5см тоже достаточно для старта.
    {
        static uint32_t lastAutoOriginLogMs = 0;
        if (!g_est.get().originSet) {
            const auto& e = g_est.get();
            bool ok = (e.sol == SOL_FIXED) ||
                      (e.sol == SOL_FLOAT && e.hAcc <= 0.05f);
            if (ok) {
                g_est.setOrigin(e.lat, e.lon);
                if (now - lastAutoOriginLogMs > 2000) {
                    Serial.printf("[AUTO-ORIGIN] captured: lat=%.7f lon=%.7f sol=%d hAcc=%.3f\n",
                                  g_est.get().originLat, g_est.get().originLon,
                                  (int)e.sol, e.hAcc);
                    lastAutoOriginLogMs = now;
                }
            }
        }
    }

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
        Serial.printf("[R] sol=%d hAcc=%.3f head=%.1f imuYaw=%.1f mag=%d acc=%.2f "
                      "spd=%.2f pvtAge=%u rtcmAge=%lu imuAge=%u "
                      "wp=%d/%d dWp=%.2f hErr=%.1f ct=%.2f "
                      "motL=%d motR=%d sp=%d st=%d "
                      "safety=%d %s fault=%s\n",
            (int)e.sol, e.hAcc, e.headingFiltDeg,
            g_imu.yawDeg(), g_imu.yawFromMag() ? 1 : 0, g_imu.yawAccRad(),
            e.speedMps, e.pvtAgeMs,
            g_rtcm.transportAgeMs(now),
            g_imu.ageMs(now),
            g_follow.wpIdx, g_route.count(),
            g_follow.distToTarget, g_follow.headingErr, g_follow.crossTrack,
            g_motor.currentLeftPwm(), g_motor.currentRightPwm(),
            g_motor.lastSpeedCmd(), g_motor.lastSteerCmd(),
            (int)g_safety.level(), g_safety.reason(),
            g_follow.faultReason ? g_follow.faultReason : "-");
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

void handleCal() {
    float cur = g_imu.yawDeg();
    g_est.seedHeadingDeg(0.0f);
    g_follow.reset();
    Serial.printf("[CAL] heading reseeded: imuYaw was %.1f → head=0\n", (double)cur);
}

bool handleGo() {
    if (!g_est.get().originSet) {
        Serial.println("[GO] refusing: no origin yet (wait for RTK FIX/FLOAT)");
        return false;
    }
    // Жёсткий гейт: если IMU не отвечает дольше 500мс — отказываем.
    // Иначе heading "застрянет" на старом значении, Stanley поведёт не туда,
    // и робот развернётся при старте. Это страховка от I2C-зависания BNO085.
    {
        uint32_t tNow = millis();
        if (g_imu.ageMs(tNow) > 500u || !g_imu.fresh()) {
            Serial.printf("[GO] refusing: IMU dead (age=%u fresh=%d)\n",
                          (unsigned)g_imu.ageMs(tNow), g_imu.fresh() ? 1 : 0);
            return false;
        }
    }
    // Диагностика перед запуском — что может блокировать движение.
    const auto& e = g_est.get();
    uint32_t now = millis();
    Serial.printf("[GO] precheck: sol=%d hAcc=%.3f imuYaw=%.1f mag=%d acc=%.2f "
                  "imuAge=%u pvtAge=%u rtcmAge=%lu motFB=%d "
                  "wsConn=%d navReq=%d safety=%d reason=%s\n",
                  (int)e.sol, e.hAcc, g_imu.yawDeg(),
                  g_imu.yawFromMag() ? 1 : 0, g_imu.yawAccRad(),
                  g_imu.ageMs(now), e.pvtAgeMs,
                  g_rtcm.transportAgeMs(now),
                  g_motor.haveFeedback() ? 1 : 0,
                  g_ws.isConnected() ? 1 : 0,
                  g_ws.navRequested() ? 1 : 0,
                  (int)g_safety.level(), g_safety.reason());

    float cur = g_imu.yawDeg();
    g_est.seedHeadingDeg(0.0f);
    g_follow.reset();
    g_route.beginUpload(2, g_est.get().originLat, g_est.get().originLon);
    g_route.addWaypoint(0, 0.0f, 0.0f);
    g_route.addWaypoint(1, 0.0f, 3.0f);
    g_route.beginBoundary(4);
    g_route.addBoundaryPoint(0, -2.0f, -2.0f);
    g_route.addBoundaryPoint(1,  2.0f, -2.0f);
    g_route.addBoundaryPoint(2,  2.0f,  5.0f);
    g_route.addBoundaryPoint(3, -2.0f,  5.0f);
    g_route.endBoundary();
    g_route.beginForbidden(0, nullptr);
    g_route.endForbidden();
    g_route.endUpload();
    g_route.start();
    Serial.printf("[GO] CAL+route+start: imuYaw was %.1f → head=0, "
                  "route: (0,0) → (0,3), speed=%.2f m/s\n",
                  (double)cur, (double)ROVER_MAX_SPEED_MPS);
    return true;
}

void setLogEnabled(bool enabled) {
    g_logEnabled = enabled;
    Serial.printf("[LOG] periodic log %s\n", enabled ? "ON (200ms)" : "OFF");
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
             "IMU_ZERO,yaw=%.1f,raw=%.1f,game=%.1f,rot=%.1f,geo=%.1f,"
             "mag=%.2f/%.2f/%.2f,gint=%.1f/%.1f/%.1f,cnt=%lu/%lu/%lu/%lu/%lu",
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
             "dYaw=%.1f,dRaw=%.1f,dGame=%.1f,dRot=%.1f,dGeo=%.1f,"
             "dGx=%.1f,dGy=%.1f,dGz=%.1f,"
             "dMxy=%.1f,dMyx=%.1f,dMxz=%.1f,dMzx=%.1f,dMyz=%.1f,dMzy=%.1f,"
             "rate=%.1f/%.1f/%.1f,"
             "age=%u,cnt=%lu/%lu/%lu/%lu/%lu,"
             "nowYaw=%.1f,raw=%.1f,game=%.1f,rot=%.1f,geo=%.1f,mag=%.2f/%.2f/%.2f",
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

}  // namespace roverdbg
