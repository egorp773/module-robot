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
    }
} g_follow;

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
    const bool rtkOk =
        (e.sol == SOL_FIXED && e.hAcc <= SAFE_HACC_FIXED_M) ||
        (e.sol == SOL_FLOAT && e.hAcc <= 0.05f);
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
    Serial.printf("[%s] distance=%.2f heading=%.1f start=(%.2f,%.2f) "
                  "target=(%.2f,%.2f) origin=(%.8f,%.8f) speed=%.2f\n",
                  tag, (double)distanceM, (double)headingDeg,
                  (double)startX, (double)startY,
                  (double)targetX, (double)targetY,
                  e.originLat, e.originLon,
                  (double)ROVER_V2_FORWARD_MPS);
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

    bool rtkOk = (e.sol == SOL_FIXED && e.hAcc <= SAFE_HACC_FIXED_M) ||
                 (e.sol == SOL_FLOAT && e.hAcc <= 0.05f);
    if (!rtkOk || e.pvtAgeMs > SAFE_PVT_AGE_MS ||
        e.acceptedPositionAgeMs > SAFE_ACCEPTED_POS_AGE_MS) {
        g_follow.linearMps = 0;
        g_follow.angularRadps = 0;
        g_motor.stopImmediately();
        return;
    }
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

    const Waypoint& wp = g_route.waypoint(g_follow.wpIdx);
    float dx = wp.p.x - current.x;
    float dy = wp.p.y - current.y;
    float distToWp = sqrtf(dx * dx + dy * dy);

    g_follow.targetX = wp.p.x;
    g_follow.targetY = wp.p.y;
    g_follow.distToTarget = distToWp;
    g_follow.crossTrack = 0;

    if (distToWp < ROVER_V2_ARRIVAL_RADIUS_M) {
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
        const Waypoint& nextWp = g_route.waypoint(g_follow.wpIdx);
        dx = nextWp.p.x - current.x;
        dy = nextWp.p.y - current.y;
        distToWp = sqrtf(dx * dx + dy * dy);
        g_follow.targetX = nextWp.p.x;
        g_follow.targetY = nextWp.p.y;
        g_follow.distToTarget = distToWp;
    }

    NavPoint target{g_follow.targetX, g_follow.targetY};
    if (!g_route.segmentAllowed(current, target, ROVER_BOUNDARY_TOLERANCE_M, ROVER_BOUNDARY_SAMPLE_M)) {
        followerFault("route_segment_blocked");
        return;
    }

    float desiredHeading = NavMath::targetHeadingDeg(dx, dy);
    float headingErr = wrapDeg180Local(desiredHeading - e.headingFiltDeg);
    float absErr = fabsf(headingErr);

    g_follow.targetHeadingDeg = desiredHeading;
    g_follow.headingErr = headingErr;

    float linear = 0.0f;
    float angular = 0.0f;
    if (absErr > ROVER_V2_TURN_IN_PLACE_DEG) {
        angular = (headingErr >= 0.0f ? 1.0f : -1.0f) * ROVER_V2_TURN_RADPS;
        const uint32_t now = millis();
        const float commandSign = angular >= 0.0f ? 1.0f : -1.0f;
        if (!g_follow.turnWatchActive ||
            commandSign != g_follow.turnWatchCommandSign) {
            g_follow.turnWatchActive = true;
            g_follow.turnWatchHeadingDeg = e.headingFiltDeg;
            g_follow.turnWatchAbsErrorDeg = absErr;
            g_follow.turnWatchCommandSign = commandSign;
            g_follow.turnWatchSinceMs = now;
        } else if ((now - g_follow.turnWatchSinceMs) >
                   ROVER_V2_TURN_WATCHDOG_MS) {
            const float signedHeadingDelta =
                wrapDeg180Local(e.headingFiltDeg -
                                g_follow.turnWatchHeadingDeg);
            const bool directionOk =
                signedHeadingDelta * commandSign >=
                ROVER_V2_TURN_MIN_DELTA_DEG;
            const bool errorImproved =
                absErr <= g_follow.turnWatchAbsErrorDeg -
                          ROVER_V2_TURN_MIN_ERROR_IMPROVE_DEG;
            if (!directionOk || !errorImproved) {
                followerFault(!directionOk ? "turn_wrong_direction_or_stuck"
                                           : "turn_not_converging");
                return;
            }
            g_follow.turnWatchHeadingDeg = e.headingFiltDeg;
            g_follow.turnWatchAbsErrorDeg = absErr;
            g_follow.turnWatchSinceMs = now;
        }
    } else {
        g_follow.turnWatchActive = false;
        linear = (g_safety.level() == SAFETY_DEGRADED) ? ROVER_FLOAT_SPEED : ROVER_V2_FORWARD_MPS;
        angular = clampf(ROVER_V2_HEADING_KP_RADPS_PER_DEG * headingErr,
                         -ROVER_V2_MAX_CORRECTION_RADPS,
                         ROVER_V2_MAX_CORRECTION_RADPS);
    }

    g_follow.linearMps = linear;
    g_follow.angularRadps = angular;

    if (!checkFollowerProgress(e, linear)) return;

    if (g_ws.navRequested() && g_route.isRunning() && g_safety.allowMotion()) {
        g_motor.setLinearAngularSpeed(linear, angular, true);
    } else {
        g_follow.linearMps = 0;
        g_follow.angularRadps = 0;
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
            } else if (strcmp(buf, "IMU_CAL_START") == 0) {
                Serial.println(roverdbg::imuCalStartLine());
            } else if (strcmp(buf, "IMU_CAL_SAVE") == 0) {
                Serial.println(roverdbg::imuCalSaveLine());
            } else if (strcmp(buf, "IMU_CAL_CLEAR") == 0) {
                Serial.println(roverdbg::imuCalClearLine());
            } else if (strcmp(buf, "IMU_TARE_YAW") == 0) {
                Serial.println(roverdbg::imuTareYawLine());
            } else if (strcmp(buf, "IMU_TARE_PERSIST") == 0) {
                Serial.println(roverdbg::imuTarePersistLine());
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
                g_imu.yawDeg(), g_imu.yawAbsoluteValid(),
                g_imu.yawAccRad(), g_imu.yawSource(), g_imu.yawIsAbsolute());
    // Hoverboard feedback → EKF predict. TX-задача на ядре 0 обновляет _fb в Motor;
    // здесь читаем актуальные обороты. Если feedback ещё не пришёл, шлёт 0/0.
    g_est.onHoverboardFeedback(now, g_motor.speedLeftMeas(), g_motor.speedRightMeas());
    g_est.tick(now);

    // Operational origin is supplied by ROUTE_BEGIN from the saved map.
    // Current-position auto-origin exists only inside explicit GO_* test commands.

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
                      "safety=%d reason=%s fault=%s\n",
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

bool handleCal() {
    if (!g_imu.yawAbsoluteValid() || g_imu.headingState() != ImuHeadingState::IMU_ABSOLUTE_OK) {
        Serial.printf("[CAL] refusing: absolute heading unavailable state=%d source=%d acc=%.2f\n",
                      (int)g_imu.headingState(), (int)g_imu.yawSource(), g_imu.yawAccRad());
        return false;
    }
    float cur = g_imu.yawDeg();
    g_est.seedHeadingDeg(cur, g_imu.yawSource());
    g_follow.reset();
    Serial.printf("[CAL] estimator reseeded from absolute imuYaw=%.1f source=%d\n",
                  (double)cur, (int)g_imu.yawSource());
    return true;
}

bool handleGo() {
    return handleGoForward(ROVER_GO_DEFAULT_DISTANCE_M);
}

static bool goPrecheck() {
    // Жёсткий гейт: если IMU не отвечает дольше 500мс — отказываем.
    // Иначе heading "застрянет" на старом значении, Stanley поведёт не туда,
    // и робот развернётся при старте. Это страховка от I2C-зависания BNO085.
    uint32_t tNow = millis();
    if (g_imu.ageMs(tNow) > SAFE_IMU_AGE_MS || !g_imu.fresh()) {
        Serial.printf("[GO] refusing: IMU dead (age=%u fresh=%d)\n",
                      (unsigned)g_imu.ageMs(tNow), g_imu.fresh() ? 1 : 0);
        return false;
    }
    if (!ImuMath::canUseAbsoluteYawForNav(
            g_imu.headingState(),
            g_imu.yawAbsoluteValid(),
            g_imu.yawAccRad(),
            g_imu.yawAgeMs(tNow),
            SAFE_IMU_AGE_MS)) {
        Serial.printf("[GO] refusing: absolute yaw unavailable state=%d source=%d acc=%.2f\n",
                      (int)g_imu.headingState(), (int)g_imu.yawSource(), g_imu.yawAccRad());
        return false;
    }
    if (!g_est.get().headingValid) {
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

bool handleGoForward(float distanceM) {
    if (!goPrecheck()) return false;
    return startGoRoute("GO_FORWARD", distanceM, true);
}

bool handleGoNorth(float distanceM) {
    if (!goPrecheck()) return false;
    return startGoRoute("GO_NORTH", distanceM, false);
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
    String err;
    if (g_imu.startCalibration(&err)) {
        return String("IMU_CAL_START,OK");
    }
    return String("IMU_CAL_START,ERR,") + err;
}

String imuCalSaveLine() {
    String err;
    if (g_imu.saveCalibration(&err)) {
        return String("IMU_CAL_SAVE,OK");
    }
    return String("IMU_CAL_SAVE,ERR,") + err;
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

}  // namespace roverdbg
