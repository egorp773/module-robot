// WsServer.cpp - WebSocket + протокол. MIT.

#include "WsServer.h"
#include "RtkConfig.h"
#include "RoverDebug.h"

void WsServer::begin(StateEstimator& est, Imu& imu, Gnss& gnss, RtcmLink& rtcm,
                     Route& route, Motor& motor, Safety& safety, uint16_t port) {
    _est = &est; _imu = &imu; _gnss = &gnss; _rtcm = &rtcm;
    _route = &route; _motor = &motor; _safety = &safety;

    _server = new AsyncWebServer(port);
    _ws = new AsyncWebSocket("/ws");

    // Включаем TCP keepalive на WiFi-уровне (см. connectWiFi), но дополнительно шлём
    // пинг клиентам вручную каждые 3 сек. Без этого NAT роутера прибивает сессию.
    // В ESPAsyncWebServer 3.11 нет keepAlivePeriod — пингуем через pingAll() в loop.

    _server->on("/ping", HTTP_GET, [](AsyncWebServerRequest* r) { r->send(200, "text/plain", "OK"); });

    _ws->onEvent([this](AsyncWebSocket* s, AsyncWebSocketClient* c, AwsEventType t, void* arg, uint8_t* d, size_t l){
        this->onWsEvent(s, c, t, arg, d, l);
    });
    _server->addHandler(_ws);
    _server->begin();
}

void WsServer::onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                         AwsEventType type, void* arg, uint8_t* data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        _connected = true;
        _client = client;
        _lastRxMs = millis();
        sendText(client, "STATE,CONNECTED");
        // Подсказка оператору — какие команды есть для отладки.
        sendText(client, "[HELP] DIAG | IMU_STATUS | IMU_ZERO | IMU_DIAG | CAL(=IMU_CAL_START) | CAL_HEADING_SEED | IMU_CAL_START | IMU_CAL_SAVE | IMU_CAL_STATUS | IMU_CAL_CLEAR | IMU_TARE_YAW | IMU_TARE_PERSIST | AUTO_ALIGN_HEADING_BY_RTK | HEADING_STATUS | CLEAR_HEADING_TRUST | NAV_START_AUTO_ALIGN | GO_FORWARD[,m] | GO_NORTH[,m] | STOP | LOG,0 | LOG,1");
    } else if (type == WS_EVT_DISCONNECT) {
        if (_client == client) _client = nullptr;
        stopActuators();
        _motor->stopImmediately();
        _route->stop();
        _navRequested = false;
        // узнаем, остались ли ещё клиенты
        if (server->count() == 0) {
            _connected = false;
            _motor->stopImmediately();
            _route->stop();
            _navRequested = false;
        }
    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo* info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len) {
            if (len > 255) {
                sendText(client, "ERR,LINE_TOO_LONG");
                return;
            }
            char lineBuf[256];
            memcpy(lineBuf, data, len);
            lineBuf[len] = 0;
            String s(lineBuf);
            s.trim();
            _lastRxMs = millis();
            handleLine(client, s);
        }
    }
}

void WsServer::sendText(AsyncWebSocketClient* client, const String& text) {
    if (client) {
        client->text(text);
    }
}
void WsServer::sendAll(const String& text) {
    if (_ws) _ws->textAll(text);
}

bool WsServer::trySendTelemetryText(AsyncWebSocketClient* client, const String& text) {
    if (!client) return false;
    if (client->status() != WS_CONNECTED) {
        _wsTelemetryDropped++;
        _lastWsDropMs = millis();
        return false;
    }
    if (client->queueIsFull()) {
        // Quietly drop telemetry. We do not log per-drop to avoid
        // flooding the Serial monitor when the client is slow; the
        // counter is the only persistent record.
        _wsTelemetryDropped++;
        _lastWsDropMs = millis();
        return false;
    }
    client->text(text);
    _wsTelemetrySent++;
    return true;
}

void WsServer::stopActuators() {
    digitalWrite(PIN_RELAY_ATTACH, LOW);
    digitalWrite(PIN_RELAY_MOUNT, LOW);
}

void WsServer::handleLine(AsyncWebSocketClient* client, const String& line) {
    _lastCmdMs = millis();

    // === Отладочные команды из приложения (через WebSocket).
    // Без этого "GO"/"CAL"/"LOG" из приложения падают в ERR,UNKNOWN. ===
    if (line == "CAL") {
        // Legacy alias: route to BNO085 dynamic calibration (not heading
        // seed). Calibration must work from ABSOLUTE_UNCALIBRATED.
        sendText(client, roverdbg::imuCalStartLine());
        return;
    }
    if (line == "CAL_HEADING_SEED") {
        sendText(client, roverdbg::handleHeadingSeed() ? "OK,CAL_HEADING_SEED" : "ERR,CAL_HEADING_SEED");
        return;
    }
    if (line == "IMU_STATUS") {
        sendText(client, roverdbg::imuStatusLine());
        return;
    }
    if (line == "IMU_CAL_START") {
        sendText(client, roverdbg::imuCalStartLine());
        return;
    }
    if (line == "IMU_CAL_SAVE") {
        sendText(client, roverdbg::imuCalSaveLine());
        return;
    }
    if (line == "IMU_CAL_STATUS") {
        sendText(client, roverdbg::imuCalStatusLine());
        return;
    }
    if (line == "IMU_CAL_CLEAR") {
        sendText(client, roverdbg::imuCalClearLine());
        return;
    }
    if (line == "IMU_TARE_YAW") {
        sendText(client, roverdbg::imuTareYawLine());
        return;
    }
    if (line == "IMU_TARE_PERSIST") {
        sendText(client, roverdbg::imuTarePersistLine());
        return;
    }
    if (line.startsWith("IMU_SET_TRUE_HEADING,")) {
        float deg = 0;
        if (sscanf(line.c_str(), "IMU_SET_TRUE_HEADING,%f", &deg) == 1) {
            sendText(client, roverdbg::imuSetTrueHeadingLine(deg));
        } else {
            sendText(client, "ERR,IMU_SET_TRUE_HEADING_FORMAT");
        }
        return;
    }
    if (line == "IMU_CLEAR_HEADING_CORRECTION") {
        sendText(client, roverdbg::imuClearHeadingCorrectionLine());
        return;
    }
    if (line == "IMU_HEADING_TEST") {
        sendText(client, roverdbg::imuHeadingTestLine());
        return;
    }
    if (line == "IMU_TRUST_CURRENT_HEADING_ONCE") {
        sendText(client, roverdbg::imuTrustCurrentHeadingOnceLine());
        return;
    }
    if (line == "IMU_CLEAR_MANUAL_HEADING_TRUST") {
        sendText(client, roverdbg::imuClearManualHeadingTrustLine());
        return;
    }
    if (line == "AUTO_ALIGN_HEADING_BY_RTK") {
        sendText(client, roverdbg::autoAlignHeadingByRtkLineWs());
        return;
    }
    if (line == "HEADING_STATUS") {
        sendText(client, roverdbg::headingStatusLine());
        return;
    }
    if (line == "CLEAR_HEADING_TRUST") {
        sendText(client, roverdbg::clearHeadingTrustLine());
        return;
    }
    if (line == "NAV_START_AUTO_ALIGN") {
        sendText(client, roverdbg::navStartAutoAlignLineWs());
        return;
    }
    if (line == "GO" || line.startsWith("GO_FORWARD")) {
        float distance = ROVER_GO_DEFAULT_DISTANCE_M;
        if (line.startsWith("GO_FORWARD,")) {
            distance = atof(line.c_str() + 11);
        }
        if (roverdbg::handleGoForward(distance)) {
            // handleGo() только готовит маршрут, но stepFollower() в loop() крутится
            // только если navRequested==true. Без этого — маршрут запущен, моторы стоят.
            _navRequested = true;
            sendText(client, "OK,GO_FORWARD: " + roverdbg::diagLine());
        } else {
            sendText(client, "ERR,GO_FORWARD: " + roverdbg::diagLine());
        }
        return;
    }
    if (line == "GO_NORTH" || line.startsWith("GO_NORTH,")) {
        float distance = ROVER_GO_DEFAULT_DISTANCE_M;
        if (line.startsWith("GO_NORTH,")) {
            distance = atof(line.c_str() + 9);
        }
        if (roverdbg::handleGoNorth(distance)) {
            _navRequested = true;
            sendText(client, "OK,GO_NORTH: " + roverdbg::diagLine());
        } else {
            sendText(client, "ERR,GO_NORTH: " + roverdbg::diagLine());
        }
        return;
    }
    if (line == "DIAG") {
        sendText(client, roverdbg::diagLine());
        return;
    }
    if (line == "IMU_ZERO") {
        sendText(client, roverdbg::imuZeroLine());
        return;
    }
    if (line == "IMU_DIAG") {
        sendText(client, roverdbg::imuDiagLine());
        return;
    }
    if (line.startsWith("LOG,")) {
        int v = atoi(line.c_str() + 4);
        roverdbg::setLogEnabled(v != 0);
        // ВАЖНО: телеметрия не трогается здесь. По умолчанию OFF (в h).
        // Если хочешь видеть TEL/NAV/IMU — отдельная кнопка LOG,1.
        sendText(client, String("[LOG] ") + (v ? "ON" : "OFF"));
        return;
    }

    if (line == "PING") {
        sendText(client, "PONG");
        sendTel(client);
        sendNav(client);
        return;
    }
    if (line == "STOP") {
        // Single source of truth: roverdbg::handleStopLine() also aborts
        // any running AUTO_ALIGN_HEADING_BY_RTK and logs the abort.
        sendText(client, roverdbg::handleStopLine());
        _navRequested = false;
        return;
    }
    if (line.startsWith("SET_HEADING,")) {
        float heading = 0;
        if (sscanf(line.c_str(), "SET_HEADING,%f", &heading) == 1) {
            _est->seedHeadingDeg(heading);
            sendText(client, "OK,HEADING_MANUAL_ESTIMATOR_ONLY");
        } else {
            sendText(client, "ERR,SET_HEADING");
        }
        return;
    }
    if (line == "ATTACHMENT_ON") {
        digitalWrite(PIN_RELAY_ATTACH, HIGH);
        sendText(client, "OK ATTACHMENT_ON");
        return;
    }
    if (line == "ATTACHMENT_OFF") {
        digitalWrite(PIN_RELAY_ATTACH, LOW);
        sendText(client, "OK ATTACHMENT_OFF");
        return;
    }
    if (line == "MOUNT_ON") {
        digitalWrite(PIN_RELAY_MOUNT, HIGH);
        sendText(client, "OK MOUNT_ON");
        return;
    }
    if (line == "MOUNT_OFF") {
        digitalWrite(PIN_RELAY_MOUNT, LOW);
        sendText(client, "OK MOUNT_OFF");
        return;
    }

    // M,left,right
    if (line.startsWith("M,")) {
        int left = 0, right = 0;
        if (sscanf(line.c_str(), "M,%d,%d", &left, &right) == 2) {
            left  /= ROVER_INPUT_DIV;
            right /= ROVER_INPUT_DIV;
            // в навигации — игнорим ручной ввод
            if (!_navRequested) {
                _motor->setManualPercent(left, right);
            }
            sendText(client, "OK M");
        } else {
            sendText(client, "ERR,M_FORMAT");
        }
        return;
    }

    // ROUTE_BEGIN,count,lat,lon
    if (line.startsWith("ROUTE_BEGIN,")) {
        int count = 0;
        double lat = 0, lon = 0;
        if (sscanf(line.c_str(), "ROUTE_BEGIN,%d,%lf,%lf", &count, &lat, &lon) == 3) {
            if (_navRequested || _route->isRunning()) {
                sendText(client, "ERR,NAV_ACTIVE");
                return;
            }
            const bool originValid =
                isfinite(lat) && isfinite(lon) &&
                lat >= -90.0 && lat <= 90.0 &&
                lon >= -180.0 && lon <= 180.0 &&
                !(lat == 0.0 && lon == 0.0);
            if (originValid && _route->beginUpload(count, lat, lon) &&
                _est->setOrigin(lat, lon)) {
                const auto& e = _est->get();
                Serial.printf("[MAP-ORIGIN] mapOriginLat=%.8f mapOriginLon=%.8f "
                              "currentRobotX=%.3f currentRobotY=%.3f\n",
                              e.originLat, e.originLon, e.x, e.y);
                sendText(client, "OK,ROUTE_BEGIN");
            } else {
                sendText(client, "ERR,ROUTE_BEGIN");
            }
        } else {
            sendText(client, "ERR,ROUTE_BEGIN");
        }
        return;
    }
    if (line.startsWith("ROUTE_WP,")) {
        int idx = 0;
        float x = 0, y = 0;
        if (sscanf(line.c_str(), "ROUTE_WP,%d,%f,%f", &idx, &x, &y) == 3) {
            if (_route->addWaypoint(idx, x, y)) {
                sendText(client, "OK,ROUTE_WP," + String(idx));
            } else {
                sendText(client, "ERR,ROUTE_WP");
            }
        } else {
            sendText(client, "ERR,ROUTE_WP_FORMAT");
        }
        return;
    }
    if (line.startsWith("ROUTE_BOUNDARY_BEGIN,")) {
        int count = 0;
        if (sscanf(line.c_str(), "ROUTE_BOUNDARY_BEGIN,%d", &count) == 1) {
            sendText(client, _route->beginBoundary(count) ? "OK,ROUTE_BOUNDARY_BEGIN" : "ERR,ROUTE_BOUNDARY_BEGIN");
        } else {
            sendText(client, "ERR,ROUTE_BOUNDARY_BEGIN");
        }
        return;
    }
    if (line.startsWith("ROUTE_BOUNDARY_PT,")) {
        int idx = 0;
        float x = 0, y = 0;
        if (sscanf(line.c_str(), "ROUTE_BOUNDARY_PT,%d,%f,%f", &idx, &x, &y) == 3) {
            sendText(client, _route->addBoundaryPoint(idx, x, y) ? "OK,ROUTE_BOUNDARY_PT," + String(idx) : "ERR,ROUTE_BOUNDARY_PT");
        } else {
            sendText(client, "ERR,ROUTE_BOUNDARY_PT_FORMAT");
        }
        return;
    }
    if (line == "ROUTE_BOUNDARY_END") {
        sendText(client, _route->endBoundary() ? "OK,ROUTE_BOUNDARY_END" : "ERR,ROUTE_BOUNDARY_END");
        return;
    }
    if (line.startsWith("FORBID_BEGIN,")) {
        int counts[NavCore::MAX_OBSTACLES] = {0};
        char buf[256];
        line.toCharArray(buf, sizeof(buf));
        char* ctx = nullptr;
        char* tok = strtok_r(buf, ",", &ctx);
        tok = strtok_r(nullptr, ",", &ctx);
        int count = tok ? atoi(tok) : -1;
        bool ok = count >= 0 && count <= NavCore::MAX_OBSTACLES;
        for (int i = 0; ok && i < count; ++i) {
            tok = strtok_r(nullptr, ",", &ctx);
            if (!tok) {
                ok = false;
                break;
            }
            counts[i] = atoi(tok);
            if (counts[i] < 3 || counts[i] > NavCore::MAX_OBSTACLE_POINTS) {
                ok = false;
            }
        }
        if (ok && strtok_r(nullptr, ",", &ctx) != nullptr) ok = false;
        if (ok) {
            sendText(client, _route->beginForbidden(count, counts) ? "OK,FORBID_BEGIN" : "ERR,FORBID_BEGIN");
        } else {
            sendText(client, "ERR,FORBID_BEGIN");
        }
        return;
    }
    if (line.startsWith("FORBID_PT,")) {
        int polyIdx = 0, ptIdx = 0;
        float x = 0, y = 0;
        if (sscanf(line.c_str(), "FORBID_PT,%d,%d,%f,%f", &polyIdx, &ptIdx, &x, &y) == 4) {
            sendText(client, _route->addForbiddenPoint(polyIdx, ptIdx, x, y) ? "OK,FORBID_PT," + String(polyIdx) + "," + String(ptIdx) : "ERR,FORBID_PT");
        } else {
            sendText(client, "ERR,FORBID_PT_FORMAT");
        }
        return;
    }
    if (line == "FORBID_END") {
        sendText(client, _route->endForbidden() ? "OK,FORBID_END" : "ERR,FORBID_END");
        return;
    }
    if (line == "ROUTE_END") {
        _route->endUpload();
        if (_route->isReady()) {
            sendText(client, "OK,ROUTE," + String(_route->count()));
        } else {
            sendText(client, "ERR,ROUTE_INCOMPLETE");
        }
        return;
    }
    if (line == "NAV_START") {
        // Single source of truth for the gate: roverdbg::handleNavStartLine().
        // It honours RTK + estimator heading trust, and the resulting
        // navigation works whether heading is sourced from IMU absolute
        // OK, manual trust, or RTK-motion alignment.
        const String reply = roverdbg::handleNavStartLine();
        sendText(client, reply);
        if (reply.startsWith("OK,NAV_START")) {
            _navRequested = true;
        }
        return;
    }
    if (line == "NAV_PAUSE")  { _route->pause();  sendText(client, "OK,NAV_PAUSE"); return; }
    if (line == "NAV_RESUME") { _route->resume(); sendText(client, "OK,NAV_RESUME"); return; }
    if (line == "NAV_STOP") {
        stopActuators();
        _route->stop();
        _navRequested = false;
        _motor->stopImmediately();
        sendText(client, "OK,NAV_STOP");
        return;
    }

    sendText(client, "ERR,UNKNOWN");
}

static const char* carrierText(SolType sol) {
    if (sol == SOL_FIXED) return "fixed";
    if (sol == SOL_FLOAT) return "float";
    return "none";
}

static const char* rtcmSourceText(RtcmSource source) {
    if (source == RTCM_UDP) return "udp";
    return "none";
}

static long telemetryAgeMs(uint32_t ageMs) {
    constexpr uint32_t kMaxTelemetryAgeMs = 60000u;
    if (ageMs == RTCM_AGE_UNKNOWN_MS || ageMs > kMaxTelemetryAgeMs) return -1L;
    return (long)ageMs;
}

static long rtcmTransportAgeForTelemetry(const RtcmLink* rtcm, uint32_t nowMs) {
    if (!rtcm || rtcm->source() == RTCM_NONE) return -1L;
    return telemetryAgeMs(rtcm->transportAgeMs(nowMs));
}

static long gnssRtcmAgeForTelemetry(const Gnss* gnss, uint32_t nowMs) {
    if (!gnss || gnss->rtcm().lastRxMs == 0) return -1L;
    uint32_t ageMs = (nowMs < gnss->rtcm().lastRxMs) ? 0u : nowMs - gnss->rtcm().lastRxMs;
    return telemetryAgeMs(ageMs);
}

void WsServer::sendTel(AsyncWebSocketClient* client) {
    if (!_est) return;
    const auto& e = _est->get();
    uint32_t now = millis();
    char buf[360];
    int n = snprintf(buf, sizeof(buf),
        "TEL,%.7f,%.7f,%.2f,%.1f,%d,%s,%d,%d,%d,%d,%.3f,%.2f,%u,%lu,%ld,%.2f,%lu,%d,%ld,%ld,%s,%lu,%lu,%lu",
        e.lat, e.lon, e.alt, e.headingFiltDeg, e.fixType, carrierText(e.sol), e.diff ? 1 : 0,
        e.numSv, (int)(e.hAcc*1000), (int)(e.vAcc*1000), e.speedMps, e.pDop,
        e.pvtAgeMs,
        (unsigned long)(_rtcm ? _rtcm->bytes() : 0u),
        rtcmTransportAgeForTelemetry(_rtcm, now),
        _imu ? _imu->yawDeg() : 0.0f,
        (unsigned long)(_imu ? _imu->ageMs(now) : RTCM_AGE_UNKNOWN_MS),
        _imu && _imu->fresh() ? 1 : 0,
        rtcmTransportAgeForTelemetry(_rtcm, now),
        gnssRtcmAgeForTelemetry(_gnss, now),
        rtcmSourceText(_rtcm ? _rtcm->source() : RTCM_NONE),
        (unsigned long)(_gnss ? _gnss->rtcm().msgCount : 0u),
        (unsigned long)(_gnss ? _gnss->rtcm().crcFail  : 0u),
        (unsigned long)(_gnss ? _gnss->rtcm().lastType : 0u));
    if (n > 0) sendText(client, String(buf));
}

void WsServer::sendGps(AsyncWebSocketClient* client) {
    if (!_est) return;
    const auto& e = _est->get();
    char buf[200];
    snprintf(buf, sizeof(buf),
        "GPS,%.7f,%.7f,%.1f,%d,%d",
        e.lat, e.lon, e.headingFiltDeg, e.fixType, (int)(e.hAcc*1000));
    sendText(client, buf);
}

void WsServer::sendGpsDbg(AsyncWebSocketClient* client) {
    if (!_est) return;
    const auto& e = _est->get();
    char buf[256];
    snprintf(buf, sizeof(buf),
        "GPSDBG,%.7f,%.7f,%.3f,%.1f,%d,%s,%d,%d,%d,%d,%.3f,%.2f,%u",
        e.lat, e.lon, e.alt, e.headingFiltDeg, e.fixType,
        carrierText(e.sol),
        e.diff ? 1 : 0, e.numSv,
        (int)(e.hAcc*1000), (int)(e.vAcc*1000), e.speedMps, e.pDop, e.pvtAgeMs);
    sendText(client, buf);
}

void WsServer::sendRtcm(AsyncWebSocketClient* client) {
    if (!_rtcm || !_gnss) return;
    uint32_t now = millis();
    char buf[200];
    snprintf(buf, sizeof(buf),
        "RTCM,%lu,%ld,%ld,%ld,%s,%lu,%lu,%lu",
        (unsigned long)_rtcm->bytes(), rtcmTransportAgeForTelemetry(_rtcm, now),
        rtcmTransportAgeForTelemetry(_rtcm, now),
        gnssRtcmAgeForTelemetry(_gnss, now),
        rtcmSourceText(_rtcm->source()),
        (unsigned long)_rtcm->packets(),
        (unsigned long)_gnss->rtcm().crcFail,
        (unsigned long)_gnss->rtcm().lastType);
    sendText(client, buf);
}

void WsServer::sendImu(AsyncWebSocketClient* client) {
    if (!_imu) return;
    char buf[80];
    snprintf(buf, sizeof(buf), "IMU,%.2f,%u,%d",
        _imu->yawDeg(), _imu->ageMs(millis()), _imu->fresh() ? 1 : 0);
    sendText(client, buf);
}

void WsServer::sendNav(AsyncWebSocketClient* client) {
    const char* st = "IDLE";
    switch (_lastNav.state) {
        case NavStateOut::RUNNING: st = "RUNNING"; break;
        case NavStateOut::APPROACHING: st = "APPROACHING"; break;
        case NavStateOut::PAUSED: st = "PAUSED"; break;
        case NavStateOut::ARRIVED: st = "ARRIVED"; break;
        case NavStateOut::ERROR:   st = "ERROR";   break;
        default: st = "IDLE";
    }
    char buf[200];
    snprintf(buf, sizeof(buf), "NAV,%s,%d,%d,%.2f,%.1f,%.2f,%d,%d,%s",
        st, _lastNav.wpIdx, _lastNav.wpTotal, _lastNav.distToWp,
        _lastNav.headingErr, _lastNav.crossTrack,
        _lastNav.lastLeftPwm, _lastNav.lastRightPwm,
        _lastNav.errorReason ? _lastNav.errorReason : "");
    sendText(client, buf);
}

String WsServer::makeMotorLine(const Motor& motor) {
    char buf[128];
    int batCentivolts = motor.haveFeedback() ? (int)lroundf(motor.batteryVolts() * 100.0f) : 0;
    int tempDeciC = motor.haveFeedback() ? (int)lroundf(motor.boardTempC() * 10.0f) : 0;
    snprintf(buf, sizeof(buf), "MOTOR,%d,%d,%d,%d,%d,%d,%d",
        motor.currentLeftPwm(), motor.currentRightPwm(),
        motor.haveFeedback() ? 1 : 0,
        motor.speedLeftMeas(), motor.speedRightMeas(),
        batCentivolts, tempDeciC);
    return String(buf);
}

void WsServer::sendMotor(AsyncWebSocketClient* client) {
    if (_motor) sendText(client, makeMotorLine(*_motor));
}

void WsServer::markNavUpdate(const NavStateOut& s) {
    _lastNav = s;
}

void WsServer::markTelemetryTick() {
    uint32_t now = millis();
    if (now - _lastTelMs < TEL_PERIOD_MS) return;
    _lastTelMs = now;
    // общая телеметрия шлётся через sendAll, чтобы не зависеть от WS-клиента
    if (!_client) return;
    // TEL
    {
        const auto& e = _est->get();
        char buf[360];
        int n = snprintf(buf, sizeof(buf),
            "TEL,%.7f,%.7f,%.2f,%.1f,%d,%s,%d,%d,%d,%d,%.3f,%.2f,%u,%lu,%ld,%.2f,%lu,%d,%ld,%ld,%s,%lu,%lu,%lu",
            e.lat, e.lon, e.alt, e.headingFiltDeg, e.fixType, carrierText(e.sol), e.diff ? 1 : 0,
            e.numSv, (int)(e.hAcc*1000), (int)(e.vAcc*1000), e.speedMps, e.pDop,
            e.pvtAgeMs,
            (unsigned long)(_rtcm ? _rtcm->bytes() : 0u),
            rtcmTransportAgeForTelemetry(_rtcm, now),
            _imu ? _imu->yawDeg() : 0.0f,
            (unsigned long)(_imu ? _imu->ageMs(now) : RTCM_AGE_UNKNOWN_MS),
            _imu && _imu->fresh() ? 1 : 0,
            rtcmTransportAgeForTelemetry(_rtcm, now),
            gnssRtcmAgeForTelemetry(_gnss, now),
            rtcmSourceText(_rtcm ? _rtcm->source() : RTCM_NONE),
            (unsigned long)(_gnss ? _gnss->rtcm().msgCount : 0u),
            (unsigned long)(_gnss ? _gnss->rtcm().crcFail  : 0u),
            (unsigned long)(_gnss ? _gnss->rtcm().lastType : 0u));
        if (n > 0) trySendTelemetryText(_client, String(buf));
    }
    {
        const auto& e = _est->get();
        char buf[256];
        snprintf(buf, sizeof(buf),
            "GPSDBG,%.7f,%.7f,%.3f,%.1f,%d,%s,%d,%d,%d,%d,%.3f,%.2f,%u",
            e.lat, e.lon, e.alt, e.headingFiltDeg, e.fixType,
            carrierText(e.sol),
            e.diff ? 1 : 0, e.numSv,
            (int)(e.hAcc*1000), (int)(e.vAcc*1000), e.speedMps, e.pDop, e.pvtAgeMs);
        trySendTelemetryText(_client, buf);
    }
    if (_rtcm && _gnss) {
        char buf[200];
        snprintf(buf, sizeof(buf),
            "RTCM,%lu,%ld,%ld,%ld,%s,%lu,%lu,%lu",
            (unsigned long)_rtcm->bytes(), rtcmTransportAgeForTelemetry(_rtcm, now),
            rtcmTransportAgeForTelemetry(_rtcm, now),
            gnssRtcmAgeForTelemetry(_gnss, now),
            rtcmSourceText(_rtcm->source()),
            (unsigned long)_rtcm->packets(),
            (unsigned long)_gnss->rtcm().crcFail,
            (unsigned long)_gnss->rtcm().lastType);
        trySendTelemetryText(_client, buf);
    }
    if (_imu) {
        char buf[80];
        snprintf(buf, sizeof(buf), "IMU,%.2f,%u,%d",
            _imu->yawDeg(), _imu->ageMs(now), _imu->fresh() ? 1 : 0);
        trySendTelemetryText(_client, buf);
    }
    if (_motor) {
        trySendTelemetryText(_client, makeMotorLine(*_motor));
    }
    if (now - _lastNavMs >= NAV_PERIOD_MS) {
        const char* st = "IDLE";
        switch (_lastNav.state) {
            case NavStateOut::RUNNING: st = "RUNNING"; break;
            case NavStateOut::APPROACHING: st = "APPROACHING"; break;
            case NavStateOut::PAUSED: st = "PAUSED"; break;
            case NavStateOut::ARRIVED: st = "ARRIVED"; break;
            case NavStateOut::ERROR:   st = "ERROR";   break;
            default: st = "IDLE";
        }
        char buf[200];
        snprintf(buf, sizeof(buf), "NAV,%s,%d,%d,%.2f,%.1f,%.2f,%d,%d,%s",
            st, _lastNav.wpIdx, _lastNav.wpTotal, _lastNav.distToWp,
            _lastNav.headingErr, _lastNav.crossTrack,
            _lastNav.lastLeftPwm, _lastNav.lastRightPwm,
            _lastNav.errorReason ? _lastNav.errorReason : "");
        trySendTelemetryText(_client, buf);
        _lastNavMs = now;
    }
}

void WsServer::loop() {
    if (!_ws) return;
    // Manual WS ping каждые 3 сек. Без этого лужи NAT-сессии через 1-3 минуты рвутся
    // со стороны роутера (Xiaomi агрессивный idle-timeout), и приложение получает
    // реконнект каждые 1-2 мин.
    uint32_t now = millis();
    if (now - _lastPingMs >= 3000) {
        _lastPingMs = now;
        _ws->pingAll();
    }
    _ws->cleanupClients();
}

void WsServer::sendBattery(int pct) {
    char buf[40];
    snprintf(buf, sizeof(buf), "BAT_PCT,%d", pct);
    if (_client) sendText(_client, buf);
}
