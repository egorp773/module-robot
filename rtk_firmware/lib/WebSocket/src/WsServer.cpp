// WsServer.cpp - WebSocket + протокол. MIT.

#include "WsServer.h"
#include "RtkConfig.h"

void WsServer::begin(StateEstimator& est, Imu& imu, Gnss& gnss, RtcmLink& rtcm,
                     Route& route, Motor& motor, Safety& safety, uint16_t port) {
    _est = &est; _imu = &imu; _gnss = &gnss; _rtcm = &rtcm;
    _route = &route; _motor = &motor; _safety = &safety;

    _server = new AsyncWebServer(port);
    _ws = new AsyncWebSocket("/ws");

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

void WsServer::stopActuators() {
    digitalWrite(PIN_RELAY_ATTACH, LOW);
    digitalWrite(PIN_RELAY_MOUNT, LOW);
}

void WsServer::handleLine(AsyncWebSocketClient* client, const String& line) {
    _lastCmdMs = millis();
    if (line == "PING") {
        sendText(client, "PONG");
        sendTel(client);
        sendNav(client);
        return;
    }
    if (line == "STOP") {
        stopActuators();
        _motor->stopImmediately();
        _route->stop();
        _navRequested = false;
        sendText(client, "OK STOP");
        return;
    }
    if (line.startsWith("SET_HEADING,")) {
        float heading = 0;
        if (sscanf(line.c_str(), "SET_HEADING,%f", &heading) == 1) {
            _est->seedHeadingDeg(heading);
            sendText(client, "OK,HEADING");
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
            if (_route->beginUpload(count, lat, lon)) {
                _est->setOrigin(lat, lon);
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
        const auto& e = _est->get();
        uint32_t now = millis();
        if (!_route->isReady()) {
            sendText(client, "ERR,NO_ROUTE");
        } else if (!e.originSet) {
            sendText(client, "ERR,NO_ORIGIN");
        } else if (e.sol != SOL_FIXED) {
            sendText(client, "ERR,RTK_NOT_FIXED");
        } else if (e.hAcc > SAFE_HACC_FIXED_M) {
            sendText(client, "ERR,HACC");
        } else if (e.pvtAgeMs > SAFE_PVT_AGE_MS || e.acceptedPositionAgeMs > SAFE_ACCEPTED_POS_AGE_MS) {
            sendText(client, "ERR,POSITION_STALE");
        } else if (e.rejectedPositionFixes > 0) {
            sendText(client, "ERR,GPS_JUMP");
        } else if (e.headingAgeMs > SAFE_HEADING_AGE_MS) {
            sendText(client, "ERR,HEADING_STALE");
        } else if (_imu && _imu->ageMs(now) > SAFE_IMU_AGE_MS) {
            sendText(client, "ERR,IMU_STALE");
        } else if (_rtcm && _rtcm->transportAgeMs(now) > SAFE_RTK_AGE_MS) {
            sendText(client, "ERR,RTCM_STALE");
        } else if (_motor && !_motor->haveFeedback()) {
            sendText(client, "ERR,MOTOR_NO_FEEDBACK");
        } else {
            _route->start();
            _navRequested = true;
            sendText(client, "OK,NAV_START");
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

void WsServer::sendTel(AsyncWebSocketClient* client) {
    if (!_est) return;
    const auto& e = _est->get();
    uint32_t now = millis();
    char buf[360];
    int n = snprintf(buf, sizeof(buf),
        "TEL,%.7f,%.7f,%.2f,%.1f,%d,%s,%d,%d,%d,%d,%.3f,%.2f,%u,%u,%u,%.2f,%u,%d,%u,%u,%s,%u,%u,%u",
        e.lat, e.lon, e.alt, e.headingFiltDeg, e.fixType, carrierText(e.sol), e.diff ? 1 : 0,
        e.numSv, (int)(e.hAcc*1000), (int)(e.vAcc*1000), e.speedMps, e.pDop,
        e.pvtAgeMs,
        _rtcm ? _rtcm->bytes() : 0u,
        _rtcm ? _rtcm->transportAgeMs(now) : 0xFFFFFFFFu,
        _imu ? _imu->yawDeg() : 0.0f,
        _imu ? _imu->ageMs(now) : 0xFFFFFFFFu,
        _imu && _imu->fresh() ? 1 : 0,
        _rtcm ? _rtcm->transportAgeMs(now) : 0xFFFFFFFFu,
        _gnss ? _gnss->pvtAgeMs(now) : 0xFFFFFFFFu,
        rtcmSourceText(_rtcm ? _rtcm->source() : RTCM_NONE),
        _gnss ? _gnss->rtcm().msgCount : 0u,
        _gnss ? _gnss->rtcm().crcFail  : 0u,
        _gnss ? _gnss->rtcm().lastType : 0u);
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
    char buf[200];
    snprintf(buf, sizeof(buf),
        "RTCM,%u,%u,%u,%u,%s,%u,%u,%u",
        _rtcm->bytes(), _rtcm->transportAgeMs(millis()),
        _rtcm->transportAgeMs(millis()),
        _gnss->pvtAgeMs(millis()),
        rtcmSourceText(_rtcm->source()),
        _rtcm->packets(), _gnss->rtcm().crcFail, _gnss->rtcm().lastType);
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
            "TEL,%.7f,%.7f,%.2f,%.1f,%d,%s,%d,%d,%d,%d,%.3f,%.2f,%u,%u,%u,%.2f,%u,%d,%u,%u,%s,%u,%u,%u",
            e.lat, e.lon, e.alt, e.headingFiltDeg, e.fixType, carrierText(e.sol), e.diff ? 1 : 0,
            e.numSv, (int)(e.hAcc*1000), (int)(e.vAcc*1000), e.speedMps, e.pDop,
            e.pvtAgeMs,
            _rtcm ? _rtcm->bytes() : 0u,
            _rtcm ? _rtcm->transportAgeMs(now) : 0xFFFFFFFFu,
            _imu ? _imu->yawDeg() : 0.0f,
            _imu ? _imu->ageMs(now) : 0xFFFFFFFFu,
            _imu && _imu->fresh() ? 1 : 0,
            _rtcm ? _rtcm->transportAgeMs(now) : 0xFFFFFFFFu,
            _gnss ? _gnss->pvtAgeMs(now) : 0xFFFFFFFFu,
            rtcmSourceText(_rtcm ? _rtcm->source() : RTCM_NONE),
            _gnss ? _gnss->rtcm().msgCount : 0u,
            _gnss ? _gnss->rtcm().crcFail  : 0u,
            _gnss ? _gnss->rtcm().lastType : 0u);
        if (n > 0) sendText(_client, String(buf));
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
        sendText(_client, buf);
    }
    if (_rtcm && _gnss) {
        char buf[200];
        snprintf(buf, sizeof(buf),
            "RTCM,%u,%u,%u,%u,%s,%u,%u,%u",
            _rtcm->bytes(), _rtcm->transportAgeMs(now),
            _rtcm->transportAgeMs(now),
            _gnss->pvtAgeMs(now),
            rtcmSourceText(_rtcm->source()),
            _rtcm->packets(), _gnss->rtcm().crcFail, _gnss->rtcm().lastType);
        sendText(_client, buf);
    }
    if (_imu) {
        char buf[80];
        snprintf(buf, sizeof(buf), "IMU,%.2f,%u,%d",
            _imu->yawDeg(), _imu->ageMs(now), _imu->fresh() ? 1 : 0);
        sendText(_client, buf);
    }
    if (_motor) {
        sendText(_client, makeMotorLine(*_motor));
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
        sendText(_client, buf);
        _lastNavMs = now;
    }
}

void WsServer::loop() {
    if (_ws) _ws->cleanupClients();
}

void WsServer::sendBattery(int pct) {
    char buf[40];
    snprintf(buf, sizeof(buf), "BAT_PCT,%d", pct);
    if (_client) sendText(_client, buf);
}
