// WsServer.h - WebSocket-сервер, парсер протокола, телеметрия. MIT.

#pragma once
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>

#include "StateEstimator.h"
#include "Imu.h"
#include "Gnss.h"
#include "RtcmLink.h"
#include "Route.h"
#include "Motor.h"
#include "Safety.h"

struct NavStateOut {
    enum State : uint8_t { IDLE = 0, RUNNING, APPROACHING, PAUSED, ARRIVED, ERROR } state = IDLE;
    int wpIdx = 0;
    int wpTotal = 0;
    float distToWp = 0;
    float headingErr = 0;
    float crossTrack = 0;
    int lastLeftPwm = 0, lastRightPwm = 0;
    const char* errorReason = nullptr;
};

class WsServer {
public:
    void begin(StateEstimator& est, Imu& imu, Gnss& gnss, RtcmLink& rtcm,
               Route& route, Motor& motor, Safety& safety,
               uint16_t port = 81);

    void loop();
    void markNavUpdate(const NavStateOut& s);
    void markTelemetryTick();

    bool isConnected() const { return _connected; }
    uint32_t lastRxMs() const { return _lastRxMs; }

    // counters / state
    uint32_t lastCmdMs() const { return _lastCmdMs; }
    bool navRequested() const { return _navRequested; }
    void requestDebugNavigation() { _navRequested = true; }
    // Выставляется из roverdbg::handleGo() чтобы stepFollower() начал крутиться —
    // иначе условие в loop() "if (g_ws.navRequested() && g_route.isRunning())"
    // не сработает и моторы не получат команду.
    void setNavRequested(bool v) { _navRequested = v; }
    void setSerialMotionActive(bool active) {
        _serialMotionActive = active;
    }
    // Включает/выключает периодическую телеметрию (TEL/GPS/NAV/IMU/MOTOR).
    // По умолчанию OFF — чтобы терминал не летал при ручной отладке.
    // LOG,1 — on, LOG,0 — off.
    void setTelemetryEnabled(bool v) { _telemetryEnabled = v; }

    // выдача батареи (вызывается извне)
    void sendBattery(int pct);

private:
    void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                   AwsEventType type, void* arg, uint8_t* data, size_t len);
    void handleLine(AsyncWebSocketClient* client, const String& line);
    void sendText(AsyncWebSocketClient* client, const String& text);
    void sendAll(const String& text);
    void sendTel(AsyncWebSocketClient* client);
    void sendGps(AsyncWebSocketClient* client);
    void sendGpsDbg(AsyncWebSocketClient* client);
    void sendRtcm(AsyncWebSocketClient* client);
    void sendImu(AsyncWebSocketClient* client);
    void sendNav(AsyncWebSocketClient* client);
    void sendMotor(AsyncWebSocketClient* client);
    void stopActuators();

    AsyncWebServer* _server = nullptr;
    AsyncWebSocket* _ws = nullptr;
    AsyncWebSocketClient* _client = nullptr;
    StateEstimator* _est = nullptr;
    Imu*  _imu = nullptr;
    Gnss* _gnss = nullptr;
    RtcmLink* _rtcm = nullptr;
    Route* _route = nullptr;
    Motor* _motor = nullptr;
    Safety* _safety = nullptr;

    volatile bool _connected = false;
    volatile uint32_t _lastRxMs = 0;
    volatile uint32_t _lastCmdMs = 0;
    uint32_t _lastTelMs = 0;
    uint32_t _lastNavMs = 0;
    uint32_t _lastPingMs = 0;
    volatile bool _navRequested = false;
    volatile bool _serialMotionActive = false;
    volatile bool _telemetryEnabled = false;
    NavStateOut _lastNav{};

    // WS telemetry backpressure counters. Drop counts are accumulated
    // here, not by AsyncWebServer (which only logs the standard
    // `_queueMessage(): Too many messages queued` warning). Exposed
    // via wsTelemetrySent/Dropped accessors below.
    uint32_t _wsTelemetrySent = 0;
    uint32_t _wsTelemetryDropped = 0;
    uint32_t _lastWsDropMs = 0;

    // Try to enqueue a telemetry frame for the connected client.
    // Returns true if the message was accepted, false if it was dropped
    // because the client queue is full or the client is not connected.
    // Control responses must use sendText() directly so they are never
    // dropped.
    bool trySendTelemetryText(AsyncWebSocketClient* client, const String& text);

    static String makeMotorLine(const Motor& motor);

public:
    // Read-only counters for diagnostics.
    uint32_t wsTelemetrySent() const { return _wsTelemetrySent; }
    uint32_t wsTelemetryDropped() const { return _wsTelemetryDropped; }
    uint32_t lastWsDropMs() const { return _lastWsDropMs; }
};
