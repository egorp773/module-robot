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

    bool _connected = false;
    uint32_t _lastRxMs = 0;
    uint32_t _lastCmdMs = 0;
    uint32_t _lastTelMs = 0;
    uint32_t _lastNavMs = 0;
    bool _navRequested = false;
    NavStateOut _lastNav{};

    static String makeMotorLine(const Motor& motor);
};
