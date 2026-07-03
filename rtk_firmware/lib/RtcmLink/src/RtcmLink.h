// RtcmLink.h - UDP RTCM transport. MIT.

#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include "Gnss.h"

enum RtcmSource : uint8_t { RTCM_NONE = 0, RTCM_UDP = 2 };
static constexpr uint32_t RTCM_AGE_UNKNOWN_MS = 0xFFFFFFFFu;

class RtcmLink {
public:
    // baseIp и tcpPort игнорируются (TCP убран), сигнатура сохранена для rover.cpp
    void begin(Gnss& gnss, const char* baseIp, uint16_t tcpPort, uint16_t udpPort);
    void loop();

    RtcmSource source() const { return _src; }
    uint32_t transportAgeMs(uint32_t nowMs) const {
        if (_lastRxMs == 0) return RTCM_AGE_UNKNOWN_MS;
        if (nowMs < _lastRxMs) return 0;
        return nowMs - _lastRxMs;
    }
    uint32_t udpRestarts()  const { return _udpRestarts; }
    uint32_t bytes()        const { return _bytes; }
    uint32_t packets()      const { return _pkts; }

private:
    Gnss*    _gnss    = nullptr;
    WiFiUDP  _udp;
    uint16_t _udpPort = 2101;

    RtcmSource _src      = RTCM_NONE;
    uint32_t   _lastRxMs = 0;
    uint32_t   _udpRestarts = 0;
    uint32_t   _bytes    = 0;
    uint32_t   _pkts     = 0;
};
