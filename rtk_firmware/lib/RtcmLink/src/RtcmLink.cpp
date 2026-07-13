// RtcmLink.cpp - UDP-only RTCM transport. MIT.

#include "RtcmLink.h"
#include "RtkConfig.h"

void RtcmLink::begin(Gnss& gnss, const char* baseIp, uint16_t tcpPort, uint16_t udpPort) {
    _gnss    = &gnss;
    (void)baseIp;
    (void)tcpPort;
    _udpPort = udpPort;
    _udpStarted = false;
    tryStartUdp(millis());
    _src = RTCM_NONE;
}

// БАГФИКС: раньше _udp.begin() вызывался только если WiFi был подключён
// в момент begin(). Если WiFi поднялся позже (таймаут на старте, реконнект) —
// UDP-сокет не открывался НИКОГДА и RTCM был мёртв до ребута.
void RtcmLink::tryStartUdp(uint32_t nowMs) {
    if (_udpStarted) return;
    if (WiFi.status() != WL_CONNECTED) return;
    if (nowMs - _lastUdpTryMs < 1000) return;   // не чаще раза в секунду
    _lastUdpTryMs = nowMs;
    if (_udp.begin(_udpPort)) {
        _udpStarted = true;
        _udpRestarts++;
        Serial.printf("[RTCM] UDP listening :%u\n", (unsigned)_udpPort);
    }
}

void RtcmLink::loop() {
    if (!_gnss) return;
    uint32_t now = millis();
    tryStartUdp(now);
    if (!_udpStarted) return;

    // ---- UDP receive ----
    int packetSize = _udp.parsePacket();
    if (packetSize > 0) {
        uint8_t buf[1024];
        int len = _udp.read(buf, min<int>(packetSize, (int)sizeof(buf)));
        if (len > 0) {
            size_t w = _gnss->feedRtcm(buf, len);
            _bytes   += w;
            _pkts++;
            _lastRxMs = now;
            _src      = RTCM_UDP;
        }
    }

    // Если пакетов давно не было — источник неизвестен
    uint32_t ageMs = transportAgeMs(now);
    if (ageMs == RTCM_AGE_UNKNOWN_MS || ageMs > 5000) {
        _src = RTCM_NONE;
    }
}
