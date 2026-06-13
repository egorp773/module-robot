// RtcmLink.cpp - UDP-only RTCM transport. MIT.

#include "RtcmLink.h"
#include "RtkConfig.h"

void RtcmLink::begin(Gnss& gnss, const char* baseIp, uint16_t tcpPort, uint16_t udpPort) {
    _gnss    = &gnss;
    (void)baseIp;
    (void)tcpPort;
    _udpPort = udpPort;
    if (WiFi.status() == WL_CONNECTED) {
        _udp.begin(_udpPort);
    }
    _src = RTCM_NONE;
}

void RtcmLink::loop() {
    if (!_gnss) return;
    uint32_t now = millis();

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
    if (_lastRxMs == 0 || (now - _lastRxMs) > 5000) {
        _src = RTCM_NONE;
    }
}
