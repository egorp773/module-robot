// base.cpp - entry point. Base station.

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include "RtkConfig.h"
#include "Gnss.h"

HardwareSerial F9pSerial(1);
Gnss g_gnss;
WiFiUDP    g_udp;

struct RtcmStats {
    uint32_t rawBytes = 0;
    uint32_t bytes = 0;
    uint32_t frames = 0;
    uint32_t udpOk = 0;
    uint32_t udpFail = 0;
    uint32_t dropped = 0;
    uint32_t crcFail = 0;
    uint32_t lastType = 0;
    uint32_t t1005 = 0;
    uint32_t t1074 = 0;
    uint32_t t1084 = 0;
    uint32_t t1094 = 0;
    uint32_t t1124 = 0;
    uint32_t t1230 = 0;
    uint32_t tOther = 0;
} g_stats;

static uint32_t crc24q(const uint8_t* data, size_t len) {
    uint32_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint32_t)data[i] << 16;
        for (uint8_t bit = 0; bit < 8; bit++) {
            crc <<= 1;
            if (crc & 0x1000000UL) crc ^= 0x1864CFBUL;
        }
    }
    return crc & 0xFFFFFFUL;
}

class RtcmFrameFilter {
public:
    bool push(uint8_t b, const uint8_t*& frame, size_t& len) {
        frame = nullptr;
        len = 0;

        switch (_state) {
            case WAIT_D3:
                if (b == 0xD3) {
                    _buf[0] = b;
                    _idx = 1;
                    _state = LEN1;
                } else {
                    g_stats.dropped++;
                }
                break;
            case LEN1:
                if ((b & 0xFC) != 0) {
                    reset();
                    if (b == 0xD3) {
                        _buf[0] = b;
                        _idx = 1;
                        _state = LEN1;
                    } else {
                        g_stats.dropped++;
                    }
                    break;
                }
                _buf[_idx++] = b;
                _lenHi = b;
                _state = LEN2;
                break;
            case LEN2: {
                _buf[_idx++] = b;
                const uint16_t payloadLen = ((uint16_t)(_lenHi & 0x03) << 8) | b;
                _expected = 3 + payloadLen + 3;
                if (payloadLen == 0 || _expected > sizeof(_buf)) {
                    reset();
                    g_stats.dropped++;
                    break;
                }
                _state = BODY;
                break;
            }
            case BODY:
                _buf[_idx++] = b;
                if (_idx >= _expected) {
                    const uint32_t got = ((uint32_t)_buf[_expected - 3] << 16) |
                                         ((uint32_t)_buf[_expected - 2] << 8) |
                                         _buf[_expected - 1];
                    const uint32_t want = crc24q(_buf, _expected - 3);
                    if (got == want) {
                        frame = _buf;
                        len = _expected;
                        reset();
                        return true;
                    }
                    g_stats.crcFail++;
                    reset();
                }
                break;
        }
        return false;
    }

private:
    enum State : uint8_t { WAIT_D3, LEN1, LEN2, BODY };

    void reset() {
        _state = WAIT_D3;
        _idx = 0;
        _expected = 0;
        _lenHi = 0;
    }

    State _state = WAIT_D3;
    uint8_t _buf[1024]{};
    size_t _idx = 0;
    size_t _expected = 0;
    uint8_t _lenHi = 0;
};

RtcmFrameFilter g_rtcmFilter;

static void enableBaseRtcmMessages() {
    g_gnss.ubx().enableRTCMmessage(UBX_RTCM_1005, COM_PORT_UART1, 1);
    g_gnss.ubx().enableRTCMmessage(UBX_RTCM_1074, COM_PORT_UART1, 1);
    g_gnss.ubx().enableRTCMmessage(UBX_RTCM_1084, COM_PORT_UART1, 1);
    g_gnss.ubx().enableRTCMmessage(UBX_RTCM_1094, COM_PORT_UART1, 1);
    g_gnss.ubx().enableRTCMmessage(UBX_RTCM_1124, COM_PORT_UART1, 1);
    g_gnss.ubx().enableRTCMmessage(UBX_RTCM_1230, COM_PORT_UART1, 1);
}

static void waitSurveyIn() {
    const uint32_t startMs = millis();
    // 10 мин: Survey-In до 3см может копить 3-5 мин на открытом небе. Раньше было 180с —
    // база сдавалась рано и слала RTCM с неточной позицией → rover не брал FIXED.
    const uint32_t timeoutMs = 600000UL;
    bool fallbackStarted = false;

    while ((millis() - startMs) < timeoutMs) {
        bool ok = g_gnss.ubx().getSurveyStatus(2500);
        bool active = g_gnss.ubx().getSurveyInActive();
        bool valid = g_gnss.ubx().getSurveyInValid();
        uint32_t dur = g_gnss.ubx().getSurveyInObservationTimeFull();
        float acc = g_gnss.ubx().getSurveyInMeanAccuracy();
        uint8_t siv = g_gnss.ubx().getSIV();   // спутников в решении — диагностика приёма базы
        Serial.printf("[BASE] survey ok=%d active=%d valid=%d dur=%lu acc=%.3f m siv=%u\n",
                      ok ? 1 : 0, active ? 1 : 0, valid ? 1 : 0, dur, acc, siv);
        if (ok && valid) {
            Serial.println("[BASE] survey valid, RTCM 1005 should be available");
            enableBaseRtcmMessages();
            return;
        }

        if (!fallbackStarted && ok && active &&
            dur >= BASE_SURVEY_FALLBACK_AFTER_S &&
            acc <= BASE_SURVEY_FALLBACK_ACC_M) {
            fallbackStarted = true;
            Serial.printf("[BASE] survey precision stuck at %.3f m after %lu s; "
                          "switching to fallback %us/%.1fm\n",
                          acc, dur,
                          (unsigned)BASE_SURVEY_FALLBACK_MIN_S,
                          (double)BASE_SURVEY_FALLBACK_ACC_M);
            g_gnss.ubx().setSurveyMode(0, 0, 0);
            delay(500);
            g_gnss.ubx().setSurveyMode(1,
                                       BASE_SURVEY_FALLBACK_MIN_S,
                                       BASE_SURVEY_FALLBACK_ACC_M);
            enableBaseRtcmMessages();
        }
        delay(1000);
    }

    Serial.println("[BASE] survey timeout, forwarding RTCM anyway");
    enableBaseRtcmMessages();
}

static void connectWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.setTxPower(WIFI_POWER_11dBm);
    IPAddress ip, gw, sn, dns;
    ip.fromString(BASE_IP);
    gw.fromString("192.168.31.1");
    sn.fromString("255.255.255.0");
    dns = gw;
    WiFi.config(ip, gw, sn, dns);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    int t = 0;
    while (WiFi.status() != WL_CONNECTED && t < 30) {
        delay(500);
        Serial.print(".");
        t++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[BASE] wifi connected ip=%s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\n[BASE] wifi TIMEOUT");
    }
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(200);
    Serial.println("\n[BASE] boot");

    if (!g_gnss.begin(F9pSerial, GNSS_BASE)) {
        Serial.println("[BASE] F9P NOT FOUND");
    }
    waitSurveyIn();
    connectWiFi();
    g_udp.begin(RTCM_UDP_PORT);
    Serial.println("[BASE] ready");
}

void loop() {
    uint32_t now = millis();
    // Do not call g_gnss.loop() here: it reads from the same UART and can
    // split RTCM3 frames before the UDP forwarder sees them.

    IPAddress rover;
    rover.fromString(ROVER_IP);

    while (F9pSerial.available()) {
        int ch = F9pSerial.read();
        if (ch < 0) break;
        g_stats.rawBytes++;

        const uint8_t* frame = nullptr;
        size_t frameLen = 0;
        if (!g_rtcmFilter.push((uint8_t)ch, frame, frameLen)) continue;

        g_stats.bytes += frameLen;
        g_stats.frames++;
        uint16_t msgType = (((uint16_t)frame[3] << 4) | (frame[4] >> 4)) & 0x0FFF;
        g_stats.lastType = msgType;
        switch (msgType) {
            case 1005: g_stats.t1005++; break;
            case 1074: g_stats.t1074++; break;
            case 1084: g_stats.t1084++; break;
            case 1094: g_stats.t1094++; break;
            case 1124: g_stats.t1124++; break;
            case 1230: g_stats.t1230++; break;
            default:   g_stats.tOther++; break;
        }

        // UDP unicast на ровер
        g_udp.beginPacket(rover, RTCM_UDP_PORT);
        g_udp.write(frame, frameLen);
        if (g_udp.endPacket()) g_stats.udpOk++;
        else                    g_stats.udpFail++;
    }

    static uint32_t lastLog = 0;
    if (now - lastLog > 5000) {
        lastLog = now;
        Serial.printf("[BASE] rtcmOut=%s rtcm=%lu/%lu raw=%lu drop=%lu crcFail=%lu udpOk=%lu udpFail=%lu "
                      "last=%lu types 1005=%lu 1074=%lu 1084=%lu 1094=%lu 1124=%lu 1230=%lu other=%lu\n",
            g_stats.frames > 0 ? "active" : "waiting",
            g_stats.bytes, g_stats.frames, g_stats.rawBytes,
            g_stats.dropped, g_stats.crcFail, g_stats.udpOk, g_stats.udpFail,
            g_stats.lastType, g_stats.t1005, g_stats.t1074, g_stats.t1084,
            g_stats.t1094, g_stats.t1124, g_stats.t1230, g_stats.tOther);
    }
}
