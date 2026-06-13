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
    uint32_t bytes = 0;
    uint32_t pkts  = 0;
    uint32_t udpOk = 0;
    uint32_t udpFail = 0;
} g_stats;

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
    connectWiFi();
    g_udp.begin(RTCM_UDP_PORT);
    Serial.println("[BASE] ready");
}

void loop() {
    uint32_t now = millis();
    g_gnss.loop();

    static uint8_t buf[512];
    IPAddress rover;
    rover.fromString(ROVER_IP);

    while (F9pSerial.available()) {
        int n = F9pSerial.readBytes(buf, sizeof(buf));
        if (n <= 0) break;
        g_stats.bytes += n;
        g_stats.pkts++;

        // UDP unicast на ровер
        g_udp.beginPacket(rover, RTCM_UDP_PORT);
        g_udp.write(buf, n);
        if (g_udp.endPacket()) g_stats.udpOk++;
        else                    g_stats.udpFail++;
    }

    static uint32_t lastLog = 0;
    if (now - lastLog > 5000) {
        lastLog = now;
        uint16_t accMm = 0; uint32_t durS = 0;
        bool complete = g_gnss.baseSurveyComplete(accMm, durS);
        Serial.printf("[BASE] svin=%s rtcm=%lu/%lu udpOk=%lu udpFail=%lu\n",
            complete ? "VALID" : (g_gnss.baseSurveyInProgress() ? "running" : "off"),
            g_stats.bytes, g_stats.pkts, g_stats.udpOk, g_stats.udpFail);
    }
}
