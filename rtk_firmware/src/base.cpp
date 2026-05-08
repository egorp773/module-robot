/**
 * RTK Base Station - Survey-In + Buffered RTCM
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#if __has_include("rtk_config_private.h")
#include "rtk_config_private.h"
#else
#include "rtk_config.example.h"
#endif

static constexpr char WIFI_SSID[] = RTK_ROUTER_WIFI_SSID;
static constexpr char WIFI_PASS[] = RTK_ROUTER_WIFI_PASS;
static constexpr uint16_t UDP_PORT = 2101;

#ifndef RTK_ROUTER_GATEWAY_A
#define RTK_ROUTER_GATEWAY_A 192
#define RTK_ROUTER_GATEWAY_C 31
#define RTK_ROUTER_GATEWAY_B 168
#endif
#ifndef RTK_ROUTER_GATEWAY_D
#define RTK_ROUTER_GATEWAY_D 1
#endif

static constexpr int PIN_GPS_RX = 4;
static constexpr int PIN_GPS_TX = 5;
static constexpr uint32_t GPS_BAUD = 38400;
static constexpr uint32_t BROADCAST_INTERVAL_MS = 50;  // отправка каждые 50мс

static HardwareSerial GpsSerial(1);
static WiFiUDP udp;

// Rover IP - unicast вместо broadcast
#ifndef RTK_ROVER_IP_A
#define RTK_ROVER_IP_A 192
#define RTK_ROVER_IP_B 168
#define RTK_ROVER_IP_C 31
#define RTK_ROVER_IP_D 222
#endif
static IPAddress roverIP(RTK_ROVER_IP_A, RTK_ROVER_IP_B, RTK_ROVER_IP_C, RTK_ROVER_IP_D);

// Буфер для RTCM
static constexpr uint16_t RTCM_BUF_SIZE = 512;
static uint8_t rtcmBuf[RTCM_BUF_SIZE];
static uint16_t rtcmLen = 0;

// UBX parsing
static uint8_t ubxClass = 0, ubxId = 0;
static uint16_t ubxLen = 0, ubxCount = 0;
static uint8_t ubxCkA = 0, ubxCkB = 0;
static bool gotSvin = false;

// UBX keys
static constexpr uint32_t CFG_TMODE_MODE = 0x20030001;
static constexpr uint32_t CFG_TMODE_SVIN_MIN_DUR = 0x40030010;
static constexpr uint32_t CFG_TMODE_SVIN_ACC_LIMIT = 0x40030011;
static constexpr uint32_t CFG_RATE_MEAS = 0x30210001;
static constexpr uint32_t CFG_RTCM_1005 = 0x209102BE;
static constexpr uint32_t CFG_RTCM_1074 = 0x2091035F;
static constexpr uint32_t CFG_RTCM_1084 = 0x20910364;
static constexpr uint32_t CFG_RTCM_1094 = 0x20910369;
static constexpr uint32_t CFG_RTCM_1124 = 0x2091036E;
static constexpr uint32_t CFG_RTCM_1230 = 0x20910304;

void writeUbx(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len) {
  uint8_t ckA = 0, ckB = 0;
  auto addCk = [&](uint8_t b) { ckA += b; ckB += ckA; };

  GpsSerial.write(0xB5);
  GpsSerial.write(0x62);
  GpsSerial.write(cls); addCk(cls);
  GpsSerial.write(id); addCk(id);
  GpsSerial.write((uint8_t)len); addCk((uint8_t)len);
  GpsSerial.write((uint8_t)(len >> 8)); addCk((uint8_t)(len >> 8));
  for (uint16_t i = 0; i < len; i++) {
    GpsSerial.write(payload[i]);
    addCk(payload[i]);
  }
  GpsSerial.write(ckA);
  GpsSerial.write(ckB);
}

void putU32(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

void sendValset(uint32_t key, uint32_t value, uint8_t size) {
  uint8_t payload[16];
  payload[0] = 0; payload[1] = 0x01; payload[2] = 0; payload[3] = 0;
  putU32(payload + 4, key);
  for (uint8_t i = 0; i < size; i++) {
    payload[8 + i] = (uint8_t)(value >> (8 * i));
  }
  writeUbx(0x06, 0x8A, payload, 8 + size);
}

void configureGps() {
  Serial.println("Configuring GPS...");

  // Survey-In: 10 минут, точность 1м
  sendValset(CFG_TMODE_SVIN_MIN_DUR, 600, 4);
  sendValset(CFG_TMODE_SVIN_ACC_LIMIT, 10000, 4);  // 10м * 1000 = 10000 мм -> 0.1мм units

  // RTCM messages
  sendValset(CFG_RTCM_1005, 1, 1);
  sendValset(CFG_RTCM_1074, 1, 1);
  sendValset(CFG_RTCM_1084, 1, 1);
  sendValset(CFG_RTCM_1094, 1, 1);
  sendValset(CFG_RTCM_1124, 1, 1);
  sendValset(CFG_RTCM_1230, 10, 1);

  // Measurement rate
  sendValset(CFG_RATE_MEAS, 1000, 2);

  // Enable Survey-In
  delay(200);
  sendValset(CFG_TMODE_MODE, 1, 1);

  Serial.println("Survey-In configured");
}

void parseUbx(uint8_t b) {
  static uint8_t state = 0;  // 0=sync1, 1=sync2, 2=class, 3=id, 4=len1, 5=len2, 6=payload, 7=cka, 8=ckb
  static uint8_t payload[128];

  switch (state) {
    case 0: state = (b == 0xB5) ? 1 : 0; break;
    case 1: state = (b == 0x62) ? 2 : 0; ubxCkA = ubxCkB = 0; break;
    case 2: ubxClass = b; ubxCkA += b; ubxCkB += ubxCkA; state = 3; break;
    case 3: ubxId = b; ubxCkA += b; ubxCkB += ubxCkA; state = 4; break;
    case 4: ubxLen = b; ubxCkA += b; ubxCkB += ubxCkA; state = 5; break;
    case 5:
      ubxLen |= ((uint16_t)b << 8);
      ubxCkA += b; ubxCkB += ubxCkA;
      ubxCount = 0;
      state = (ubxLen == 0 || ubxLen > sizeof(payload)) ? 7 : 6;
      break;
    case 6:
      payload[ubxCount++] = b;
      ubxCkA += b; ubxCkB += ubxCkA;
      if (ubxCount >= ubxLen) state = 7;
      break;
    case 7:
      state = (b == ubxCkA) ? 8 : 0;
      break;
    case 8:
      state = 0;
      if (b == ubxCkB) {
        // NAV-SVIN (0x01 0x3B)
        if (ubxClass == 0x01 && ubxId == 0x3B) {
          gotSvin = true;
          uint32_t dur = (uint32_t)payload[8] | ((uint32_t)payload[9] << 8) |
                         ((uint32_t)payload[10] << 16) | ((uint32_t)payload[11] << 24);
          uint32_t acc = (uint32_t)payload[28] | ((uint32_t)payload[29] << 8) |
                         ((uint32_t)payload[30] << 16) | ((uint32_t)payload[31] << 24);
          bool valid = payload[36];
          bool active = payload[37];
          Serial.printf("SVIN: active=%u valid=%u dur=%us acc=%.3fm\n", active, valid, dur, acc / 10000.0);
        }
      }
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("===== RTK BASE STATION =====");
  Serial.printf("WiFi: %s\n", WIFI_SSID);
  Serial.printf("RTCM to rover %s port %u\n", roverIP.toString().c_str(), UDP_PORT);

  GpsSerial.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  delay(200);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.println("WiFi connecting...");
}

void loop() {
  static bool wifiShown = false;
  static bool gpsConfigured = false;
  static uint32_t lastBroadcast = 0;

  // WiFi
  if (WiFi.status() == WL_CONNECTED && !wifiShown) {
    Serial.printf("WiFi: IP=%s RSSI=%d\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
    wifiShown = true;
    udp.begin(UDP_PORT);
    delay(500);
  }

  // Configure GPS once
  if (wifiShown && !gpsConfigured) {
    configureGps();
    gpsConfigured = true;
  }

  // Read GPS
  while (GpsSerial.available()) {
    uint8_t b = GpsSerial.read();

    // Parse UBX
    parseUbx(b);

    // Buffer RTCM (starts with 0xD3)
    if (b == 0xD3) {
      rtcmLen = 0;
      rtcmBuf[rtcmLen++] = b;
    } else if (rtcmLen > 0 && rtcmLen < RTCM_BUF_SIZE) {
      rtcmBuf[rtcmLen++] = b;
    }
  }

  // Send RTCM to rover (unicast)
  if (wifiShown && rtcmLen > 0 && millis() - lastBroadcast > BROADCAST_INTERVAL_MS) {
    udp.beginPacket(roverIP, UDP_PORT);
    udp.write(rtcmBuf, rtcmLen);
    udp.endPacket();
    lastBroadcast = millis();
    rtcmLen = 0;
  }
}
