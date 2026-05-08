/**
 * RTK Base Station - Simplified
 *
 * Функции:
 * - Survey-In (10 минут или до достижения точности 1м)
 * - TCP server (порт 2102) для передачи RTCM роверу
 * - UDP broadcast (порт 2101) как backup
 *
 * Пины: GPS UART1 RX=4 TX=5, 38400 baud
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiServer.h>
#include <WiFiUdp.h>

#if __has_include("rtk_config_private.h")
#include "rtk_config_private.h"
#else
#include "rtk_config.example.h"
#endif

// ============== КОНФИГУРАЦИЯ ==============

static constexpr char GPS_PORT_NAME[] = "GPIO4/GPIO5";
static constexpr int PIN_GPS_RX = 4;
static constexpr int PIN_GPS_TX = 5;
static constexpr uint32_t GPS_BAUD = 38400;

static constexpr char WIFI_SSID[] = RTK_ROUTER_WIFI_SSID;
static constexpr char WIFI_PASS[] = RTK_ROUTER_WIFI_PASS;

#ifndef RTK_BASE_IP_A
#define RTK_BASE_IP_A 192
#define RTK_BASE_IP_B 168
#define RTK_BASE_IP_C 31
#define RTK_BASE_IP_D 207
#endif

static const IPAddress BASE_IP(RTK_BASE_IP_A, RTK_BASE_IP_B, RTK_BASE_IP_C, RTK_BASE_IP_D);
static const IPAddress GATEWAY(RTK_ROUTER_GATEWAY_A, RTK_ROUTER_GATEWAY_B, RTK_ROUTER_GATEWAY_C, RTK_ROUTER_GATEWAY_D);
static const IPAddress SUBNET(255, 255, 255, 0);
static const IPAddress BROADCAST_IP(RTK_ROUTER_GATEWAY_A, RTK_ROUTER_GATEWAY_B, RTK_ROUTER_GATEWAY_C, 255);

static constexpr uint16_t TCP_PORT = 2102;
static constexpr uint16_t UDP_PORT = 2101;

static constexpr uint32_t SURVEY_IN_SECONDS = 600;        // 10 минут
static constexpr uint32_t SURVEY_IN_ACC_LIMIT_MM = 1000;  // 1 метр
static constexpr uint32_t STATUS_MS = 2000;
static constexpr uint32_t WIFI_RETRY_MS = 5000;

// ============== UBX КЛЮЧИ ==============

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

// ============== UART И СЕТЬ ==============

static HardwareSerial GpsSerial(1);
static WiFiServer tcpServer(TCP_PORT);
static WiFiClient tcpClient;
static WiFiUDP rtcmUdp;

// ============== СОСТОЯНИЕ ==============

static bool wifiConnected = false;
static uint32_t wifiConnectedMs = 0;
static uint32_t lastWifiAttemptMs = 0;
static uint32_t lastStatusMs = 0;
static uint32_t lastSvinPollMs = 0;
static uint32_t lastReconfigMs = 0;

static bool svinActive = false;
static bool svinValid = false;
static uint32_t svinDur = 0;
static uint32_t svinAcc = 0;

static uint32_t rtcmBytesSent = 0;
static uint32_t rtcmPacketsSent = 0;
static uint32_t tcpClients = 0;
static uint32_t tcpSent = 0;
static uint32_t tcpFailed = 0;
static uint32_t udpSent = 0;
static uint32_t udpFailed = 0;

static uint32_t gnssRawBytes = 0;
static uint32_t gnssParsed = 0;

// ============== UBX ПАРСИНГ ==============

enum UbxState { SYNC1, SYNC2, CLASS, ID, LEN1, LEN2, PAYLOAD, CKA, CKB };
static UbxState ubxState = SYNC1;
static uint8_t ubxClass = 0;
static uint8_t ubxId = 0;
static uint16_t ubxLen = 0;
static uint16_t ubxCount = 0;
static uint8_t ubxPayload[128];
static uint8_t ubxCkA = 0;
static uint8_t ubxCkB = 0;

static void writeUbx(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len) {
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

static void pollUbx(uint8_t cls, uint8_t id) {
  writeUbx(cls, id, nullptr, 0);
}

static void putU32(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

static uint32_t getU32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint32_t lastAckMs = 0;
static uint8_t lastAckClass = 0;
static uint8_t lastAckId = 0;
static bool lastAckOk = false;

static bool waitForAck(uint8_t cls, uint8_t id, uint32_t timeoutMs) {
  lastAckMs = millis();
  lastAckClass = cls;
  lastAckId = id;
  lastAckOk = false;
  writeUbx(cls, id, nullptr, 0);

  while (millis() - lastAckMs < timeoutMs) {
    while (GpsSerial.available()) {
      uint8_t b = GpsSerial.read();
      gnssRawBytes++;

      switch (ubxState) {
        case SYNC1: ubxState = (b == 0xB5) ? SYNC2 : SYNC1; break;
        case SYNC2: ubxState = (b == 0x62) ? CLASS : SYNC1; ubxCkA = ubxCkB = 0; break;
        case CLASS: ubxClass = b; ubxCkA += b; ubxCkB += ubxCkA; ubxState = ID; break;
        case ID: ubxId = b; ubxCkA += b; ubxCkB += ubxCkA; ubxState = LEN1; break;
        case LEN1: ubxLen = b; ubxCkA += b; ubxCkB += ubxCkA; ubxState = LEN2; break;
        case LEN2: ubxLen |= ((uint16_t)b << 8); ubxCkA += b; ubxCkB += ubxCkA;
          ubxCount = 0;
          ubxState = (ubxLen == 0) ? CKA : (ubxLen > sizeof(ubxPayload) ? SYNC1 : PAYLOAD);
          break;
        case PAYLOAD: ubxPayload[ubxCount++] = b; ubxCkA += b; ubxCkB += ubxCkA;
          if (ubxCount >= ubxLen) ubxState = CKA; break;
        case CKA: ubxState = (b == ubxCkA) ? CKB : SYNC1; break;
        case CKB:
          if (b == ubxCkB) {
            if (ubxClass == 0x05 && ubxId == 0x01) {
              lastAckOk = true;
            }
          }
          ubxState = SYNC1;
          break;
      }

      if (lastAckOk && lastAckClass == cls && lastAckId == id) {
        return true;
      }
    }
    delayMicroseconds(100);
  }
  return false;
}

static void sendValset(uint32_t key, uint32_t value, uint8_t size) {
  uint8_t payload[16];
  payload[0] = 0; payload[1] = 0x01; payload[2] = 0; payload[3] = 0;
  putU32(payload + 4, key);
  for (uint8_t i = 0; i < size; i++) {
    payload[8 + i] = (uint8_t)(value >> (8 * i));
  }
  waitForAck(0x06, 0x8A, 800);
  writeUbx(0x06, 0x8A, payload, 8 + size);
  delay(50);
}

static void sendCfgTmode3(uint16_t flags, uint32_t dur, uint32_t acc) {
  uint8_t payload[40] = {};
  payload[0] = 0;
  payload[1] = (uint8_t)(flags & 0xFF);
  payload[2] = (uint8_t)((flags >> 8) & 0xFF);
  putU32(payload + 24, dur);
  putU32(payload + 28, acc);
  waitForAck(0x06, 0x71, 800);
  writeUbx(0x06, 0x71, payload, sizeof(payload));
  delay(50);
}

static void configureBase() {
  Serial.println("BASE: configuring...");

  // Survey-In параметры
  sendValset(CFG_TMODE_SVIN_MIN_DUR, SURVEY_IN_SECONDS, 4);
  sendValset(CFG_TMODE_SVIN_ACC_LIMIT, SURVEY_IN_ACC_LIMIT_MM * 10, 4);  // в 0.1мм

  // RTCM output rate
  sendValset(CFG_RTCM_1005, 1, 1);
  sendValset(CFG_RTCM_1074, 1, 1);
  sendValset(CFG_RTCM_1084, 1, 1);
  sendValset(CFG_RTCM_1094, 1, 1);
  sendValset(CFG_RTCM_1124, 1, 1);
  sendValset(CFG_RTCM_1230, 10, 1);

  // Measurement rate 1000ms
  sendValset(CFG_RATE_MEAS, 1000, 2);

  // Включить Survey-In mode
  sendValset(CFG_TMODE_MODE, 1, 1);

  // Legacy CFG-TMODE3 как backup
  sendCfgTmode3(1, SURVEY_IN_SECONDS, SURVEY_IN_ACC_LIMIT_MM * 10);

  lastReconfigMs = millis();
  Serial.println("BASE: survey-in started");
}

static void feedUbx(uint8_t b) {
  switch (ubxState) {
    case SYNC1: ubxState = (b == 0xB5) ? SYNC2 : SYNC1; break;
    case SYNC2: ubxState = (b == 0x62) ? CLASS : SYNC1; ubxCkA = ubxCkB = 0; break;
    case CLASS: ubxClass = b; ubxCkA += b; ubxCkB += ubxCkA; ubxState = ID; break;
    case ID: ubxId = b; ubxCkA += b; ubxCkB += ubxCkA; ubxState = LEN1; break;
    case LEN1: ubxLen = b; ubxCkA += b; ubxCkB += ubxCkA; ubxState = LEN2; break;
    case LEN2: ubxLen |= ((uint16_t)b << 8); ubxCkA += b; ubxCkB += ubxCkA;
      ubxCount = 0;
      ubxState = (ubxLen == 0) ? CKA : (ubxLen > sizeof(ubxPayload) ? SYNC1 : PAYLOAD);
      break;
    case PAYLOAD: ubxPayload[ubxCount++] = b; ubxCkA += b; ubxCkB += ubxCkA;
      if (ubxCount >= ubxLen) ubxState = CKA; break;
    case CKA: ubxState = (b == ubxCkA) ? CKB : SYNC1; break;
    case CKB:
      if (b == ubxCkB) {
        gnssParsed++;
        // NAV-SVIN (0x01 0x3B)
        if (ubxClass == 0x01 && ubxId == 0x3B && ubxLen >= 40) {
          svinDur = getU32(ubxPayload + 8);
          svinAcc = getU32(ubxPayload + 28);
          svinValid = ubxPayload[36] != 0;
          svinActive = ubxPayload[37] != 0;
        }
        // ACK/NACK
        if (ubxClass == 0x05) {
          lastAckClass = ubxPayload[0];
          lastAckId = ubxPayload[1];
          lastAckOk = (ubxId == 0x01);
        }
      }
      ubxState = SYNC1;
      break;
  }
}

// ============== RTCM ПАРСИНГ ==============

enum RtcmState { R_WAIT, R_LEN1, R_LEN2, R_BODY };
static RtcmState rtcmState = R_WAIT;
static uint8_t rtcmBuf[1200];
static uint16_t rtcmLen = 0;
static uint16_t rtcmTarget = 0;

static void feedRtcm(uint8_t b) {
  switch (rtcmState) {
    case R_WAIT:
      if (b == 0xD3) {
        rtcmBuf[0] = b;
        rtcmLen = 1;
        rtcmState = R_LEN1;
      }
      break;
    case R_LEN1:
      rtcmBuf[rtcmLen++] = b;
      rtcmTarget = ((uint16_t)(b & 0x03) << 8);
      rtcmState = R_LEN2;
      break;
    case R_LEN2:
      rtcmBuf[rtcmLen++] = b;
      rtcmTarget = (rtcmTarget | b) + 6;
      if (rtcmTarget > sizeof(rtcmBuf)) {
        rtcmState = R_WAIT;
      } else {
        rtcmState = R_BODY;
      }
      break;
    case R_BODY:
      rtcmBuf[rtcmLen++] = b;
      if (rtcmLen >= rtcmTarget) {
        rtcmPacketsSent++;
        rtcmBytesSent += rtcmLen;

        // TCP клиенту
        if (tcpClient && tcpClient.connected()) {
          size_t written = tcpClient.write(rtcmBuf, rtcmLen);
          if (written == rtcmLen) {
            tcpSent++;
          } else {
            tcpFailed++;
          }
        }

        // UDP broadcast как backup
        if (rtcmUdp.beginPacket(BROADCAST_IP, UDP_PORT) == 1) {
          rtcmUdp.write(rtcmBuf, rtcmLen);
          if (rtcmUdp.endPacket() == 1) {
            udpSent++;
          } else {
            udpFailed++;
          }
        } else {
          udpFailed++;
        }

        rtcmState = R_WAIT;
      }
      break;
  }
}

static void updateTcpClient() {
  if (tcpClient && !tcpClient.connected()) {
    tcpClient.stop();
  }

  WiFiClient newClient = tcpServer.available();
  if (newClient) {
    if (tcpClient && tcpClient.connected()) {
      tcpClient.stop();
    }
    tcpClient = newClient;
    tcpClient.setNoDelay(true);
    tcpClients++;
    Serial.printf("BASE: TCP client #%lu connected\n", (unsigned long)tcpClients);
  }
}

static void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiConnected) {
      wifiConnected = true;
      wifiConnectedMs = millis();
      Serial.printf("BASE: WiFi connected IP=%s\n", WiFi.localIP().toString().c_str());

      // Начать TCP server
      tcpServer.begin();
      Serial.printf("BASE: TCP server port %u\n", TCP_PORT);

      // Начать UDP
      rtcmUdp.begin(UDP_PORT);
      Serial.printf("BASE: UDP broadcast port %u\n", UDP_PORT);
    }
    return;
  }

  wifiConnected = false;
  uint32_t now = millis();
  if (now - lastWifiAttemptMs < WIFI_RETRY_MS) return;
  lastWifiAttemptMs = now;

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_15dBm);
  WiFi.setAutoReconnect(true);
  WiFi.config(BASE_IP, GATEWAY, SUBNET);

  Serial.printf("BASE: connecting to %s...\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

static void pollSvin() {
  uint32_t now = millis();
  if (now - lastSvinPollMs < 1000) return;
  lastSvinPollMs = now;
  pollUbx(0x01, 0x3B);  // NAV-SVIN poll
}

static void checkSurveyStatus() {
  uint32_t now = millis();

  // Retry config каждые 30 секунд если Survey-In не активен
  if (!svinActive && !svinValid && now - lastReconfigMs > 30000) {
    Serial.println("BASE: survey-in not active, reconfiguring...");
    configureBase();
  }

  // Если Survey-In валиден, вывести сообщение
  if (svinValid && !wifiConnected) {
    Serial.println("BASE: Survey-In VALID!");
  }
}

static void printStatus() {
  uint32_t now = millis();
  if (now - lastStatusMs < STATUS_MS) return;
  lastStatusMs = now;

  Serial.printf("BASE svin=%u/%u dur=%lus acc=%.3fm rtcm=%luB/%lu pkts tcp=%lu/%lu udp=%lu/%lu wifi=%s\n",
    svinActive, svinValid,
    (unsigned long)svinDur,
    svinAcc / 10000.0,
    (unsigned long)rtcmBytesSent,
    (unsigned long)rtcmPacketsSent,
    (unsigned long)tcpSent,
    (unsigned long)tcpFailed,
    (unsigned long)udpSent,
    (unsigned long)udpFailed,
    wifiConnected ? "connected" : "disconnected"
  );
}

// ============== SETUP И LOOP ==============

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("===== RTK BASE STATION =====");
  Serial.printf("GPS: UART1 %s %lu baud\n", GPS_PORT_NAME, (unsigned long)GPS_BAUD);

  // GPS UART
  GpsSerial.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  delay(300);

  // Конфигурация
  configureBase();

  // WiFi подключение
  connectWiFi();
}

void loop() {
  connectWiFi();

  if (wifiConnected) {
    updateTcpClient();
  }

  // Чтение GPS
  while (GpsSerial.available()) {
    uint8_t b = GpsSerial.read();
    gnssRawBytes++;
    feedUbx(b);
    feedRtcm(b);
  }

  pollSvin();
  checkSurveyStatus();
  printStatus();
}
