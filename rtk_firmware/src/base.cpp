/**
 * RTK Base Station - FULL DEBUG VERSION
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_system.h>

#if __has_include("rtk_config_private.h")
#include "rtk_config_private.h"
#else
#include "rtk_config.example.h"
#endif

static constexpr char WIFI_SSID[] = RTK_ROUTER_WIFI_SSID;
static constexpr char WIFI_PASS[] = RTK_ROUTER_WIFI_PASS;
static constexpr uint16_t UDP_PORT = 2101;
static constexpr uint32_t WIFI_RECONNECT_INITIAL_MS = 15000;
static constexpr uint32_t WIFI_RECONNECT_MAX_MS = 60000;

#ifndef RTK_ROUTER_GATEWAY_A
#define RTK_ROUTER_GATEWAY_A 192
#define RTK_ROUTER_GATEWAY_B 168
#define RTK_ROUTER_GATEWAY_C 31
#define RTK_ROUTER_GATEWAY_D 1
#endif

static constexpr int PIN_GPS_RX = 4;
static constexpr int PIN_GPS_TX = 5;
static constexpr int PIN_GPS_ALT_RX = PIN_GPS_TX;
static constexpr int PIN_GPS_ALT_TX = PIN_GPS_RX;
static constexpr uint32_t GPS_BAUD = 38400;
static constexpr uint32_t STATUS_INTERVAL_MS = 5000;
static constexpr uint32_t SVIN_MIN_DUR_S = 300;
static constexpr uint32_t SVIN_ACC_LIMIT_0_1MM = 10000;  // 1.0 m, units are 0.1 mm
static int activeGpsRx = PIN_GPS_RX;
static int activeGpsTx = PIN_GPS_TX;

static HardwareSerial GpsSerial(1);
static WiFiUDP udp;

#ifndef RTK_BASE_IP_A
#define RTK_BASE_IP_A 192
#define RTK_BASE_IP_B 168
#define RTK_BASE_IP_C 31
#define RTK_BASE_IP_D 207
#endif
#ifndef RTK_ROVER_IP_A
#define RTK_ROVER_IP_A 192
#define RTK_ROVER_IP_B 168
#define RTK_ROVER_IP_C 31
#define RTK_ROVER_IP_D 222
#endif
static IPAddress baseIP(RTK_BASE_IP_A, RTK_BASE_IP_B, RTK_BASE_IP_C, RTK_BASE_IP_D);
static IPAddress roverIP(RTK_ROVER_IP_A, RTK_ROVER_IP_B, RTK_ROVER_IP_C, RTK_ROVER_IP_D);
static IPAddress gateway(RTK_ROUTER_GATEWAY_A, RTK_ROUTER_GATEWAY_B, RTK_ROUTER_GATEWAY_C, RTK_ROUTER_GATEWAY_D);
static IPAddress subnet(255, 255, 255, 0);

// RTCM буфер
// RTCM frames can carry up to 1023 bytes of payload plus header/CRC.
static constexpr uint16_t RTCM_BUF_SIZE = 1200;
static constexpr uint8_t RTCM_FRAME_QUEUE_SIZE = 8;
static uint8_t rtcmBuf[RTCM_BUF_SIZE];
static uint16_t rtcmLen = 0;
static uint16_t rtcmExpectedLen = 0;
static uint8_t rtcmFrameQueue[RTCM_FRAME_QUEUE_SIZE][RTCM_BUF_SIZE];
static uint16_t rtcmFrameQueueLen[RTCM_FRAME_QUEUE_SIZE];
static uint8_t rtcmFrameQueueHead = 0;
static uint8_t rtcmFrameQueueTail = 0;
static uint8_t rtcmFrameQueueCount = 0;
static uint32_t rtcmFrameQueueDrop = 0;
static uint32_t rtcmPktsSent = 0;
static uint32_t rtcmBytesSent = 0;
static uint32_t rtcmErrors = 0;
static uint32_t rtcmOverflow = 0;
static uint32_t rtcmCrcFail = 0;
static uint32_t rtcmLastType = 0;
static uint32_t rtcmType1005 = 0;
static uint32_t rtcmType1074 = 0;
static uint32_t rtcmType1084 = 0;
static uint32_t rtcmType1094 = 0;
static uint32_t rtcmType1124 = 0;
static uint32_t rtcmType1230 = 0;
static uint32_t rtcmTypeOther = 0;
static uint32_t lastRtcmMs = 0;
static uint32_t lastRtcmSendMs = 0;
static uint32_t lastRtcmByteMs = 0;
static bool rtcmRawForwarding = false;
static bool rtcmFrameForwarding = false;
static constexpr uint16_t RAW_RTCM_UDP_BUF_SIZE = 1024;
static constexpr uint32_t RAW_RTCM_FLUSH_MS = 80;
static uint8_t rawRtcmUdpBuf[RAW_RTCM_UDP_BUF_SIZE];
static uint16_t rawRtcmUdpLen = 0;
static uint32_t rawRtcmPktsSent = 0;
static uint32_t rawRtcmBytesSent = 0;
static uint32_t rawRtcmLastFlushMs = 0;

// GPS stats
static uint32_t gpsRawBytes = 0;
static uint32_t ubxParsed = 0;
static uint32_t rtcm3Parsed = 0;
static uint8_t firstBytes[20];
static uint8_t firstBytesCount = 0;
static bool firstBytesPrinted = false;

// UBX parsing
static uint8_t ubxClass = 0, ubxId = 0;
static uint16_t ubxLen = 0, ubxCount = 0;
static uint8_t ubxCkA = 0, ubxCkB = 0;
static uint8_t ubxState = 0;  // 0=sync1, 1=sync2, 2=class, 3=id, 4=len1, 5=len2, 6=payload, 7=cka, 8=ckb
static uint8_t ubxPayload[128];

// UBX keys
static constexpr uint32_t CFG_TMODE_MODE = 0x20030001;
static constexpr uint32_t CFG_TMODE_SVIN_MIN_DUR = 0x40030010;
static constexpr uint32_t CFG_TMODE_SVIN_ACC_LIMIT = 0x40030011;
static constexpr uint32_t CFG_RATE_MEAS = 0x30210001;
static constexpr uint32_t CFG_MSGOUT_UBX_NAV_SVIN_UART1 = 0x2091008B;
static constexpr uint32_t CFG_MSGOUT_UBX_NAV_PVT_UART1 = 0x20910007;
static constexpr uint32_t CFG_RTCM_1005 = 0x209102BE;
static constexpr uint32_t CFG_RTCM_1074 = 0x2091035F;
static constexpr uint32_t CFG_RTCM_1084 = 0x20910364;
static constexpr uint32_t CFG_RTCM_1094 = 0x20910369;
static constexpr uint32_t CFG_RTCM_1124 = 0x2091036E;
static constexpr uint32_t CFG_RTCM_1230 = 0x20910304;
static constexpr uint32_t CFG_MSGOUT_NMEA_GGA_UART1 = 0x209100BA;
static constexpr uint32_t CFG_MSGOUT_NMEA_GLL_UART1 = 0x209100C9;
static constexpr uint32_t CFG_MSGOUT_NMEA_GSA_UART1 = 0x209100BE;
static constexpr uint32_t CFG_MSGOUT_NMEA_GSV_UART1 = 0x209100C3;
static constexpr uint32_t CFG_MSGOUT_NMEA_RMC_UART1 = 0x209100AC;
static constexpr uint32_t CFG_MSGOUT_NMEA_VTG_UART1 = 0x209100B1;
static constexpr uint32_t CFG_SIGNAL_GPS_ENA = 0x1031001F;
static constexpr uint32_t CFG_SIGNAL_GPS_L1CA_ENA = 0x10310001;
static constexpr uint32_t CFG_SIGNAL_GPS_L2C_ENA = 0x10310003;
static constexpr uint32_t CFG_SIGNAL_GAL_ENA = 0x10310021;
static constexpr uint32_t CFG_SIGNAL_GAL_E1_ENA = 0x10310007;
static constexpr uint32_t CFG_SIGNAL_GAL_E5B_ENA = 0x1031000A;
static constexpr uint32_t CFG_SIGNAL_BDS_ENA = 0x10310022;
static constexpr uint32_t CFG_SIGNAL_BDS_B1_ENA = 0x1031000D;
static constexpr uint32_t CFG_SIGNAL_BDS_B2_ENA = 0x1031000E;
static constexpr uint32_t CFG_SIGNAL_GLO_ENA = 0x10310025;
static constexpr uint32_t CFG_SIGNAL_GLO_L1_ENA = 0x10310018;
static constexpr uint32_t CFG_SIGNAL_GLO_L2_ENA = 0x1031001A;

// UBX-CFG-NMEA: принудительный режим UBX (игнорирует NMEA mode)
static constexpr uint32_t CFG_NMEA_PROTOVER = 0x2099001A;
static constexpr uint32_t CFG_NMEA_OUT_PROTOVERSION = 0x2099001B;
static constexpr uint32_t CFG_NMEA_BRDOT = 0x20990026;

// Состояние
static bool wifiConnected = false;
static bool gpsConfigured = false;
static uint32_t lastWifiReconnectMs = 0;
static uint32_t wifiReconnectIntervalMs = WIFI_RECONNECT_INITIAL_MS;
static uint32_t svinValid = 0;
static uint32_t svinDur = 0;
static uint32_t svinAcc = 0;
static uint32_t svinActive = 0;
static bool rtcmConfiguredAfterSvin = false;
static bool svinPollingEnabled = true;

static void beginBaseWiFi() {
  WiFi.config(baseIP, gateway, subnet);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

static const char* resetReasonString(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON: return "poweron";
    case ESP_RST_EXT: return "external";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "interrupt_wdt";
    case ESP_RST_TASK_WDT: return "task_wdt";
    case ESP_RST_WDT: return "other_wdt";
    case ESP_RST_DEEPSLEEP: return "deepsleep";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_SDIO: return "sdio";
    default: return "unknown";
  }
}

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

struct GpsUartCandidate {
  int rx;
  int tx;
  const char* label;
};

static uint32_t scoreGpsUart(uint32_t passiveMs, bool activeProbe) {
  uint32_t bytes = 0;
  uint32_t score = 0;
  int prev = -1;
  uint32_t start = millis();
  bool probeSent = false;

  while (millis() - start < passiveMs) {
    if (activeProbe && !probeSent && millis() - start > passiveMs / 3) {
      writeUbx(0x01, 0x07, nullptr, 0);
      GpsSerial.flush();
      probeSent = true;
    }
    while (GpsSerial.available()) {
      uint8_t b = GpsSerial.read();
      bytes++;
      if (b == '$') score += 8;
      if (b == 0xD3) score += 3;
      if (prev == 0xB5 && b == 0x62) score += 40;
      if (b >= 32 && b <= 126) score++;
      prev = b;
    }
    delay(1);
  }
  return score + min(bytes, (uint32_t)200);
}

static void selectGpsUart() {
  const GpsUartCandidate candidates[] = {
    {PIN_GPS_RX, PIN_GPS_TX, "normal"},
    {PIN_GPS_ALT_RX, PIN_GPS_ALT_TX, "swapped"},
  };
  uint32_t bestScore = 0;
  uint8_t bestIndex = 0;

  Serial.println("GPS UART autodetect: checking normal and swapped TX/RX");
  for (uint8_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++) {
    GpsSerial.end();
    delay(50);
    GpsSerial.begin(GPS_BAUD, SERIAL_8N1, candidates[i].rx, candidates[i].tx);
    delay(200);
    while (GpsSerial.available()) GpsSerial.read();
    uint32_t score = scoreGpsUart(1200, true);
    Serial.printf("GPS UART candidate %s RX=%d TX=%d score=%lu\n",
      candidates[i].label, candidates[i].rx, candidates[i].tx, (unsigned long)score);
    if (score > bestScore) {
      bestScore = score;
      bestIndex = i;
    }
  }

  activeGpsRx = candidates[bestIndex].rx;
  activeGpsTx = candidates[bestIndex].tx;
  GpsSerial.end();
  delay(50);
  GpsSerial.begin(GPS_BAUD, SERIAL_8N1, activeGpsRx, activeGpsTx);
  delay(200);
  Serial.printf("GPS UART selected: %s RX=%d TX=%d score=%lu\n",
    candidates[bestIndex].label, activeGpsRx, activeGpsTx, (unsigned long)bestScore);
}

void putU32(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

void putU16(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
}

void sendValset(uint32_t key, uint32_t value, uint8_t size) {
  uint8_t payload[16];
  payload[0] = 0; payload[1] = 0x01; payload[2] = 0; payload[3] = 0;
  putU32(payload + 4, key);
  for (uint8_t i = 0; i < size; i++) {
    payload[8 + i] = (uint8_t)(value >> (8 * i));
  }
  writeUbx(0x06, 0x8A, payload, 8 + size);
  GpsSerial.flush();
  delay(40);
  Serial.printf("BASE: CFG msg 0x%08X = %lu\n", key, (unsigned long)value);
}

void sendCfgMsg(uint8_t msgClass, uint8_t msgId, uint8_t uart1Rate) {
  uint8_t payload[8] = {0};
  payload[0] = msgClass;
  payload[1] = msgId;
  payload[3] = uart1Rate;
  writeUbx(0x06, 0x01, payload, sizeof(payload));
  GpsSerial.flush();
  delay(40);
  Serial.printf("BASE: CFG-MSG class=0x%02X id=0x%02X uart1Rate=%u\n",
    msgClass, msgId, uart1Rate);
}

void sendCfgTmode3SurveyIn() {
  uint8_t payload[40] = {0};
  payload[0] = 0;
  putU16(payload + 2, 1);
  putU32(payload + 20, 0);  // fixedPosAcc, unused in survey-in mode.
  putU32(payload + 24, SVIN_MIN_DUR_S);
  putU32(payload + 28, SVIN_ACC_LIMIT_0_1MM);
  writeUbx(0x06, 0x71, payload, sizeof(payload));
  GpsSerial.flush();
  delay(40);
  Serial.printf("BASE: Sent CFG-TMODE3 survey-in minDur=%lus accLimit=%.3fm\n",
    (unsigned long)SVIN_MIN_DUR_S, SVIN_ACC_LIMIT_0_1MM / 10000.0);
}

void sendCfgPrt(uint16_t inProtoMask, uint16_t outProtoMask, const char* label) {
  uint8_t cfgPrt[32];
  cfgPrt[0] = 0xB5; cfgPrt[1] = 0x62;  // sync
  cfgPrt[2] = 0x06; cfgPrt[3] = 0x00;  // class, id
  cfgPrt[4] = 0x14; cfgPrt[5] = 0x00;  // len = 20
  cfgPrt[6] = 0x01;  // portID = UART
  cfgPrt[7] = 0x00;  // reserved
  cfgPrt[8] = 0x00; cfgPrt[9] = 0x00;  // txReady
  cfgPrt[10] = 0xC0; cfgPrt[11] = 0x08; cfgPrt[12] = 0x00; cfgPrt[13] = 0x00;  // mode: 8N1
  cfgPrt[14] = (uint8_t)GPS_BAUD;
  cfgPrt[15] = (uint8_t)(GPS_BAUD >> 8);
  cfgPrt[16] = (uint8_t)(GPS_BAUD >> 16);
  cfgPrt[17] = (uint8_t)(GPS_BAUD >> 24);
  cfgPrt[18] = (uint8_t)inProtoMask;
  cfgPrt[19] = (uint8_t)(inProtoMask >> 8);
  cfgPrt[20] = (uint8_t)outProtoMask;
  cfgPrt[21] = (uint8_t)(outProtoMask >> 8);
  cfgPrt[22] = 0x00; cfgPrt[23] = 0x00;  // flags
  cfgPrt[24] = 0x00; cfgPrt[25] = 0x00;  // reserved
  uint8_t ckA = 0, ckB = 0;
  for (int i = 2; i <= 25; i++) { ckA += cfgPrt[i]; ckB += ckA; }
  cfgPrt[26] = ckA; cfgPrt[27] = ckB;
  GpsSerial.write(cfgPrt, 28);
  GpsSerial.flush();
  delay(80);
  Serial.printf("BASE: Sent CFG-PRT %s in=0x%04X out=0x%04X baud=%lu\n",
    label, inProtoMask, outProtoMask, (unsigned long)GPS_BAUD);
}

void configureRtcmOutput() {
  Serial.println("BASE: configuring RTCM output...");

  sendValset(CFG_RTCM_1005, 1, 1);
  sendValset(CFG_RTCM_1074, 1, 1);
  sendValset(CFG_RTCM_1084, 1, 1);
  sendValset(CFG_RTCM_1094, 1, 1);
  sendValset(CFG_RTCM_1124, 1, 1);
  sendValset(CFG_RTCM_1230, 10, 1);

  sendCfgMsg(0xF5, 0x05, 10);  // RTCM3 1005 - чаще (каждые 100 мс при 10Hz)
  sendCfgMsg(0xF5, 0x4A, 1);   // RTCM3 1074 GPS MSM4
  sendCfgMsg(0xF5, 0x54, 1);   // RTCM3 1084 GLONASS MSM4
  sendCfgMsg(0xF5, 0x5E, 1);   // RTCM3 1094 Galileo MSM4
  sendCfgMsg(0xF5, 0x7C, 1);   // RTCM3 1124 BeiDou MSM4
  sendCfgMsg(0xF5, 0xE6, 10);  // RTCM3 1230 GLONASS code phase biases
}

void configureGnssSignals() {
  Serial.println("BASE: configuring GNSS signals...");
  sendValset(CFG_SIGNAL_GPS_ENA, 1, 1);
  sendValset(CFG_SIGNAL_GPS_L1CA_ENA, 1, 1);
  sendValset(CFG_SIGNAL_GPS_L2C_ENA, 1, 1);
  sendValset(CFG_SIGNAL_GAL_ENA, 1, 1);
  sendValset(CFG_SIGNAL_GAL_E1_ENA, 1, 1);
  sendValset(CFG_SIGNAL_GAL_E5B_ENA, 1, 1);
  sendValset(CFG_SIGNAL_BDS_ENA, 1, 1);
  sendValset(CFG_SIGNAL_BDS_B1_ENA, 1, 1);
  sendValset(CFG_SIGNAL_BDS_B2_ENA, 1, 1);
  sendValset(CFG_SIGNAL_GLO_ENA, 1, 1);
  sendValset(CFG_SIGNAL_GLO_L1_ENA, 1, 1);
  sendValset(CFG_SIGNAL_GLO_L2_ENA, 1, 1);
}

// Принудительно включает UBX протокол (важно если GPS в NMEA режиме)
void forceUbxMode() {
  Serial.println("BASE: forcing UBX protocol mode...");

  // UBX-CFG-NMEA: отключаем NMEA режим, включаем UBX
  // b2=0x00 отключает compatibility mode, filter=0 фильтрует NMEA
  uint8_t nmeaPayload[12] = {0};
  nmeaPayload[0] = 0x00;  // fixme: major NMEA version (ignored if outProtoVersion=1)
  nmeaPayload[1] = 0x00;  // filter flags
  nmeaPayload[2] = 0x00;  // nmeaVersion = 0 = unknown/out
  nmeaPayload[3] = 0x00;  // flags
  nmeaPayload[4] = 0x00;  // inProtoMask: UBX only (0x01)
  nmeaPayload[5] = 0x01;  // (continued)
  nmeaPayload[6] = 0x00;  // outProtoMask: UBX only (0x01)
  nmeaPayload[7] = 0x01;  // (continued)
  nmeaPayload[8] = 0x00;  // outProtoVersion: 0 = current
  nmeaPayload[9] = 0x00;  // (continued)
  nmeaPayload[10] = 0x00; // bbr: 0
  nmeaPayload[11] = 0x00; // (continued)

  writeUbx(0x06, 0x41, nmeaPayload, sizeof(nmeaPayload));
  GpsSerial.flush();
  delay(100);

  // Дополнительно: UBX-CFG-PRT для UART с протоколами UBX
  // Port 1 = UART
  uint8_t cfgPrt[32];
  cfgPrt[0] = 0xB5; cfgPrt[1] = 0x62;  // sync
  cfgPrt[2] = 0x06; cfgPrt[3] = 0x00;  // class, id
  cfgPrt[4] = 0x14; cfgPrt[5] = 0x00;  // len = 20
  cfgPrt[6] = 0x01;  // portID = UART
  cfgPrt[7] = 0x00;  // reserved
  cfgPrt[8] = 0x00; cfgPrt[9] = 0x00;  // txReady
  cfgPrt[10] = 0xC0; cfgPrt[11] = 0x08; cfgPrt[12] = 0x00; cfgPrt[13] = 0x00;  // mode: 8N1
  cfgPrt[14] = (uint8_t)GPS_BAUD;
  cfgPrt[15] = (uint8_t)(GPS_BAUD >> 8);
  cfgPrt[16] = (uint8_t)(GPS_BAUD >> 16);
  cfgPrt[17] = (uint8_t)(GPS_BAUD >> 24);
  cfgPrt[18] = 0x01; cfgPrt[19] = 0x00;  // inProtoMask = UBX only
  cfgPrt[20] = 0x01; cfgPrt[21] = 0x00;  // outProtoMask = UBX only
  cfgPrt[22] = 0x00; cfgPrt[23] = 0x00;  // flags
  cfgPrt[24] = 0x00; cfgPrt[25] = 0x00;  // reserved
  uint8_t ckA = 0, ckB = 0;
  for (int i = 2; i <= 25; i++) { ckA += cfgPrt[i]; ckB += ckA; }
  cfgPrt[26] = ckA; cfgPrt[27] = ckB;
  GpsSerial.write(cfgPrt, 28);
  GpsSerial.flush();
  delay(100);

  Serial.println("BASE: UBX mode forced");
}

void configureGps() {
  Serial.println("BASE: === GPS CONFIGURATION ===");

  // ПЕРВОЕ: сбрасываем GPS конфигурацию (BBR)
  // Это КРИТИЧНО для сброса старого Survey-In режима
  Serial.println("BASE: resetting GPS config (BBR clear)...");
  uint8_t resetPayload[12] = {0};
  resetPayload[0] = 0xFF; resetPayload[1] = 0xFF;  // clearMask: IOPORT + MSG
  resetPayload[2] = 0x00; resetPayload[3] = 0x00;
  resetPayload[4] = 0x00; resetPayload[5] = 0x00;
  resetPayload[6] = 0x00; resetPayload[7] = 0x00;  // saveMask
  resetPayload[8] = 0x00; resetPayload[9] = 0x00;
  resetPayload[10] = 0x00; resetPayload[11] = 0x00;  // loadMask
  writeUbx(0x06, 0x09, resetPayload, 12);
  GpsSerial.flush();
  delay(200);
  Serial.println("BASE: GPS config reset done");

  // ВТОРОЕ: очищаем буфер UART
  while (GpsSerial.available()) GpsSerial.read();
  delay(100);

  // ТРЕТЬЕ: принудительно включаем UBX протокол
  // Это КРИТИЧНО - если GPS в NMEA режиме, все UBX команды игнорируются!
  forceUbxMode();

  // Survey-In: 10 минут, точность 1м
  configureGnssSignals();

  sendValset(CFG_TMODE_MODE, 0, 1);
  delay(100);
  sendValset(CFG_TMODE_SVIN_MIN_DUR, SVIN_MIN_DUR_S, 4);
  sendValset(CFG_TMODE_SVIN_ACC_LIMIT, SVIN_ACC_LIMIT_0_1MM, 4);

  configureRtcmOutput();

  // ВАЖНО: открываем выходной протокол для RTCM3 (UBX-CFG-PRT)
  // forceUbxMode() установил UBX only, теперь добавляем RTCM3
  sendCfgPrt(0x0001, 0x0021, "ubx+rtcm");  // UBX in, UBX + RTCM3 out

  sendValset(CFG_MSGOUT_UBX_NAV_SVIN_UART1, 0, 1);
  sendValset(CFG_MSGOUT_UBX_NAV_PVT_UART1, 0, 1);
  sendCfgMsg(0x01, 0x3B, 0);   // UBX-NAV-SVIN off on UART1
  sendCfgMsg(0x01, 0x07, 0);   // UBX-NAV-PVT off on UART1

  // Keep the base UART focused on UBX + RTCM. NMEA can create false D3 syncs
  // in the mixed binary stream and wastes bandwidth needed for MSM.
  sendValset(CFG_MSGOUT_NMEA_GGA_UART1, 0, 1);
  sendValset(CFG_MSGOUT_NMEA_GLL_UART1, 0, 1);
  sendValset(CFG_MSGOUT_NMEA_GSA_UART1, 0, 1);
  sendValset(CFG_MSGOUT_NMEA_GSV_UART1, 0, 1);
  sendValset(CFG_MSGOUT_NMEA_RMC_UART1, 0, 1);
  sendValset(CFG_MSGOUT_NMEA_VTG_UART1, 0, 1);

  // Measurement rate
  sendValset(CFG_RATE_MEAS, 1000, 2);

  // Enable Survey-In
  delay(200);
  sendValset(CFG_TMODE_MODE, 1, 1);
  sendCfgTmode3SurveyIn();
  writeUbx(0x01, 0x3B, nullptr, 0);

  // Очищаем буфер от мусора конфигурации
  delay(100);
  while (GpsSerial.available()) GpsSerial.read();
  rtcmLen = 0;
  rtcmExpectedLen = 0;
  Serial.println("BASE: GPS config complete, starting fresh");

  Serial.println("BASE: === GPS CONFIGURED ===");
}

static uint16_t rtcmPayloadLength(const uint8_t* frame) {
  return ((uint16_t)(frame[1] & 0x03) << 8) | frame[2];
}

static uint16_t rtcmMessageType(const uint8_t* frame, uint16_t len) {
  if (len < 5) return 0;
  return ((uint16_t)frame[3] << 4) | (frame[4] >> 4);
}

static void countRtcmType(uint16_t type) {
  switch (type) {
    case 1005: rtcmType1005++; break;
    case 1074: rtcmType1074++; break;
    case 1084: rtcmType1084++; break;
    case 1094: rtcmType1094++; break;
    case 1124: rtcmType1124++; break;
    case 1230: rtcmType1230++; break;
    default: rtcmTypeOther++; break;
  }
}

static uint32_t rtcmCrc24q(const uint8_t* data, uint16_t len) {
  uint32_t crc = 0;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= (uint32_t)data[i] << 16;
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc <<= 1;
      if (crc & 0x1000000) crc ^= 0x1864CFB;
    }
  }
  return crc & 0xFFFFFF;
}

static bool rtcmCrcOk(const uint8_t* frame, uint16_t len) {
  if (len < 6) return false;
  const uint32_t expected = ((uint32_t)frame[len - 3] << 16) |
                            ((uint32_t)frame[len - 2] << 8) |
                            frame[len - 1];
  return rtcmCrc24q(frame, len - 3) == expected;
}

static void resyncRtcmBuffer() {
  for (uint16_t i = 1; i + 2 < rtcmLen; i++) {
    if (rtcmBuf[i] != 0xD3) continue;
    if ((rtcmBuf[i + 1] & 0xFC) != 0) continue;
    const uint16_t payloadLen = ((uint16_t)(rtcmBuf[i + 1] & 0x03) << 8) | rtcmBuf[i + 2];
    const uint16_t expectedLen = payloadLen + 6;
    if (payloadLen == 0 || expectedLen > RTCM_BUF_SIZE) continue;

    const uint16_t remaining = rtcmLen - i;
    memmove(rtcmBuf, rtcmBuf + i, remaining);
    rtcmLen = remaining;
    rtcmExpectedLen = expectedLen;
    return;
  }

  rtcmLen = 0;
  rtcmExpectedLen = 0;
}

static void sendRtcmFrameBytes(const uint8_t* frame, uint16_t len) {
  if (!wifiConnected || len == 0) return;

  int result = udp.beginPacket(roverIP, UDP_PORT);
  if (result == 1) {
    udp.write(frame, len);
    result = udp.endPacket();
    if (result == 1) {
      rtcmPktsSent++;
      rtcmBytesSent += len;
      lastRtcmMs = millis();
      lastRtcmSendMs = lastRtcmMs;
      return;
    }
    rtcmErrors++;
    Serial.printf("BASE ERR udp_end result=%d len=%u type=%lu\n",
      result, len, (unsigned long)rtcmMessageType(frame, len));
    return;
  }

  rtcmErrors++;
  Serial.printf("BASE ERR udp_begin result=%d target=%s:%u len=%u\n",
    result, roverIP.toString().c_str(), UDP_PORT, len);
}

static void queueRtcmFrameForSend() {
  if (rtcmLen == 0) return;
  if (rtcmFrameQueueCount >= RTCM_FRAME_QUEUE_SIZE) {
    rtcmFrameQueueDrop++;
    return;
  }
  memcpy(rtcmFrameQueue[rtcmFrameQueueHead], rtcmBuf, rtcmLen);
  rtcmFrameQueueLen[rtcmFrameQueueHead] = rtcmLen;
  rtcmFrameQueueHead = (rtcmFrameQueueHead + 1) % RTCM_FRAME_QUEUE_SIZE;
  rtcmFrameQueueCount++;
}

static void flushRtcmFrameQueue() {
  while (rtcmFrameQueueCount > 0) {
    const uint8_t tail = rtcmFrameQueueTail;
    sendRtcmFrameBytes(rtcmFrameQueue[tail], rtcmFrameQueueLen[tail]);
    rtcmFrameQueueTail = (rtcmFrameQueueTail + 1) % RTCM_FRAME_QUEUE_SIZE;
    rtcmFrameQueueCount--;
  }
}

static void flushRawRtcmUdp() {
  if (rawRtcmUdpLen == 0) return;
  if (!wifiConnected) {
    rawRtcmUdpLen = 0;
    return;
  }

  int result = udp.beginPacket(roverIP, UDP_PORT);
  if (result == 1) {
    udp.write(rawRtcmUdpBuf, rawRtcmUdpLen);
    result = udp.endPacket();
    if (result == 1) {
      rawRtcmPktsSent++;
      rawRtcmBytesSent += rawRtcmUdpLen;
      rtcmPktsSent++;
      rtcmBytesSent += rawRtcmUdpLen;
      lastRtcmMs = millis();
      lastRtcmSendMs = lastRtcmMs;
      rawRtcmLastFlushMs = lastRtcmMs;
      rawRtcmUdpLen = 0;
      return;
    }
    rtcmErrors++;
    Serial.printf("BASE ERR raw_udp_end result=%d len=%u\n", result, rawRtcmUdpLen);
    rawRtcmUdpLen = 0;
    return;
  }

  rtcmErrors++;
  Serial.printf("BASE ERR raw_udp_begin result=%d target=%s:%u len=%u\n",
    result, roverIP.toString().c_str(), UDP_PORT, rawRtcmUdpLen);
  rawRtcmUdpLen = 0;
}

static void queueRawRtcmByte(uint8_t b) {
  rawRtcmUdpBuf[rawRtcmUdpLen++] = b;
  if (rawRtcmUdpLen >= RAW_RTCM_UDP_BUF_SIZE) {
    flushRawRtcmUdp();
  }
}

static void feedRtcmByte(uint8_t b, bool forwardFrame = true) {
  if (rtcmLen == 0) {
    if (b != 0xD3) return;
    rtcmBuf[rtcmLen++] = b;
    rtcmExpectedLen = 0;
    lastRtcmByteMs = millis();
    return;
  }

  if (rtcmLen == 1 && (b & 0xFC) != 0) {
    rtcmLen = 0;
    rtcmExpectedLen = 0;
    return;
  }

  if (rtcmLen >= RTCM_BUF_SIZE) {
    rtcmOverflow++;
    rtcmLen = 0;
    rtcmExpectedLen = 0;
    return;
  }

  rtcmBuf[rtcmLen++] = b;
  lastRtcmByteMs = millis();
  if (rtcmLen == 3) {
    const uint16_t payloadLen = rtcmPayloadLength(rtcmBuf);
    rtcmExpectedLen = payloadLen + 6;
    if (payloadLen == 0 || rtcmExpectedLen > RTCM_BUF_SIZE) {
      rtcmOverflow++;
      rtcmLen = 0;
      rtcmExpectedLen = 0;
      return;
    }
  }

  if (rtcmExpectedLen > 0 && rtcmLen >= rtcmExpectedLen) {
    if (rtcmCrcOk(rtcmBuf, rtcmLen)) {
      rtcm3Parsed++;
      rtcmLastType = rtcmMessageType(rtcmBuf, rtcmLen);
      countRtcmType(rtcmLastType);
      if (forwardFrame) {
        queueRtcmFrameForSend();
      }
      rtcmLen = 0;
      rtcmExpectedLen = 0;
    } else {
      rtcmCrcFail++;
      resyncRtcmBuffer();
    }
  }
}

bool parseUbx(uint8_t b) {
  gpsRawBytes++;

  // Capture first bytes for diagnosis
  if (!firstBytesPrinted && firstBytesCount < 20) {
    firstBytes[firstBytesCount++] = b;
    if (firstBytesCount == 20) {
      Serial.print("GPS FIRST 20 BYTES: ");
      for (int i = 0; i < 20; i++) {
        Serial.printf("%02X ", firstBytes[i]);
      }
      Serial.println();
      firstBytesPrinted = true;
    }
  }

  switch (ubxState) {
    case 0:  // SYNC1
      if (b == 0xB5) {
        ubxState = 1;
        return true;
      }
      return false;
    case 1:  // SYNC2
      if (b == 0x62) {
        ubxState = 2;
        ubxCkA = ubxCkB = 0;
        return true;
      }
      ubxState = (b == 0xB5) ? 1 : 0;
      return b == 0xB5;
    case 2:  // CLASS
      ubxClass = b;
      ubxCkA += b; ubxCkB += ubxCkA;
      ubxState = 3;
      return true;
    case 3:  // ID
      ubxId = b;
      ubxCkA += b; ubxCkB += ubxCkA;
      ubxState = 4;
      return true;
    case 4:  // LEN1
      ubxLen = b;
      ubxCkA += b; ubxCkB += ubxCkA;
      ubxState = 5;
      return true;
    case 5:  // LEN2
      ubxLen |= ((uint16_t)b << 8);
      ubxCkA += b; ubxCkB += ubxCkA;
      ubxCount = 0;
      ubxState = (ubxLen == 0 || ubxLen > sizeof(ubxPayload)) ? 7 : 6;
      return true;
    case 6:  // PAYLOAD
      ubxPayload[ubxCount++] = b;
      ubxCkA += b; ubxCkB += ubxCkA;
      if (ubxCount >= ubxLen) ubxState = 7;
      return true;
    case 7:  // CKA
      ubxState = (b == ubxCkA) ? 8 : 0;
      return true;
    case 8:  // CKB
      ubxState = 0;
      if (b == ubxCkB) {
        ubxParsed++;
        // NAV-SVIN (0x01 0x3B)
        if (ubxClass == 0x01 && ubxId == 0x3B && ubxLen >= 40) {
          svinDur = (uint32_t)ubxPayload[8] | ((uint32_t)ubxPayload[9] << 8) |
                    ((uint32_t)ubxPayload[10] << 16) | ((uint32_t)ubxPayload[11] << 24);
          svinAcc = (uint32_t)ubxPayload[28] | ((uint32_t)ubxPayload[29] << 8) |
                    ((uint32_t)ubxPayload[30] << 16) | ((uint32_t)ubxPayload[31] << 24);
          svinValid = ubxPayload[36];
          svinActive = ubxPayload[37];
          Serial.printf("BASE: SVIN active=%u valid=%u dur=%us acc=%.3fm\n",
            svinActive, svinValid, svinDur, svinAcc / 10000.0);
        }
        // MON-HW (0x0A 0x36) - показывает антенну
        if (ubxClass == 0x0A && ubxId == 0x36) {
          Serial.printf("BASE: MON-HW rxStat=0x%02X\n", ubxPayload[20]);
        }
      }
      return true;
  }
  return false;
}

void printStatus() {
  static uint32_t lastStatus = 0;
  uint32_t now = millis();
  if (now - lastStatus < STATUS_INTERVAL_MS) return;
  lastStatus = now;

  Serial.println();
  Serial.println("=== BASE STATUS ===");
  Serial.printf("WiFi: %s\n", wifiConnected ? "CONNECTED" : "DISCONNECTED");
  if (wifiConnected) {
    Serial.printf("  IP=%s RSSI=%d\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
  }
  Serial.printf("GPS: raw=%lu ubx=%lu\n", (unsigned long)gpsRawBytes, (unsigned long)ubxParsed);
  Serial.printf("RTCM: frames=%lu pkts=%lu bytes=%lu lastType=%lu age=%lums errors=%lu overflow=%lu crcFail=%lu pending=%u/%u\n",
    (unsigned long)rtcm3Parsed,
    (unsigned long)rtcmPktsSent,
    (unsigned long)rtcmBytesSent,
    (unsigned long)rtcmLastType,
    (unsigned long)(lastRtcmMs ? now - lastRtcmMs : 0),
    (unsigned long)rtcmErrors,
    (unsigned long)rtcmOverflow,
    (unsigned long)rtcmCrcFail,
    rtcmLen,
    rtcmExpectedLen);
  Serial.printf("RTCM raw: enabled=%u pkts=%lu bytes=%lu pendingUdp=%u\n",
    rtcmRawForwarding ? 1 : 0,
    (unsigned long)rawRtcmPktsSent,
    (unsigned long)rawRtcmBytesSent,
    rawRtcmUdpLen);
  Serial.printf("RTCM queue: pending=%u dropped=%lu\n",
    rtcmFrameQueueCount, (unsigned long)rtcmFrameQueueDrop);
  Serial.printf("RTCM types: 1005=%lu 1074=%lu 1084=%lu 1094=%lu 1124=%lu 1230=%lu other=%lu\n",
    (unsigned long)rtcmType1005,
    (unsigned long)rtcmType1074,
    (unsigned long)rtcmType1084,
    (unsigned long)rtcmType1094,
    (unsigned long)rtcmType1124,
    (unsigned long)rtcmType1230,
    (unsigned long)rtcmTypeOther);
  Serial.printf("SVIN: active=%u valid=%u dur=%us acc=%.3fm\n",
    svinActive, svinValid, svinDur, svinAcc / 10000.0);
  Serial.printf("Memory: free=%lu\n", (unsigned long)ESP.getFreeHeap());
  Serial.printf("BASE line wifi=%u ip=%s rssi=%d gpsRaw=%lu ubx=%lu svinActive=%u svinValid=%u svinDur=%lus svinAcc=%.3fm rtcmFrames=%lu rtcmPkts=%lu rtcmBytes=%lu rtcmRaw=%u rawPkts=%lu rawBytes=%lu rawPending=%u rtcmType=%lu rtcmAge=%lums t1005=%lu t1074=%lu t1084=%lu t1094=%lu t1124=%lu t1230=%lu tOther=%lu udpErr=%lu overflow=%lu crcFail=%lu heap=%lu\n",
    wifiConnected ? 1 : 0,
    wifiConnected ? WiFi.localIP().toString().c_str() : "-",
    wifiConnected ? WiFi.RSSI() : 0,
    (unsigned long)gpsRawBytes,
    (unsigned long)ubxParsed,
    svinActive,
    svinValid,
    (unsigned long)svinDur,
    svinAcc / 10000.0,
    (unsigned long)rtcm3Parsed,
    (unsigned long)rtcmPktsSent,
    (unsigned long)rtcmBytesSent,
    rtcmRawForwarding ? 1 : 0,
    (unsigned long)rawRtcmPktsSent,
    (unsigned long)rawRtcmBytesSent,
    rawRtcmUdpLen,
    (unsigned long)rtcmLastType,
    (unsigned long)(lastRtcmMs ? now - lastRtcmMs : 0),
    (unsigned long)rtcmType1005,
    (unsigned long)rtcmType1074,
    (unsigned long)rtcmType1084,
    (unsigned long)rtcmType1094,
    (unsigned long)rtcmType1124,
    (unsigned long)rtcmType1230,
    (unsigned long)rtcmTypeOther,
    (unsigned long)rtcmErrors,
    (unsigned long)rtcmOverflow,
    (unsigned long)rtcmCrcFail,
    (unsigned long)ESP.getFreeHeap());
  Serial.println("===================");
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("========================================");
  Serial.println("     RTK BASE STATION - DEBUG VERSION");
  Serial.println("========================================");
  Serial.printf("GPS UART default: RX=%d TX=%d alt RX=%d TX=%d %lu baud\n",
    PIN_GPS_RX, PIN_GPS_TX, PIN_GPS_ALT_RX, PIN_GPS_ALT_TX, (unsigned long)GPS_BAUD);
  Serial.printf("WiFi SSID: %s\n", WIFI_SSID);
  Serial.printf("Base static IP: %s\n", baseIP.toString().c_str());
  Serial.printf("RTCM target: %s:%u\n", roverIP.toString().c_str(), UDP_PORT);
  Serial.printf("Chip: %s rev=%d\n", ESP.getChipModel(), ESP.getChipRevision());
  Serial.printf("SDK: %s\n", ESP.getSdkVersion());
  Serial.printf("Reset reason: %s (%d)\n",
    resetReasonString(esp_reset_reason()), (int)esp_reset_reason());

  // GPS
  selectGpsUart();
  Serial.println("GPS UART initialized");

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  beginBaseWiFi();
  Serial.println("WiFi: connecting...");
}

void loop() {
  uint32_t now = millis();

  // WiFi status
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiConnected) {
      wifiConnected = true;
      wifiReconnectIntervalMs = WIFI_RECONNECT_INITIAL_MS;
      Serial.println();
      Serial.println("!!! WiFi CONNECTED !!!");
      Serial.printf("   IP: %s\n", WiFi.localIP().toString().c_str());
      Serial.printf("   RSSI: %d dBm\n", WiFi.RSSI());
      Serial.printf("   MAC: %s\n", WiFi.macAddress().c_str());

      // Start UDP
      if (udp.begin(UDP_PORT)) {
        Serial.printf("UDP: listening on port %u\n", UDP_PORT);
      } else {
        Serial.println("UDP: FAILED to begin!");
      }
    }
  } else {
    if (wifiConnected) {
      wifiConnected = false;
      Serial.println("!!! WiFi DISCONNECTED !!!");
    }
    if (now - lastWifiReconnectMs >= wifiReconnectIntervalMs) {
      lastWifiReconnectMs = now;
      Serial.printf("WiFi: reconnecting, next retry in %lu ms...\n",
        (unsigned long)min(wifiReconnectIntervalMs * 2, WIFI_RECONNECT_MAX_MS));
      if (!WiFi.reconnect()) {
        beginBaseWiFi();
      }
      wifiReconnectIntervalMs = min(wifiReconnectIntervalMs * 2, WIFI_RECONNECT_MAX_MS);
    }
  }

  // GPS configuration (once after WiFi connected)
  if (wifiConnected && !gpsConfigured) {
    delay(500);  // Wait for GPS to initialize
    configureGps();
    gpsConfigured = true;
  }

  // Read GPS data
  while (GpsSerial.available()) {
    uint8_t b = GpsSerial.read();

    if (rtcmRawForwarding) {
      gpsRawBytes++;
      queueRawRtcmByte(b);
      feedRtcmByte(b, false);
    } else if (rtcmFrameForwarding) {
      gpsRawBytes++;
      feedRtcmByte(b, true);
    } else if (rtcmLen > 0) {
      feedRtcmByte(b);
    } else if (!parseUbx(b)) {
      feedRtcmByte(b);
    }
  }

  if (rtcmFrameForwarding && rtcmFrameQueueCount > 0) {
    flushRtcmFrameQueue();
  }

  if (rtcmRawForwarding && rawRtcmUdpLen > 0 &&
      millis() - rawRtcmLastFlushMs >= RAW_RTCM_FLUSH_MS) {
    flushRawRtcmUdp();
  }

  if (!rtcmRawForwarding && rtcmLen > 0 && lastRtcmByteMs > 0 && now - lastRtcmByteMs > 1000) {
    rtcmCrcFail++;
    rtcmLen = 0;
    rtcmExpectedLen = 0;
  }

  if (svinValid && !rtcmConfiguredAfterSvin) {
    rtcmConfiguredAfterSvin = true;
    configureRtcmOutput();

    // Переключаем UART на UBX in + RTCM3 out (без UBX out чтобы не мешал)
    uint8_t cfgPrt[32];
    cfgPrt[0] = 0xB5; cfgPrt[1] = 0x62;
    cfgPrt[2] = 0x06; cfgPrt[3] = 0x00;
    cfgPrt[4] = 0x14; cfgPrt[5] = 0x00;
    cfgPrt[6] = 0x01;  // portID = UART
    cfgPrt[7] = 0x00;  // reserved
    cfgPrt[8] = 0x00; cfgPrt[9] = 0x00;  // txReady
    cfgPrt[10] = 0xC0; cfgPrt[11] = 0x08; cfgPrt[12] = 0x00; cfgPrt[13] = 0x00;  // mode: 8N1
    cfgPrt[14] = (uint8_t)GPS_BAUD;
    cfgPrt[15] = (uint8_t)(GPS_BAUD >> 8);
    cfgPrt[16] = (uint8_t)(GPS_BAUD >> 16);
    cfgPrt[17] = (uint8_t)(GPS_BAUD >> 24);
    cfgPrt[18] = 0x01; cfgPrt[19] = 0x00;  // inProtoMask = UBX only
    cfgPrt[20] = 0x20; cfgPrt[21] = 0x00;  // outProtoMask = RTCM3 only
    cfgPrt[22] = 0x00; cfgPrt[23] = 0x00;  // flags
    cfgPrt[24] = 0x00; cfgPrt[25] = 0x00;  // reserved
    uint8_t ckA = 0, ckB = 0;
    for (int i = 2; i <= 25; i++) { ckA += cfgPrt[i]; ckB += ckA; }
    cfgPrt[26] = ckA; cfgPrt[27] = ckB;
    GpsSerial.write(cfgPrt, 28);
    GpsSerial.flush();
    delay(200);
    Serial.println("BASE: switched to RTCM-only output mode");
    delay(200);
    while (GpsSerial.available()) {
      GpsSerial.read();
    }
    rtcmLen = 0;
    rtcmExpectedLen = 0;
    rtcmFrameQueueHead = 0;
    rtcmFrameQueueTail = 0;
    rtcmFrameQueueCount = 0;
    rawRtcmUdpLen = 0;
    rawRtcmLastFlushMs = millis();
    // Forward the RTCM byte stream directly. Full frame forwarding blocks long
    // enough to lose bytes from large MSM frames on this ESP32/F9P link.
    rtcmRawForwarding = true;
    rtcmFrameForwarding = false;
    Serial.println("BASE: raw RTCM forwarding enabled");
    svinPollingEnabled = false;
  }

  // Status output
  static uint32_t lastSvinPoll = 0;
  if (svinPollingEnabled && now - lastSvinPoll > 2000) {
    lastSvinPoll = now;
    writeUbx(0x01, 0x3B, nullptr, 0);
  }

  printStatus();
}
