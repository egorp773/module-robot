#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <WiFiUdp.h>

#if __has_include("rtk_config_private.h")
#include "rtk_config_private.h"
#else
#include "rtk_config.example.h"
#endif

static constexpr char GPS_PORT_NAME[] = "GPIO4/GPIO5";
static constexpr int PIN_GPS_RX = 4; // ESP32 RX <- F9P TX
static constexpr int PIN_GPS_TX = 5; // ESP32 TX -> F9P RX
static constexpr uint32_t GPS_BAUD = 38400;

static constexpr char ROUTER_WIFI_SSID[] = RTK_ROUTER_WIFI_SSID;
static constexpr char ROUTER_WIFI_PASS[] = RTK_ROUTER_WIFI_PASS;
#ifndef RTK_BASE_IP_A
#define RTK_BASE_IP_A 192
#define RTK_BASE_IP_B 168
#define RTK_BASE_IP_C 31
#define RTK_BASE_IP_D 207
#endif
static const IPAddress BASE_STA_IP(RTK_BASE_IP_A, RTK_BASE_IP_B,
                                   RTK_BASE_IP_C, RTK_BASE_IP_D);
static const IPAddress ROVER_IP(RTK_ROVER_IP_A, RTK_ROVER_IP_B,
                                RTK_ROVER_IP_C, RTK_ROVER_IP_D);
static const IPAddress ROUTER_GATEWAY(RTK_ROUTER_GATEWAY_A,
                                      RTK_ROUTER_GATEWAY_B,
                                      RTK_ROUTER_GATEWAY_C,
                                      RTK_ROUTER_GATEWAY_D);
static const IPAddress ROUTER_SUBNET(255, 255, 255, 0);
static const IPAddress RTCM_BROADCAST_IP(RTK_ROUTER_GATEWAY_A,
                                         RTK_ROUTER_GATEWAY_B,
                                         RTK_ROUTER_GATEWAY_C, 255);
static constexpr uint16_t ROVER_RTCM_PORT = 2101;
static constexpr uint16_t RTCM_TCP_PORT = 2102;

static constexpr uint32_t SURVEY_IN_SECONDS = 600;
static constexpr uint32_t SURVEY_IN_ACC_LIMIT_0_1MM = 10000; // 1.0 m, precision RTK base
static constexpr uint32_t SURVEY_IN_FALLBACK_AFTER_MS = 15UL * 60UL * 1000UL;
static constexpr uint32_t FALLBACK_SURVEY_IN_SECONDS = 60;
static constexpr uint32_t FALLBACK_SURVEY_IN_ACC_LIMIT_0_1MM = 250000; // 25.0 m, keeps RTK usable in weak sky
static constexpr uint32_t WIFI_RETRY_MS = 5000;
static constexpr uint32_t STATUS_MS = 1000;
static constexpr uint32_t SVIN_POLL_MS = 1000;
static constexpr uint32_t BASE_RECONFIG_MS = 15000;
static constexpr uint8_t UDP_FAIL_RECONNECT_LIMIT = 10;
static constexpr uint32_t UDP_RECONNECT_COOLDOWN_MS = 10000;
static constexpr uint32_t WIFI_AFTER_CONNECT_GRACE_MS = 5000;
static constexpr uint32_t RTCM_TCP_WRITE_TIMEOUT_MS = 25;
static constexpr uint8_t RTCM_BROADCAST_EVERY_N = 10;

// u-blox configuration keys for VALSET.
static constexpr uint32_t CFG_UART1INPROT_UBX = 0x10730001;
static constexpr uint32_t CFG_UART1INPROT_NMEA = 0x10730002;
static constexpr uint32_t CFG_UART1INPROT_RTCM3 = 0x10730004;
static constexpr uint32_t CFG_UART1OUTPROT_UBX = 0x10740001;
static constexpr uint32_t CFG_UART1OUTPROT_NMEA = 0x10740002;
static constexpr uint32_t CFG_UART1OUTPROT_RTCM3 = 0x10740004;
static constexpr uint32_t CFG_MSGOUT_UBX_NAV_PVT_UART1 = 0x20910007;
static constexpr uint32_t CFG_MSGOUT_UBX_NAV_SVIN_UART1 = 0x20910089;
static constexpr uint32_t CFG_RTCM_1005_UART1 = 0x209102BE;
static constexpr uint32_t CFG_RTCM_1074_UART1 = 0x2091035F;
static constexpr uint32_t CFG_RTCM_1084_UART1 = 0x20910364;
static constexpr uint32_t CFG_RTCM_1094_UART1 = 0x20910369;
static constexpr uint32_t CFG_RTCM_1124_UART1 = 0x2091036E;
static constexpr uint32_t CFG_RTCM_1230_UART1 = 0x20910304;
static constexpr uint32_t CFG_RTCM_4072_0_UART1 = 0x209102FF;
static constexpr uint32_t CFG_TMODE_MODE = 0x20030001;
static constexpr uint32_t CFG_TMODE_SVIN_MIN_DUR = 0x40030010;
static constexpr uint32_t CFG_TMODE_SVIN_ACC_LIMIT = 0x40030011;
static constexpr uint32_t CFG_RATE_MEAS = 0x30210001;
static constexpr uint32_t CFG_RATE_NAV = 0x30210002;
static constexpr uint32_t CFG_RATE_TIMEREF = 0x20210003;

static HardwareSerial GpsSerial(1);
static WiFiUDP rtcmUdp;
static WiFiServer rtcmTcpServer(RTCM_TCP_PORT);
static WiFiClient rtcmTcpClient;

static uint32_t lastWiFiAttemptMs = 0;
static uint32_t lastWiFiConnectedMs = 0;
static uint32_t lastUdpReconnectMs = 0;
static bool wasWiFiConnected = false;
static uint32_t lastStatusMs = 0;
static uint32_t lastSvinPollMs = 0;
static uint32_t lastReconfigMs = 0;
static uint32_t firstSurveyStartMs = 0;
static bool fallbackSurveyMode = false;
static uint32_t rtcmBytesSent = 0;
static uint32_t rtcmPacketsSent = 0;
static uint32_t udpSendOk = 0;
static uint32_t udpSendFail = 0;
static uint32_t udpBroadcastOk = 0;
static uint32_t udpBroadcastFail = 0;
static uint32_t tcpClientCount = 0;
static uint32_t tcpSendOk = 0;
static uint32_t tcpSendFail = 0;
static uint8_t tcpZeroWriteStreak = 0;
static uint8_t udpSendFailStreak = 0;
static uint32_t wifiReconnectCount = 0;
static uint32_t gnssRawBytes = 0;
static uint32_t gnssParsedMessages = 0;
static uint32_t ubxMessages = 0;
static uint32_t ackCount = 0;
static uint32_t nakCount = 0;
static uint32_t rtcmFramesSeen = 0;
static uint8_t lastAckClass = 0;
static uint8_t lastAckId = 0;
static int8_t lastAckResult = 0; // 1 ACK, -1 NAK, 0 none
static char lastConfigResult[96] = "not_configured";
static const char* gnssSource = "none";

struct GnssDebug {
  double lat = 0.0;
  double lon = 0.0;
  uint8_t fixType = 0;
  uint8_t numSv = 0;
  uint32_t lastMs = 0;
};

static GnssDebug gnss;

struct SvinStatus {
  bool active = false;
  bool valid = false;
  uint32_t dur = 0;
  uint32_t meanAcc01mm = 0;
  uint32_t lastMs = 0;
};

static SvinStatus svin;

static void putU16(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
}

static void putU32(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

static uint32_t getU32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int32_t getI32(const uint8_t* p) {
  return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                   ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

static void writeUbx(uint8_t cls, uint8_t id, const uint8_t* payload,
                     uint16_t len) {
  uint8_t ckA = 0;
  uint8_t ckB = 0;
  auto ck = [&](uint8_t b) {
    ckA += b;
    ckB += ckA;
  };

  GpsSerial.write(0xB5);
  GpsSerial.write(0x62);
  GpsSerial.write(cls);
  ck(cls);
  GpsSerial.write(id);
  ck(id);
  GpsSerial.write((uint8_t)len);
  ck((uint8_t)len);
  GpsSerial.write((uint8_t)(len >> 8));
  ck((uint8_t)(len >> 8));
  for (uint16_t i = 0; i < len; i++) {
    GpsSerial.write(payload[i]);
    ck(payload[i]);
  }
  GpsSerial.write(ckA);
  GpsSerial.write(ckB);
}

static void pollUbx(uint8_t cls, uint8_t id) {
  writeUbx(cls, id, nullptr, 0);
}

static bool waitForAck(uint8_t cls, uint8_t id, uint32_t timeoutMs);

static bool sendUbxWithAck(const char* label, uint8_t cls, uint8_t id,
                           const uint8_t* payload, uint16_t len) {
  lastAckResult = 0;
  lastAckClass = 0;
  lastAckId = 0;
  writeUbx(cls, id, payload, len);
  const bool ok = waitForAck(cls, id, 1200);
  snprintf(lastConfigResult, sizeof(lastConfigResult), "%s %s", label,
           ok ? "ACK" : (lastAckResult < 0 ? "NAK" : "NO_ACK"));
  Serial.printf("UBX %s cls=0x%02X id=0x%02X result=%s\n", label, cls, id,
                ok ? "ACK" : (lastAckResult < 0 ? "NAK" : "NO_ACK"));
  return ok;
}

struct ValItem {
  uint32_t key;
  uint32_t value;
  uint8_t size;
};

static bool sendValset(const char* label, const ValItem* items, size_t count) {
  uint8_t payload[160];
  size_t n = 0;
  payload[n++] = 0;    // version
  payload[n++] = 0x01; // RAM layer only
  payload[n++] = 0;
  payload[n++] = 0;

  for (size_t i = 0; i < count; i++) {
    putU32(payload + n, items[i].key);
    n += 4;
    for (uint8_t b = 0; b < items[i].size; b++) {
      payload[n++] = (uint8_t)(items[i].value >> (8 * b));
    }
  }

  return sendUbxWithAck(label, 0x06, 0x8A, payload, (uint16_t)n);
}

static uint32_t activeSurveySeconds() {
  return fallbackSurveyMode ? FALLBACK_SURVEY_IN_SECONDS : SURVEY_IN_SECONDS;
}

static uint32_t activeSurveyAccLimit01mm() {
  return fallbackSurveyMode ? FALLBACK_SURVEY_IN_ACC_LIMIT_0_1MM
                            : SURVEY_IN_ACC_LIMIT_0_1MM;
}

static bool sendCfgTmode3(const char* label, uint16_t flags) {
  uint8_t payload[40] = {};
  payload[0] = 0; // version
  putU16(payload + 2, flags);
  putU32(payload + 24, activeSurveySeconds());
  putU32(payload + 28, activeSurveyAccLimit01mm());
  return sendUbxWithAck(label, 0x06, 0x71, payload, sizeof(payload));
}

static void configureBase() {
  Serial.printf("GNSS: configuring base on %s baud %lu\n", GPS_PORT_NAME,
                (unsigned long)GPS_BAUD);

  const ValItem portItems[] = {
      {CFG_UART1INPROT_UBX, 1, 1},
      {CFG_UART1INPROT_NMEA, 1, 1},
      {CFG_UART1INPROT_RTCM3, 0, 1},
      {CFG_UART1OUTPROT_UBX, 1, 1},
      {CFG_UART1OUTPROT_NMEA, 0, 1},
      {CFG_UART1OUTPROT_RTCM3, 1, 1},
  };
  const bool portOk =
      sendValset("VALSET port-protocols", portItems,
                 sizeof(portItems) / sizeof(portItems[0]));

  const ValItem navItems[] = {
      {CFG_MSGOUT_UBX_NAV_PVT_UART1, 1, 1},
      {CFG_MSGOUT_UBX_NAV_SVIN_UART1, 1, 1},
      {CFG_RATE_MEAS, 1000, 2},
      {CFG_RATE_NAV, 1, 2},
      {CFG_RATE_TIMEREF, 1, 1},
  };
  const bool navOk =
      sendValset("VALSET nav-svin-rate", navItems,
                 sizeof(navItems) / sizeof(navItems[0]));

  const ValItem rtcmItems[] = {
      {CFG_RTCM_1005_UART1, 1, 1},   {CFG_RTCM_1074_UART1, 1, 1},
      {CFG_RTCM_1084_UART1, 1, 1},   {CFG_RTCM_1094_UART1, 1, 1},
      {CFG_RTCM_1124_UART1, 1, 1},   {CFG_RTCM_1230_UART1, 1, 1},
      {CFG_RTCM_4072_0_UART1, 1, 1},
  };
  const bool rtcmOk =
      sendValset("VALSET rtcm-output", rtcmItems,
                 sizeof(rtcmItems) / sizeof(rtcmItems[0]));

  const ValItem disableTmode[] = {
      {CFG_TMODE_MODE, 0, 1},
  };
  const bool disableValOk =
      sendValset("VALSET tmode-disable", disableTmode,
                 sizeof(disableTmode) / sizeof(disableTmode[0]));
  const bool disableLegacyOk = sendCfgTmode3("CFG-TMODE3 disable", 0);

  const ValItem surveyItems[] = {
      {CFG_TMODE_SVIN_MIN_DUR, activeSurveySeconds(), 4},
      {CFG_TMODE_SVIN_ACC_LIMIT, activeSurveyAccLimit01mm(), 4},
      {CFG_TMODE_MODE, 1, 1},
  };
  const bool surveyValOk =
      sendValset("VALSET survey-in", surveyItems,
                 sizeof(surveyItems) / sizeof(surveyItems[0]));
  const bool surveyLegacyOk = sendCfgTmode3("CFG-TMODE3 survey-in", 1);

  pollUbx(0x01, 0x3B);
  snprintf(lastConfigResult, sizeof(lastConfigResult),
           "port=%u nav=%u rtcm=%u tmode=%u/%u survey=%u/%u",
           portOk ? 1 : 0, navOk ? 1 : 0, rtcmOk ? 1 : 0,
           disableValOk ? 1 : 0, disableLegacyOk ? 1 : 0,
           surveyValOk ? 1 : 0, surveyLegacyOk ? 1 : 0);
  Serial.printf("GNSS: base config summary %s\n", lastConfigResult);
  lastReconfigMs = millis();
  if (firstSurveyStartMs == 0) firstSurveyStartMs = lastReconfigMs;
}

enum UbxState { U_SYNC1, U_SYNC2, U_CLASS, U_ID, U_LEN1, U_LEN2, U_PAYLOAD, U_CKA, U_CKB };
static UbxState ubxState = U_SYNC1;
static uint8_t ubxClass = 0;
static uint8_t ubxId = 0;
static uint16_t ubxLen = 0;
static uint16_t ubxCount = 0;
static uint8_t ubxPayload[128];
static uint8_t ubxCkA = 0;
static uint8_t ubxCkB = 0;

static double parseNmeaCoord(const char* value, const char* hemi) {
  if (value == nullptr || hemi == nullptr || value[0] == '\0') return 0.0;

  const double raw = atof(value);
  const int degrees = (int)(raw / 100.0);
  const double minutes = raw - (double)degrees * 100.0;
  double coord = (double)degrees + minutes / 60.0;
  if (hemi[0] == 'S' || hemi[0] == 'W') coord = -coord;
  return coord;
}

static int nmeaHex(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

static bool nmeaChecksumOk(const char* s) {
  if (s == nullptr || s[0] != '$') return false;
  const char* star = strchr(s, '*');
  if (star == nullptr || star[1] == '\0' || star[2] == '\0') return false;

  uint8_t ck = 0;
  for (const char* p = s + 1; p < star; p++) ck ^= (uint8_t)*p;

  const int hi = nmeaHex(star[1]);
  const int lo = nmeaHex(star[2]);
  if (hi < 0 || lo < 0) return false;
  return ck == (uint8_t)((hi << 4) | lo);
}

static void parseNmeaSentence(char* sentence) {
  if (!nmeaChecksumOk(sentence)) return;

  char* star = strchr(sentence, '*');
  if (star != nullptr) *star = '\0';

  char* fields[24] = {};
  size_t count = 0;
  char* p = sentence + 1;
  while (count < sizeof(fields) / sizeof(fields[0])) {
    fields[count++] = p;
    char* comma = strchr(p, ',');
    if (comma == nullptr) break;
    *comma = '\0';
    p = comma + 1;
  }
  if (count == 0 || fields[0] == nullptr || strlen(fields[0]) < 3) return;

  const char* type = fields[0] + strlen(fields[0]) - 3;
  if (strcmp(type, "GGA") == 0 && count > 7) {
    gnss.lat = parseNmeaCoord(fields[2], fields[3]);
    gnss.lon = parseNmeaCoord(fields[4], fields[5]);
    gnss.fixType = atoi(fields[6]) > 0 ? 3 : 0;
    gnss.numSv = (uint8_t)atoi(fields[7]);
    gnss.lastMs = millis();
    gnssParsedMessages++;
    gnssSource = "NMEA";
  } else if (strcmp(type, "RMC") == 0 && count > 6) {
    gnss.fixType = fields[2][0] == 'A' ? 3 : 0;
    gnss.lat = parseNmeaCoord(fields[3], fields[4]);
    gnss.lon = parseNmeaCoord(fields[5], fields[6]);
    gnss.lastMs = millis();
    gnssParsedMessages++;
    gnssSource = "NMEA";
  }
}

static void feedNmea(uint8_t b) {
  static char nmea[128];
  static uint8_t n = 0;
  static bool collecting = false;

  if (b == '$') {
    collecting = true;
    n = 0;
    nmea[n++] = (char)b;
    return;
  }
  if (!collecting) return;
  if (b == '\r') return;
  if (b == '\n') {
    nmea[n] = '\0';
    collecting = false;
    parseNmeaSentence(nmea);
    return;
  }
  if (n < sizeof(nmea) - 1) {
    nmea[n++] = (char)b;
  } else {
    collecting = false;
    n = 0;
  }
}

static void parseNavPvt(const uint8_t* p, uint16_t len) {
  if (len < 92) return;
  gnss.lon = (double)getI32(p + 24) * 1e-7;
  gnss.lat = (double)getI32(p + 28) * 1e-7;
  gnss.fixType = p[20];
  gnss.numSv = p[23];
  gnss.lastMs = millis();
  gnssParsedMessages++;
  gnssSource = "UBX";
}

static void processUbx() {
  ubxMessages++;

  if (ubxClass == 0x05 && (ubxId == 0x01 || ubxId == 0x00) && ubxLen >= 2) {
    lastAckClass = ubxPayload[0];
    lastAckId = ubxPayload[1];
    lastAckResult = ubxId == 0x01 ? 1 : -1;
    if (lastAckResult > 0) ackCount++;
    else nakCount++;
    return;
  }

  if (ubxClass == 0x01 && ubxId == 0x07) {
    parseNavPvt(ubxPayload, ubxLen);
    return;
  }

  if (ubxClass == 0x01 && ubxId == 0x3B && ubxLen >= 40) {
    svin.dur = getU32(ubxPayload + 8);
    svin.meanAcc01mm = getU32(ubxPayload + 28);
    svin.valid = ubxPayload[36] != 0;
    svin.active = ubxPayload[37] != 0;
    svin.lastMs = millis();
    gnssParsedMessages++;
    gnssSource = "UBX";
  }
}

static void feedUbx(uint8_t b) {
  switch (ubxState) {
    case U_SYNC1:
      if (b == 0xB5) ubxState = U_SYNC2;
      break;
    case U_SYNC2:
      if (b == 0x62) {
        ubxState = U_CLASS;
        ubxCkA = 0;
        ubxCkB = 0;
      } else {
        ubxState = U_SYNC1;
      }
      break;
    case U_CLASS:
      ubxClass = b;
      ubxCkA += b;
      ubxCkB += ubxCkA;
      ubxState = U_ID;
      break;
    case U_ID:
      ubxId = b;
      ubxCkA += b;
      ubxCkB += ubxCkA;
      ubxState = U_LEN1;
      break;
    case U_LEN1:
      ubxLen = b;
      ubxCkA += b;
      ubxCkB += ubxCkA;
      ubxState = U_LEN2;
      break;
    case U_LEN2:
      ubxLen |= ((uint16_t)b << 8);
      ubxCkA += b;
      ubxCkB += ubxCkA;
      ubxCount = 0;
      ubxState = (ubxLen == 0) ? U_CKA
                               : (ubxLen > sizeof(ubxPayload) ? U_SYNC1 : U_PAYLOAD);
      break;
    case U_PAYLOAD:
      ubxPayload[ubxCount++] = b;
      ubxCkA += b;
      ubxCkB += ubxCkA;
      if (ubxCount >= ubxLen) ubxState = U_CKA;
      break;
    case U_CKA:
      ubxState = (b == ubxCkA) ? U_CKB : U_SYNC1;
      break;
    case U_CKB:
      if (b == ubxCkB) processUbx();
      ubxState = U_SYNC1;
      break;
  }
}

static bool waitForAck(uint8_t cls, uint8_t id, uint32_t timeoutMs) {
  const uint32_t started = millis();
  while (millis() - started < timeoutMs) {
    while (GpsSerial.available()) {
      const uint8_t b = (uint8_t)GpsSerial.read();
      gnssRawBytes++;
      feedUbx(b);
      feedNmea(b);
      if (lastAckClass == cls && lastAckId == id) {
        return lastAckResult > 0;
      }
    }
    delay(2);
  }
  return false;
}

enum RtcmState { R_WAIT, R_LEN1, R_LEN2, R_BODY };
static RtcmState rtcmState = R_WAIT;
static uint8_t rtcmBuf[1100];
static uint16_t rtcmLen = 0;
static uint16_t rtcmTarget = 0;

static void connectWiFi(bool force = false);

static void updateRtcmTcpClient() {
  if (rtcmTcpClient && !rtcmTcpClient.connected()) {
    Serial.println("RTCM TCP client disconnected");
    rtcmTcpClient.stop();
  }

  WiFiClient next = rtcmTcpServer.available();
  if (!next) return;

  if (next.remoteIP() != ROVER_IP) {
    Serial.printf("RTCM TCP rejected non-rover client from %s:%u\n",
                  next.remoteIP().toString().c_str(),
                  next.remotePort());
    next.stop();
    return;
  }

  if (rtcmTcpClient && rtcmTcpClient.connected()) {
    rtcmTcpClient.stop();
  }
  rtcmTcpClient = next;
  rtcmTcpClient.setNoDelay(true);
  tcpClientCount++;
  Serial.printf("RTCM TCP client #%lu connected from %s:%u\n",
                (unsigned long)tcpClientCount,
                rtcmTcpClient.remoteIP().toString().c_str(),
                rtcmTcpClient.remotePort());
}

static bool sendRtcmTcp() {
  if (!rtcmTcpClient || !rtcmTcpClient.connected()) return false;

  size_t written = 0;
  const uint32_t started = millis();
  while (written < rtcmLen && millis() - started < RTCM_TCP_WRITE_TIMEOUT_MS) {
    const size_t n = rtcmTcpClient.write(rtcmBuf + written, rtcmLen - written);
    if (n == 0) {
      delay(1);
      continue;
    }
    written += n;
  }

  if (written == rtcmLen) {
    tcpSendOk++;
    tcpZeroWriteStreak = 0;
    return true;
  }

  tcpSendFail++;
  Serial.printf("WARN base TCP RTCM failed written=%u/%u fail=%lu\n",
                (unsigned)written, rtcmLen, (unsigned long)tcpSendFail);
  if (written > 0) {
    rtcmTcpClient.stop();
  } else {
    if (tcpZeroWriteStreak < 255) tcpZeroWriteStreak++;
    if (tcpZeroWriteStreak >= 5) {
      Serial.println("WARN base TCP zero-write streak: closing RTCM client");
      tcpZeroWriteStreak = 0;
      rtcmTcpClient.stop();
    }
  }
  return false;
}

static bool sendRtcmUdpTo(const IPAddress& target, const char* label,
                          uint32_t* okCount, uint32_t* failCount) {
  const int beginOk = rtcmUdp.beginPacket(target, ROVER_RTCM_PORT);
  if (beginOk == 1) {
    rtcmUdp.write(rtcmBuf, rtcmLen);
  }
  const int endOk = beginOk == 1 ? rtcmUdp.endPacket() : 0;
  if (endOk == 1) {
    (*okCount)++;
    return true;
  }

  (*failCount)++;
  if (*failCount <= 3 || (*failCount % 25) == 0) {
    Serial.printf("WARN base UDP RTCM %s failed begin=%d end=%d fail=%lu target=%s:%u\n",
                  label, beginOk, endOk, (unsigned long)*failCount,
                  target.toString().c_str(), ROVER_RTCM_PORT);
  }
  return false;
}

static void sendRtcmFrame() {
  if (WiFi.status() != WL_CONNECTED || rtcmLen == 0) return;

  const bool tcpOk = sendRtcmTcp();
  bool unicastOk = false;
  bool broadcastOk = false;
  if (!tcpOk) {
    unicastOk =
        sendRtcmUdpTo(ROVER_IP, "unicast", &udpSendOk, &udpSendFail);
    if ((rtcmFramesSeen % RTCM_BROADCAST_EVERY_N) == 0) {
      broadcastOk = sendRtcmUdpTo(RTCM_BROADCAST_IP, "broadcast",
                                  &udpBroadcastOk, &udpBroadcastFail);
    }
  }

  if (unicastOk || broadcastOk || tcpOk) {
    rtcmBytesSent += rtcmLen;
    rtcmPacketsSent++;
    udpSendFailStreak = 0;
  } else {
    if (udpSendFailStreak < 255) udpSendFailStreak++;
    const uint32_t now = millis();
    const bool wifiSettled =
        lastWiFiConnectedMs != 0 &&
        now - lastWiFiConnectedMs > WIFI_AFTER_CONNECT_GRACE_MS;
    const bool reconnectCooldownOk =
        lastUdpReconnectMs == 0 ||
        now - lastUdpReconnectMs > UDP_RECONNECT_COOLDOWN_MS;
    if (udpSendFailStreak >= UDP_FAIL_RECONNECT_LIMIT && wifiSettled &&
        reconnectCooldownOk) {
      Serial.println("WARN base UDP fail streak: forcing WiFi reconnect");
      udpSendFailStreak = 0;
      lastUdpReconnectMs = now;
      connectWiFi(true);
    }
  }
  rtcmFramesSeen++;
}

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
      rtcmTarget = (rtcmTarget | b) + 6; // header + payload + CRC
      if (rtcmTarget > sizeof(rtcmBuf)) {
        rtcmState = R_WAIT;
      } else {
        rtcmState = R_BODY;
      }
      break;
    case R_BODY:
      rtcmBuf[rtcmLen++] = b;
      if (rtcmLen >= rtcmTarget) {
        sendRtcmFrame();
        rtcmState = R_WAIT;
      }
      break;
  }
}

static void connectWiFi(bool force) {
  const uint32_t now = millis();
  if (WiFi.status() == WL_CONNECTED) {
    if (!wasWiFiConnected) {
      wasWiFiConnected = true;
      lastWiFiConnectedMs = now;
      udpSendFailStreak = 0;
      Serial.printf("WiFi STA connected ip=%s\n",
                    WiFi.localIP().toString().c_str());
    }
    if (!force) return;
  } else {
    wasWiFiConnected = false;
  }
  if (!force && lastWiFiAttemptMs != 0 &&
      now - lastWiFiAttemptMs < WIFI_RETRY_MS) {
    return;
  }
  lastWiFiAttemptMs = now;
  wifiReconnectCount++;
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_15dBm);
  WiFi.setAutoReconnect(true);
  WiFi.config(BASE_STA_IP, ROUTER_GATEWAY, ROUTER_SUBNET);
  if (force) {
    rtcmTcpClient.stop();
    rtcmUdp.stop();
    WiFi.disconnect(false);
    delay(50);
  }
  WiFi.begin(ROUTER_WIFI_SSID, ROUTER_WIFI_PASS);
  Serial.printf("WiFi STA reconnect #%lu to %s, base IP %s, rover RTCM target %s:%u\n",
                (unsigned long)wifiReconnectCount, ROUTER_WIFI_SSID,
                BASE_STA_IP.toString().c_str(),
                ROVER_IP.toString().c_str(),
                ROVER_RTCM_PORT);
}

static void pollSvin() {
  const uint32_t now = millis();
  if (now - lastSvinPollMs < SVIN_POLL_MS) return;
  lastSvinPollMs = now;
  pollUbx(0x01, 0x3B);
}

static const char* baseInactiveReason(uint32_t now) {
  if (gnssRawBytes == 0) return "no raw GNSS bytes; check F9P TX->ESP32 RX";
  if (gnss.lastMs == 0) return "no parsed NAV-PVT/NMEA yet; check UART baud/protocol";
  if (now - gnss.lastMs > 5000) return "GNSS data stale";
  if (gnss.numSv < 5) return "waiting for satellites/open sky";
  if (svin.lastMs == 0) return "no NAV-SVIN yet; checking UBX output";
  if (now - svin.lastMs > 5000) return "NAV-SVIN stale";
  if (lastAckResult < 0) return "last UBX command was NAK";
  return "TMODE survey-in not active; reconfig will retry";
}

static void maybeReconfigureBase() {
  const uint32_t now = millis();
  if (svin.active || svin.valid) return;
  if (now - lastReconfigMs < BASE_RECONFIG_MS) return;

  Serial.printf("GNSS: Survey-In inactive: %s; retrying base config\n",
                baseInactiveReason(now));
  configureBase();
}

static void maybeFallbackSurveyMode() {
  if (fallbackSurveyMode || svin.valid || firstSurveyStartMs == 0) return;

  const uint32_t now = millis();
  const bool appTimerExpired =
      now - firstSurveyStartMs >= SURVEY_IN_FALLBACK_AFTER_MS;
  const bool gnssSurveyStuck =
      svin.active && svin.dur >= (SURVEY_IN_FALLBACK_AFTER_MS / 1000UL) &&
      svin.meanAcc01mm > SURVEY_IN_ACC_LIMIT_0_1MM;
  if (!appTimerExpired && !gnssSurveyStuck) return;

  fallbackSurveyMode = true;
  Serial.printf("GNSS: precision survey stuck %.3fm for %lus; switching to fallback survey %lus %.1fm\n",
                (double)svin.meanAcc01mm / 10000.0,
                (unsigned long)((now - firstSurveyStartMs) / 1000),
                (unsigned long)FALLBACK_SURVEY_IN_SECONDS,
                (double)FALLBACK_SURVEY_IN_ACC_LIMIT_0_1MM / 10000.0);
  configureBase();
}

static void printStatus() {
  const uint32_t now = millis();
  if (now - lastStatusMs < STATUS_MS) return;
  lastStatusMs = now;
  const uint32_t gnssAgeMs = gnss.lastMs == 0 ? 0 : now - gnss.lastMs;
  const uint32_t svinAgeMs = svin.lastMs == 0 ? 0 : now - svin.lastMs;
  Serial.printf("BASE wifi=%s ip=%s rover=%s:%u bcast=%s:%u tcpClient=%u tcpOk=%lu tcpFail=%lu wifiReconnect=%lu mode=%s survey=%lus/%.1fm port=%s baud=%lu src=%s raw=%lu parsed=%lu ubx=%lu ack=%lu nak=%lu fix=%u sv=%u gnssAge=%lums svin_active=%u svin_valid=%u dur=%lus meanAcc=%.3fm svinAge=%lums rtcm=%lubytes/%lupkts frames=%lu udpOk=%lu udpFail=%lu bcastOk=%lu bcastFail=%lu udpFailStreak=%u last=%s\n",
                WiFi.status() == WL_CONNECTED ? "connected" : "not_connected",
                WiFi.localIP().toString().c_str(),
                ROVER_IP.toString().c_str(),
                ROVER_RTCM_PORT,
                RTCM_BROADCAST_IP.toString().c_str(),
                ROVER_RTCM_PORT,
                rtcmTcpClient && rtcmTcpClient.connected() ? 1 : 0,
                (unsigned long)tcpSendOk,
                (unsigned long)tcpSendFail,
                (unsigned long)wifiReconnectCount,
                fallbackSurveyMode ? "fallback" : "precision",
                (unsigned long)activeSurveySeconds(),
                (double)activeSurveyAccLimit01mm() / 10000.0,
                GPS_PORT_NAME,
                (unsigned long)GPS_BAUD,
                gnssSource,
                (unsigned long)gnssRawBytes,
                (unsigned long)gnssParsedMessages,
                (unsigned long)ubxMessages,
                (unsigned long)ackCount,
                (unsigned long)nakCount,
                gnss.fixType,
                gnss.numSv,
                (unsigned long)gnssAgeMs,
                svin.active ? 1 : 0,
                svin.valid ? 1 : 0,
                (unsigned long)svin.dur,
                (double)svin.meanAcc01mm / 10000.0,
                (unsigned long)svinAgeMs,
                (unsigned long)rtcmBytesSent,
                (unsigned long)rtcmPacketsSent,
                (unsigned long)rtcmFramesSeen,
                (unsigned long)udpSendOk,
                (unsigned long)udpSendFail,
                (unsigned long)udpBroadcastOk,
                (unsigned long)udpBroadcastFail,
                udpSendFailStreak,
                lastConfigResult);
  if (!svin.active && !svin.valid) {
    Serial.printf("BASE reason=%s\n", baseInactiveReason(now));
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("ESP32 ZED-F9P RTK BASE");
  Serial.printf("GNSS: UART %s baud %lu\n", GPS_PORT_NAME,
                (unsigned long)GPS_BAUD);

  GpsSerial.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  delay(500);
  configureBase();
  connectWiFi();
  rtcmTcpServer.begin();
  rtcmTcpServer.setNoDelay(true);
  Serial.printf("RTCM TCP server port %u\n", RTCM_TCP_PORT);
}

void loop() {
  connectWiFi();
  updateRtcmTcpClient();
  while (GpsSerial.available()) {
    const uint8_t b = (uint8_t)GpsSerial.read();
    gnssRawBytes++;
    feedUbx(b);
    feedNmea(b);
    feedRtcm(b);
  }
  pollSvin();
  maybeFallbackSurveyMode();
  maybeReconfigureBase();
  printStatus();
}
