#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#if __has_include("rtk_config_private.h")
#include "rtk_config_private.h"
#else
#include "rtk_config.example.h"
#endif

static constexpr uint32_t GPS_BAUDS[] = {38400, 9600, 115200};

struct GpsPortConfig {
  const char* name;
  int rx; // ESP32 RX <- F9P TX
  int tx; // ESP32 TX -> F9P RX
};

static constexpr GpsPortConfig GPS_PORTS[] = {
    {"GPIO4/GPIO5", 4, 5},
    {"GPIO16/GPIO17", 16, 17},
};

static constexpr bool ROUTER_MODE = true;
static constexpr char ROUTER_WIFI_SSID[] = RTK_ROUTER_WIFI_SSID;
static constexpr char ROUTER_WIFI_PASS[] = RTK_ROUTER_WIFI_PASS;
static const IPAddress ROVER_STA_IP(RTK_ROVER_IP_A, RTK_ROVER_IP_B,
                                    RTK_ROVER_IP_C, RTK_ROVER_IP_D);
static const IPAddress ROUTER_GATEWAY(RTK_ROUTER_GATEWAY_A,
                                      RTK_ROUTER_GATEWAY_B,
                                      RTK_ROUTER_GATEWAY_C,
                                      RTK_ROUTER_GATEWAY_D);
static const IPAddress ROUTER_SUBNET(255, 255, 255, 0);

static constexpr char AP_WIFI_SSID[] = "RTK-Rover";
static constexpr char AP_WIFI_PASS[] = "rtk-rover-123";
static constexpr uint16_t WS_PORT = 81;
static constexpr uint16_t RTCM_UDP_PORT = 2101;
static constexpr uint32_t GPS_BROADCAST_MS = 200;
static constexpr uint32_t STATUS_MS = 1000;
static constexpr uint32_t WIFI_RETRY_MS = 3000;

static constexpr uint32_t CFG_UART1INPROT_UBX = 0x11731001;
static constexpr uint32_t CFG_UART1INPROT_NMEA = 0x11731002;
static constexpr uint32_t CFG_UART1INPROT_RTCM3 = 0x11731004;
static constexpr uint32_t CFG_UART1OUTPROT_UBX = 0x11741001;
static constexpr uint32_t CFG_UART1OUTPROT_NMEA = 0x11741002;
static constexpr uint32_t CFG_UART1OUTPROT_RTCM3 = 0x11741004;
static constexpr uint32_t CFG_MSGOUT_UBX_NAV_PVT_UART1 = 0x21912007;
static constexpr uint32_t CFG_RATE_MEAS = 0x31213001;
static constexpr uint32_t CFG_RATE_NAV = 0x31213002;
static constexpr uint32_t CFG_RATE_TIMEREF = 0x2121C003;

struct GpsFix {
  double lat = 0.0;
  double lon = 0.0;
  float heightM = 0.0f;
  float heading = 0.0f;
  float speedMps = 0.0f;
  float pDop = 0.0f;
  uint8_t fixType = 0;
  uint8_t carrier = 0;
  bool diff = false;
  uint8_t numSv = 0;
  uint32_t hAccMm = 0;
  uint32_t vAccMm = 0;
  uint32_t lastMs = 0;
  bool valid = false;
};

static HardwareSerial GpsSerial(1);
static AsyncWebServer server(WS_PORT);
static AsyncWebSocket ws("/ws");
static WiFiUDP rtcmUdp;
static GpsFix gps;
static uint32_t rtcmBytesRx = 0;
static uint32_t rtcmPacketsRx = 0;
static uint32_t lastRtcmMs = 0;
static uint32_t lastGpsBroadcastMs = 0;
static uint32_t lastStatusMs = 0;
static uint32_t lastWiFiAttemptMs = 0;
static uint32_t activeGpsBaud = 0;
static const char* activeGpsPort = GPS_PORTS[0].name;
static uint32_t gpsRawBytes = 0;
static uint32_t gpsParsedMessages = 0;
static const char* gpsSource = "none";

static void putU32(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

static int32_t getI32(const uint8_t* p) {
  return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                   ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

static uint32_t getU32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint16_t getU16(const uint8_t* p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
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

struct ValItem {
  uint32_t key;
  uint32_t value;
  uint8_t size;
};

static void sendValset(const ValItem* items, size_t count) {
  uint8_t payload[160];
  size_t n = 0;
  payload[n++] = 0;
  payload[n++] = 0x01;
  payload[n++] = 0;
  payload[n++] = 0;

  for (size_t i = 0; i < count; i++) {
    putU32(payload + n, items[i].key);
    n += 4;
    for (uint8_t b = 0; b < items[i].size; b++) {
      payload[n++] = (uint8_t)(items[i].value >> (8 * b));
    }
  }

  writeUbx(0x06, 0x8A, payload, (uint16_t)n);
  delay(80);
}

static void configureRover() {
  const ValItem portItems[] = {
      {CFG_UART1INPROT_UBX, 1, 1},
      {CFG_UART1INPROT_NMEA, 1, 1},
      {CFG_UART1INPROT_RTCM3, 1, 1},
      {CFG_UART1OUTPROT_UBX, 1, 1},
      {CFG_UART1OUTPROT_NMEA, 0, 1},
      {CFG_UART1OUTPROT_RTCM3, 0, 1},
      {CFG_MSGOUT_UBX_NAV_PVT_UART1, 1, 1},
      {CFG_RATE_MEAS, 200, 2},
      {CFG_RATE_NAV, 1, 2},
      {CFG_RATE_TIMEREF, 1, 1},
  };
  sendValset(portItems, sizeof(portItems) / sizeof(portItems[0]));
  Serial.println("GNSS: rover NAV-PVT and RTCM input configured");
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

static const char* carrierName(uint8_t carrier) {
  if (carrier == 1) return "float";
  if (carrier == 2) return "fixed";
  return "none";
}

static void parseNavPvt(const uint8_t* p, uint16_t len) {
  if (len < 92) return;
  gps.lon = (double)getI32(p + 24) * 1e-7;
  gps.lat = (double)getI32(p + 28) * 1e-7;
  gps.heightM = (float)getI32(p + 36) / 1000.0f;
  gps.hAccMm = getU32(p + 40);
  gps.vAccMm = getU32(p + 44);
  gps.speedMps = (float)getI32(p + 60) / 1000.0f;
  gps.heading = (float)getI32(p + 64) * 1e-5f;
  gps.pDop = (float)getU16(p + 76) * 0.01f;
  gps.fixType = p[20];
  gps.diff = (p[21] & 0x02) != 0;
  gps.carrier = (p[21] >> 6) & 0x03;
  gps.numSv = p[23];
  gps.valid = (p[21] & 0x01) != 0;
  gps.lastMs = millis();
  gpsParsedMessages++;
  gpsSource = "UBX";
}

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
  if (count == 0 || fields[0] == nullptr) return;

  if (strlen(fields[0]) < 3) return;
  const char* type = fields[0] + strlen(fields[0]) - 3;
  if (strcmp(type, "GGA") == 0 && count > 9) {
    const int quality = atoi(fields[6]);
    gps.lat = parseNmeaCoord(fields[2], fields[3]);
    gps.lon = parseNmeaCoord(fields[4], fields[5]);
    gps.fixType = quality > 0 ? 3 : 0;
    gps.diff = quality == 2 || quality == 4 || quality == 5;
    gps.carrier = quality == 4 ? 2 : (quality == 5 ? 1 : 0);
    gps.numSv = (uint8_t)atoi(fields[7]);
    gps.pDop = (float)atof(fields[8]);
    gps.heightM = (float)atof(fields[9]);
    gps.valid = quality > 0;
    gps.lastMs = millis();
    gpsParsedMessages++;
    gpsSource = "NMEA";
    return;
  }

  if (strcmp(type, "RMC") == 0 && count > 8) {
    gps.valid = fields[2][0] == 'A';
    gps.lat = parseNmeaCoord(fields[3], fields[4]);
    gps.lon = parseNmeaCoord(fields[5], fields[6]);
    gps.speedMps = (float)(atof(fields[7]) * 0.514444);
    gps.heading = (float)atof(fields[8]);
    if (!gps.valid) gps.fixType = 0;
    gps.lastMs = millis();
    gpsParsedMessages++;
    gpsSource = "NMEA";
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

static void processUbx() {
  if (ubxClass == 0x01 && ubxId == 0x07) {
    parseNavPvt(ubxPayload, ubxLen);
  }
}

static void feedUbx(uint8_t b);

static bool waitForPvt(uint32_t timeoutMs, uint32_t* bytesSeen) {
  const uint32_t started = millis();
  const uint32_t previousGpsMs = gps.lastMs;
  uint32_t bytes = 0;

  while (millis() - started < timeoutMs) {
    while (GpsSerial.available()) {
      bytes++;
      gpsRawBytes++;
      const uint8_t b = (uint8_t)GpsSerial.read();
      feedUbx(b);
      feedNmea(b);
    }
    if (gps.lastMs != 0 && gps.lastMs != previousGpsMs) {
      if (bytesSeen != nullptr) *bytesSeen = bytes;
      return true;
    }
    delay(2);
  }

  if (bytesSeen != nullptr) *bytesSeen = bytes;
  return false;
}

static bool startGpsAtConfig(const GpsPortConfig& port, uint32_t baud) {
  Serial.printf("GNSS: trying UART1 %s baud %lu\n", port.name,
                (unsigned long)baud);
  GpsSerial.end();
  delay(80);
  GpsSerial.begin(baud, SERIAL_8N1, port.rx, port.tx);
  delay(300);
  configureRover();

  uint32_t bytesSeen = 0;
  if (waitForPvt(2200, &bytesSeen)) {
    activeGpsBaud = baud;
    activeGpsPort = port.name;
    Serial.printf("GNSS: %s detected on %s at %lu baud, raw=%lu parsed=%lu\n",
                  gpsSource, port.name, (unsigned long)baud,
                  (unsigned long)bytesSeen, (unsigned long)gpsParsedMessages);
    return true;
  }

  Serial.printf("GNSS: no GPS parse on %s at %lu baud, raw=%lu\n", port.name,
                (unsigned long)baud, (unsigned long)bytesSeen);
  return false;
}

static void autoDetectGpsBaud() {
  const GpsPortConfig* bestPort = &GPS_PORTS[0];
  uint32_t bestBaud = GPS_BAUDS[0];
  uint32_t bestRaw = 0;
  const uint32_t rawBefore = gpsRawBytes;

  for (const GpsPortConfig& port : GPS_PORTS) {
    for (uint32_t baud : GPS_BAUDS) {
      const uint32_t before = gpsRawBytes;
      if (startGpsAtConfig(port, baud)) {
        return;
      }
      const uint32_t rawDelta = gpsRawBytes - before;
      if (rawDelta > bestRaw) {
        bestRaw = rawDelta;
        bestPort = &port;
        bestBaud = baud;
      }
    }
  }

  activeGpsBaud = bestBaud;
  activeGpsPort = bestPort->name;
  GpsSerial.end();
  delay(80);
  GpsSerial.begin(activeGpsBaud, SERIAL_8N1, bestPort->rx, bestPort->tx);
  delay(300);
  configureRover();
  Serial.printf("GNSS: GPS messages not detected, staying on %s at %lu baud, total raw=%lu\n",
                activeGpsPort, (unsigned long)activeGpsBaud,
                (unsigned long)(gpsRawBytes - rawBefore));
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

static void relayRtcmToF9p() {
  const int packetSize = rtcmUdp.parsePacket();
  if (packetSize <= 0) return;

  uint8_t buf[512];
  const int len = rtcmUdp.read(buf, sizeof(buf));
  if (len <= 0) return;

  GpsSerial.write(buf, len);
  rtcmBytesRx += len;
  rtcmPacketsRx++;
  lastRtcmMs = millis();
}

static void broadcastGps() {
  const uint32_t now = millis();
  if (now - lastGpsBroadcastMs < GPS_BROADCAST_MS) return;
  lastGpsBroadcastMs = now;

  const uint32_t gpsAgeMs = gps.lastMs == 0 ? 0 : now - gps.lastMs;
  const uint32_t rtcmAgeMs = lastRtcmMs == 0 ? 0 : now - lastRtcmMs;

  char msg[192];
  snprintf(msg, sizeof(msg), "GPS,%.8f,%.8f,%.2f,%u,%lu",
           gps.lat, gps.lon, gps.heading, gps.fixType,
           (unsigned long)gps.hAccMm);
  ws.textAll(msg);

  snprintf(msg, sizeof(msg),
           "GPSDBG,%.8f,%.8f,%.3f,%.2f,%u,%s,%u,%u,%lu,%lu,%.3f,%.2f,%lu",
           gps.lat, gps.lon, gps.heightM, gps.heading, gps.fixType,
           carrierName(gps.carrier), gps.diff ? 1 : 0, gps.numSv,
           (unsigned long)gps.hAccMm, (unsigned long)gps.vAccMm,
           gps.speedMps, gps.pDop, (unsigned long)gpsAgeMs);
  ws.textAll(msg);

  snprintf(msg, sizeof(msg), "RTCM,%lu,%lu",
           (unsigned long)rtcmBytesRx, (unsigned long)rtcmAgeMs);
  ws.textAll(msg);
}

static void connectWiFi() {
  if (!ROUTER_MODE) return;
  if (WiFi.status() == WL_CONNECTED) return;

  const uint32_t now = millis();
  if (lastWiFiAttemptMs != 0 && now - lastWiFiAttemptMs < WIFI_RETRY_MS) return;
  lastWiFiAttemptMs = now;

  WiFi.mode(WIFI_STA);
  WiFi.config(ROVER_STA_IP, ROUTER_GATEWAY, ROUTER_SUBNET);
  WiFi.begin(ROUTER_WIFI_SSID, ROUTER_WIFI_PASS);
  Serial.printf("WiFi STA connecting to %s, static IP %s\n",
                ROUTER_WIFI_SSID, ROVER_STA_IP.toString().c_str());
}

static void setupWeb() {
  if (ROUTER_MODE) {
    connectWiFi();
    Serial.printf("Router mode: WebSocket will be ws://%s:%u/ws\n",
                  ROVER_STA_IP.toString().c_str(), WS_PORT);
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_WIFI_SSID, AP_WIFI_PASS);
    Serial.printf("AP %s IP %s\n", AP_WIFI_SSID,
                  WiFi.softAPIP().toString().c_str());
  }

  ws.onEvent([](AsyncWebSocket*, AsyncWebSocketClient* client, AwsEventType type,
                void*, uint8_t* data, size_t len) {
    if (type == WS_EVT_CONNECT) {
      client->text("STATE,CONNECTED");
      return;
    }
    if (type != WS_EVT_DATA || len == 0) return;

    String msg;
    msg.reserve(len);
    for (size_t i = 0; i < len; i++) msg += (char)data[i];
    msg.trim();
    msg.toUpperCase();

    if (msg == "PING") client->text("PONG");
    else if (msg == "GPS_STATUS") client->text(gps.valid ? "OK GPS" : "NO GPS");
    else client->text("ERR,UNKNOWN");
  });

  server.addHandler(&ws);
  server.on("/ping", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send(200, "text/plain", "OK");
  });
  server.begin();
}

static void printStatus() {
  const uint32_t now = millis();
  if (now - lastStatusMs < STATUS_MS) return;
  lastStatusMs = now;

  const uint32_t gpsAgeMs = gps.lastMs == 0 ? 0 : now - gps.lastMs;
  const uint32_t rtcmAgeMs = lastRtcmMs == 0 ? 0 : now - lastRtcmMs;
  Serial.printf("ROVER wifi=%s ip=%s port=%s baud=%lu src=%s parsed=%lu clients=%u fix=%u carrier=%s diff=%u sv=%u hAcc=%lumm gpsAge=%lums raw=%lu rtcm=%lubytes/%lupkts age=%lums\n",
                WiFi.status() == WL_CONNECTED ? "connected" : "not_connected",
                WiFi.localIP().toString().c_str(),
                activeGpsPort,
                (unsigned long)activeGpsBaud,
                gpsSource,
                (unsigned long)gpsParsedMessages,
                ws.count(),
                gps.fixType,
                carrierName(gps.carrier),
                gps.diff ? 1 : 0,
                gps.numSv,
                (unsigned long)gps.hAccMm,
                (unsigned long)gpsAgeMs,
                (unsigned long)gpsRawBytes,
                (unsigned long)rtcmBytesRx,
                (unsigned long)rtcmPacketsRx,
                (unsigned long)rtcmAgeMs);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("ESP32 ZED-F9P RTK ROVER");

  setupWeb();
  rtcmUdp.begin(RTCM_UDP_PORT);
  Serial.printf("RTCM UDP input port %u\n", RTCM_UDP_PORT);

  autoDetectGpsBaud();
}

void loop() {
  connectWiFi();
  relayRtcmToF9p();
  while (GpsSerial.available()) {
    gpsRawBytes++;
    const uint8_t b = (uint8_t)GpsSerial.read();
    feedUbx(b);
    feedNmea(b);
  }

  broadcastGps();
  printStatus();
  ws.cleanupClients();
}
