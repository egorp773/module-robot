#include "gps.h"
#include "config.h"
#include <WiFi.h>
#include <WiFiUdp.h>

GpsData g_gpsData = {0};
uint32_t g_lastGpsMs = 0;

static WiFiUDP g_rtcmUdp;
static HardwareSerial GpsSerial(1);

// UBX parser state
enum UbxState {
  SYNC1,
  SYNC2,
  CLASS,
  ID,
  LEN1,
  LEN2,
  PAYLOAD,
  CK_A,
  CK_B
};

static UbxState ubxState = SYNC1;
static uint8_t ubxClass = 0;
static uint8_t ubxId = 0;
static uint16_t ubxLen = 0;
static uint16_t ubxCount = 0;
static uint8_t ubxPayload[128];
static uint8_t ubxCkA = 0;
static uint8_t ubxCkB = 0;

static inline int32_t readI32(const uint8_t* p) {
  return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

static inline uint32_t readU32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void parseNavPvt(const uint8_t* payload, uint16_t len) {
  if (len < 92) return;

  int32_t lonRaw = readI32(payload + 24);
  int32_t latRaw = readI32(payload + 28);
  int32_t headMotRaw = readI32(payload + 64);
  uint8_t fixType = payload[20];
  uint8_t flags = payload[21];
  uint32_t hAcc = readU32(payload + 40);

  g_gpsData.lon = (double)lonRaw * 1e-7;
  g_gpsData.lat = (double)latRaw * 1e-7;
  g_gpsData.headMot = (float)headMotRaw * 1e-5f;
  g_gpsData.fixType = fixType;
  g_gpsData.diffSoln = (flags & 0x02) != 0;
  g_gpsData.hAcc = hAcc;
  g_gpsData.valid = (fixType >= 3);

  g_lastGpsMs = millis();
}

static void processUbxMessage() {
  if (ubxClass == 0x01 && ubxId == 0x07) {
    parseNavPvt(ubxPayload, ubxLen);
  }
}

static void feedUbxByte(uint8_t b) {
  switch (ubxState) {
    case SYNC1:
      if (b == 0xB5) ubxState = SYNC2;
      break;
    case SYNC2:
      if (b == 0x62) {
        ubxState = CLASS;
        ubxCkA = 0;
        ubxCkB = 0;
      } else {
        ubxState = SYNC1;
      }
      break;
    case CLASS:
      ubxClass = b;
      ubxCkA += b;
      ubxCkB += ubxCkA;
      ubxState = ID;
      break;
    case ID:
      ubxId = b;
      ubxCkA += b;
      ubxCkB += ubxCkA;
      ubxState = LEN1;
      break;
    case LEN1:
      ubxLen = b;
      ubxCkA += b;
      ubxCkB += ubxCkA;
      ubxState = LEN2;
      break;
    case LEN2:
      ubxLen |= ((uint16_t)b << 8);
      ubxCkA += b;
      ubxCkB += ubxCkA;
      ubxCount = 0;
      if (ubxLen == 0) {
        ubxState = CK_A;
      } else if (ubxLen > sizeof(ubxPayload)) {
        ubxState = SYNC1;
      } else {
        ubxState = PAYLOAD;
      }
      break;
    case PAYLOAD:
      ubxPayload[ubxCount++] = b;
      ubxCkA += b;
      ubxCkB += ubxCkA;
      if (ubxCount >= ubxLen) {
        ubxState = CK_A;
      }
      break;
    case CK_A:
      if (b == ubxCkA) {
        ubxState = CK_B;
      } else {
        ubxState = SYNC1;
      }
      break;
    case CK_B:
      if (b == ubxCkB) {
        processUbxMessage();
      }
      ubxState = SYNC1;
      break;
  }
}

static void sendUbxConfig() {
  // Enable UBX-NAV-PVT at 5 Hz on UART1
  // UBX-CFG-VALSET: set CFG-MSGOUT-UBX_NAV_PVT_UART1 = 5
  uint8_t cfg[] = {
    0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00,
    0x00, 0x01, 0x00, 0x00,
    0x07, 0x00, 0x91, 0x20, 0x05,
    0x00, 0x00
  };
  // Calculate checksum
  uint8_t ckA = 0, ckB = 0;
  for (int i = 2; i < 15; i++) {
    ckA += cfg[i];
    ckB += ckA;
  }
  cfg[15] = ckA;
  cfg[16] = ckB;

  GpsSerial.write(cfg, 17);
  Serial.println("GPS: UBX-NAV-PVT config sent");
}

void gps_init() {
  GpsSerial.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  Serial.printf("GPS: UART1 %d baud, RX=%d TX=%d\n", GPS_BAUD, PIN_GPS_RX, PIN_GPS_TX);

  delay(100);
  sendUbxConfig();

  // Start UDP listener for RTCM
  g_rtcmUdp.begin(RTCM_UDP_PORT);
  Serial.printf("GPS: RTCM UDP listener on port %d\n", RTCM_UDP_PORT);

  g_gpsData.valid = false;
}

void gps_update() {
  while (GpsSerial.available()) {
    feedUbxByte(GpsSerial.read());
  }
}

void rtcm_relay() {
  int packetSize = g_rtcmUdp.parsePacket();
  if (packetSize > 0) {
    uint8_t buf[512];
    int len = g_rtcmUdp.read(buf, sizeof(buf));
    if (len > 0) {
      GpsSerial.write(buf, len);
      Serial.printf("RTCM: relayed %d bytes to F9P\n", len);
    }
  }
}
