#include "websocket.h"
#include "config.h"
#include "motors.h"
#include "nav.h"
#include "sound.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

static AsyncWebServer server(WS_PORT);
static AsyncWebSocket ws("/ws");

static inline void trimInPlace(char* s) {
  int n = (int)strlen(s);
  while (n > 0 && (s[n - 1] == ' ' || s[n - 1] == '\r' || s[n - 1] == '\n' || s[n - 1] == '\t')) s[--n] = 0;
  int i = 0;
  while (s[i] == ' ' || s[i] == '\r' || s[i] == '\n' || s[i] == '\t') i++;
  if (i > 0) memmove(s, s + i, strlen(s + i) + 1);
}

static inline bool streq(const char* a, const char* b) { return strcmp(a, b) == 0; }
static inline bool startsWith(const char* s, const char* pref) { return strncmp(s, pref, strlen(pref)) == 0; }

static bool parseMove(const char* msg, int16_t& outL, int16_t& outR) {
  if (!startsWith(msg, "M,")) return false;

  const char* p = msg + 2;
  char* end1 = nullptr;
  long L = strtol(p, &end1, 10);
  if (!end1 || *end1 != ',') return false;

  const char* p2 = end1 + 1;
  char* end2 = nullptr;
  long R = strtol(p2, &end2, 10);
  if (!end2) return false;

  L = L / INPUT_DIV;
  R = R / INPUT_DIV;

  outL = constrain((int)L, -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);
  outR = constrain((int)R, -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);
  return true;
}

static void onWsEvent(AsyncWebSocket *serverPtr, AsyncWebSocketClient *client,
                      AwsEventType type, void *arg, uint8_t *data, size_t len) {

  if (type == WS_EVT_CONNECT) {
    Serial.printf("WS: Client %u connected\n", client->id());
    client->text("STATE,CONNECTED");
    return;
  }

  if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WS: Client %u disconnected\n", client->id());
    motors_request_smooth_stop("websocket disconnect");
    nav_stop();
    return;
  }

  if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;

    if (!info->final || info->index != 0 || info->len != len) return;
    if (info->opcode != WS_TEXT) return;
    if (len >= MAX_WS_MSG) { client->text("ERR,TOO_LONG"); return; }

    char msg[MAX_WS_MSG];
    memcpy(msg, data, len);
    msg[len] = 0;
    trimInPlace(msg);

    // PING/STOP
    if (streq(msg, "PING")) { client->text("PONG"); return; }
    if (streq(msg, "STOP")) {
      motors_request_smooth_stop("manual stop");
      client->text("OK STOP");
      return;
    }

    // Movement
    int16_t L = 0, R = 0;
    if (parseMove(msg, L, R)) {
      g_targetLeft = L;
      g_targetRight = R;
      g_lastCmdMs = millis();
      client->text("OK M");
      return;
    }

    // Relays
    if (streq(msg, "ATTACHMENT_ON"))  { setAttachment(true);  client->text("OK ATTACHMENT_ON");  return; }
    if (streq(msg, "ATTACHMENT_OFF")) { setAttachment(false); client->text("OK ATTACHMENT_OFF"); return; }
    if (streq(msg, "MOUNT_ON"))       { setMount(true);       client->text("OK MOUNT_ON");       return; }
    if (streq(msg, "MOUNT_OFF"))      { setMount(false);      client->text("OK MOUNT_OFF");      return; }

    // Sound
    if (startsWith(msg, "SOUND:") || startsWith(msg, "SOUND,")) {
      const char* p = msg + 6;
      if (*p == ':' || *p == ',') p++;
      int n = atoi(p);
      if (n >= 1 && n <= 4) {
        enqueueSound((SoundId)n);
        client->text("OK SOUND");
      } else {
        client->text("ERR SOUND_RANGE");
      }
      return;
    }

    // Route commands
    if (startsWith(msg, "ROUTE_BEGIN,")) {
      int count = atoi(msg + 12);
      nav_clear_route();
      Serial.printf("WS: ROUTE_BEGIN count=%d\n", count);
      client->text("OK ROUTE_BEGIN");
      return;
    }

    if (startsWith(msg, "ROUTE_WP,")) {
      // Format: ROUTE_WP,<index>,<lat>,<lon>
      const char* p = msg + 9;
      char* end1 = nullptr;
      int idx = strtol(p, &end1, 10);
      if (!end1 || *end1 != ',') { client->text("ERR ROUTE_WP_FORMAT"); return; }

      const char* p2 = end1 + 1;
      char* end2 = nullptr;
      double lat = strtod(p2, &end2);
      if (!end2 || *end2 != ',') { client->text("ERR ROUTE_WP_FORMAT"); return; }

      const char* p3 = end2 + 1;
      char* end3 = nullptr;
      double lon = strtod(p3, &end3);
      if (!end3) { client->text("ERR ROUTE_WP_FORMAT"); return; }

      if (nav_add_waypoint(lat, lon)) {
        Serial.printf("WS: ROUTE_WP %d: %.8f, %.8f\n", idx, lat, lon);
        client->text("OK ROUTE_WP");
      } else {
        client->text("ERR ROUTE_WP_FULL");
      }
      return;
    }

    if (streq(msg, "ROUTE_END")) {
      Serial.printf("WS: ROUTE_END total=%d\n", g_navWpTotal);
      char resp[32];
      snprintf(resp, sizeof(resp), "OK ROUTE,%d", g_navWpTotal);
      client->text(resp);
      return;
    }

    // Navigation commands
    if (streq(msg, "NAV_START")) {
      nav_start();
      client->text("OK NAV_START");
      return;
    }

    if (streq(msg, "NAV_PAUSE")) {
      nav_pause();
      client->text("OK NAV_PAUSE");
      return;
    }

    if (streq(msg, "NAV_RESUME")) {
      nav_resume();
      client->text("OK NAV_RESUME");
      return;
    }

    if (streq(msg, "NAV_STOP")) {
      nav_stop();
      client->text("OK NAV_STOP");
      return;
    }

    client->text("ERR,UNKNOWN");
  }
}

void websocket_init() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi AP: ");
  Serial.println(WiFi.softAPIP());

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/ping", HTTP_GET, [](AsyncWebServerRequest *req){
    req->send(200, "text/plain", "OK");
  });

  server.begin();
  Serial.printf("WebSocket server on port %d\n", WS_PORT);
}

void websocket_cleanup() {
  ws.cleanupClients();
}

void websocket_send(const char* msg) {
  if (ws.count() > 0) {
    ws.textAll(msg);
  }
}
