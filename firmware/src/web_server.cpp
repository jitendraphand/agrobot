#include "web_server.h"

#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>

#include "protocol.h"
#include "web_ui.h"

// The robot is its own access point so it works in a field with no network.
// Change the password before taking it anywhere public.
static const char* AP_PASSWORD = "agrobot123";

namespace net {

static WebServer server(80);
static agro::Robot* g_robot = nullptr;
static char g_reply[1024];

static void handleRoot() { server.send_P(200, "text/html", WEB_UI); }

static void handleCmd() {
  if (!server.hasArg("c")) {
    server.send(400, "text/plain", "ERR missing c=");
    return;
  }
  const String line = server.arg("c");
  agro::handleCommand(*g_robot, line.c_str(), g_reply, sizeof(g_reply));
  server.send(200, "text/plain", g_reply);
}

static void handleStatus() {
  g_robot->statusJson(g_reply, sizeof(g_reply));
  server.send(200, "application/json", g_reply);
}

void begin(agro::Robot& robot) {
  g_robot = &robot;
  char ssid[24];
  uint8_t mac[6];
  WiFi.mode(WIFI_AP);
  WiFi.softAPmacAddress(mac);
  snprintf(ssid, sizeof(ssid), "AgroBot-%02X%02X", mac[4], mac[5]);
  WiFi.softAP(ssid, AP_PASSWORD);
  server.on("/", handleRoot);
  server.on("/cmd", handleCmd);
  server.on("/status", handleStatus);
  server.onNotFound([]() { server.send(404, "text/plain", "not found"); });
  server.begin();
  Serial.printf("WiFi AP '%s'  password '%s'  ->  http://%s/\n", ssid, AP_PASSWORD, WiFi.softAPIP().toString().c_str());
}

void poll() { server.handleClient(); }

}  // namespace net
