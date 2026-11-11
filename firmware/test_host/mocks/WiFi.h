#pragma once
#include <Arduino.h>
enum wifi_mode_t { WIFI_OFF, WIFI_STA, WIFI_AP, WIFI_AP_STA };
class IPAddress {
 public:
  String toString() const;
};
class WiFiClass {
 public:
  bool mode(wifi_mode_t);
  bool softAP(const char* ssid, const char* pass = nullptr);
  IPAddress softAPIP();
  uint8_t* softAPmacAddress(uint8_t* mac);
};
extern WiFiClass WiFi;
