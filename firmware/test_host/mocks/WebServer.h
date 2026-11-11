#pragma once
#include <Arduino.h>
#include <functional>
class WebServer {
 public:
  explicit WebServer(int port);
  void on(const char* uri, std::function<void()> handler);
  void onNotFound(std::function<void()> handler);
  void begin();
  void handleClient();
  bool hasArg(const char* name);
  String arg(const char* name);
  void send(int code, const char* type, const char* content);
  void send_P(int code, const char* type, const char* content);
};
