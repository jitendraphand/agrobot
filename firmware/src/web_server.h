#pragma once
#include "robot.h"

namespace net {
void begin(agro::Robot& robot);   // start the WiFi access point + HTTP server
void poll();                      // call from loop()
}  // namespace net
