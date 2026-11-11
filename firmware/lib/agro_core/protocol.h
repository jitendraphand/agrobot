// Line-oriented text protocol shared by the USB serial port, the HTTP API
// (/cmd?c=...) and the simulator.  Human units: millimetres and degrees.
//
//   PING                                   -> OK PONG
//   STATUS                                 -> one line of JSON
//   VEL <mm/s> <deg/s>                     teleop (repeat at >= 3 Hz)
//   MOVE <mm> [mm/s]      TURN <deg> [deg/s]
//   ARM J <yaw> <shoulder> <elbow> <wrist> joint move
//   ARM P <x> <y> <z> <pitch> [G|P|N]      move tool centre point (robot frame)
//   ARM L <x> <y> <z> <pitch> [G|P|N] [mm/s]   straight-line move
//   ARM HOME | ARM STOW
//   GRIP <deg> | GRIP OPEN | GRIP CLOSE
//   PUMP <seconds> | PUMP ON | PUMP OFF
//   PROBE <rowY mm> [spray height mm]      probe soil beside the robot, spray if dry
//   ROW <count> <first mm> <spacing mm> <rowY mm> [spray height mm]
//   PICK <x> <y> <z> <diameter mm>         pick a fruit, drop it in the basket
//   STOP | ESTOP | RESET
#pragma once

#include <stddef.h>

#include "robot.h"

namespace agro {

// Executes one command line.  Writes a single-line reply ("OK ..." or
// "ERR ...") into `reply`.  Returns true when the command was accepted.
bool handleCommand(Robot& robot, const char* line, char* reply, size_t replySize);

}  // namespace agro
