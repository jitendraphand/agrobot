#include "protocol.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "agro_config.h"
#include "robot_geometry.h"

namespace agro {

namespace {

constexpr int MAX_TOK = 10;

struct Tokens {
  char buf[128];
  char* v[MAX_TOK];
  int n = 0;
};

void tokenize(const char* line, Tokens& t) {
  size_t len = strlen(line);
  if (len >= sizeof(t.buf)) len = sizeof(t.buf) - 1;
  for (size_t i = 0; i < len; ++i) t.buf[i] = (char)toupper((unsigned char)line[i]);
  t.buf[len] = 0;
  t.n = 0;
  char* p = t.buf;
  while (*p && t.n < MAX_TOK) {
    while (*p == ' ' || *p == '\t' || *p == ',' || *p == '\r' || *p == '\n') ++p;
    if (!*p) break;
    t.v[t.n++] = p;
    while (*p && *p != ' ' && *p != '\t' && *p != ',' && *p != '\r' && *p != '\n') ++p;
    if (*p) *p++ = 0;
  }
}

bool isNumber(const char* s) {
  char* end = nullptr;
  strtod(s, &end);
  return end != s && *end == 0;
}

float num(const Tokens& t, int i, float dflt) {
  return (i < t.n && isNumber(t.v[i])) ? (float)atof(t.v[i]) : dflt;
}

bool haveNums(const Tokens& t, int first, int count) {
  if (t.n < first + count) return false;
  for (int i = first; i < first + count; ++i)
    if (!isNumber(t.v[i])) return false;
  return true;
}

Tool toolFrom(const Tokens& t, int i) {
  if (i < t.n) {
    if (t.v[i][0] == 'P') return Tool::Probe;
    if (t.v[i][0] == 'N') return Tool::Nozzle;
  }
  return Tool::Gripper;
}

bool finish(Robot& r, bool ok, char* reply, size_t n) {
  if (ok) snprintf(reply, n, "OK");
  else snprintf(reply, n, "ERR %s", r.lastError());
  return ok;
}

bool usage(char* reply, size_t n, const char* text) {
  snprintf(reply, n, "ERR usage: %s", text);
  return false;
}

}  // namespace

bool handleCommand(Robot& robot, const char* line, char* reply, size_t n) {
  Tokens t;
  tokenize(line, t);
  if (t.n == 0) {
    snprintf(reply, n, "ERR empty");
    return false;
  }
  const char* c = t.v[0];
  const float MM = 0.001f;

  if (!strcmp(c, "PING")) {
    snprintf(reply, n, "OK PONG");
    return true;
  }
  if (!strcmp(c, "STATUS")) {
    robot.statusJson(reply, n);
    return true;
  }
  if (!strcmp(c, "STOP")) {
    robot.cmdStop();
    snprintf(reply, n, "OK");
    return true;
  }
  if (!strcmp(c, "ESTOP")) {
    robot.cmdEStop();
    snprintf(reply, n, "OK");
    return true;
  }
  if (!strcmp(c, "RESET")) return finish(robot, robot.cmdResetEStop(), reply, n);

  if (!strcmp(c, "VEL")) {
    if (!haveNums(t, 1, 2)) return usage(reply, n, "VEL <mm/s> <deg/s>");
    return finish(robot, robot.cmdVelocity(num(t, 1, 0) * MM, num(t, 2, 0) * DEG), reply, n);
  }
  if (!strcmp(c, "MOVE")) {
    if (!haveNums(t, 1, 1)) return usage(reply, n, "MOVE <mm> [mm/s]");
    return finish(robot, robot.cmdMove(num(t, 1, 0) * MM, num(t, 2, 200) * MM), reply, n);
  }
  if (!strcmp(c, "TURN")) {
    if (!haveNums(t, 1, 1)) return usage(reply, n, "TURN <deg> [deg/s]");
    return finish(robot, robot.cmdTurn(num(t, 1, 0) * DEG, num(t, 2, 45) * DEG), reply, n);
  }
  if (!strcmp(c, "ARM")) {
    if (t.n >= 2 && !strcmp(t.v[1], "HOME")) return finish(robot, robot.cmdHome(), reply, n);
    if (t.n >= 2 && !strcmp(t.v[1], "STOW")) return finish(robot, robot.cmdStow(), reply, n);
    if (t.n >= 2 && !strcmp(t.v[1], "J")) {
      if (!haveNums(t, 2, 4)) return usage(reply, n, "ARM J <yaw> <shoulder> <elbow> <wrist>");
      const ArmJoints q = {num(t, 2, 0) * DEG, num(t, 3, 0) * DEG, num(t, 4, 0) * DEG, num(t, 5, 0) * DEG};
      return finish(robot, robot.cmdArmJoints(q), reply, n);
    }
    if (t.n >= 2 && (!strcmp(t.v[1], "P") || !strcmp(t.v[1], "L"))) {
      if (!haveNums(t, 2, 4)) return usage(reply, n, "ARM P|L <x> <y> <z> <pitch> [G|P|N] [mm/s]");
      const Vec3 p = {num(t, 2, 0) * MM, num(t, 3, 0) * MM, num(t, 4, 0) * MM};
      const float pitch = num(t, 5, 0) * DEG;
      const Tool tool = toolFrom(t, 6);
      if (t.v[1][0] == 'P') return finish(robot, robot.cmdArmPose(p, pitch, tool), reply, n);
      return finish(robot, robot.cmdArmLinear(p, pitch, tool, num(t, 7, cfg::LINEAR_VEL * 1000.0f) * MM), reply, n);
    }
    return usage(reply, n, "ARM J|P|L|HOME|STOW ...");
  }
  if (!strcmp(c, "GRIP")) {
    if (t.n < 2) return usage(reply, n, "GRIP <deg>|OPEN|CLOSE");
    float a;
    if (!strcmp(t.v[1], "OPEN")) a = geom::GRIP_MAX;
    else if (!strcmp(t.v[1], "CLOSE")) a = geom::GRIP_MIN;
    else if (isNumber(t.v[1])) a = num(t, 1, 0) * DEG;
    else return usage(reply, n, "GRIP <deg>|OPEN|CLOSE");
    return finish(robot, robot.cmdGrip(a), reply, n);
  }
  if (!strcmp(c, "PUMP")) {
    if (t.n < 2) return usage(reply, n, "PUMP <seconds>|ON|OFF");
    float s;
    if (!strcmp(t.v[1], "ON")) s = -1.0f;
    else if (!strcmp(t.v[1], "OFF")) s = 0.0f;
    else if (isNumber(t.v[1])) s = clampf(num(t, 1, 0), 0.0f, 30.0f);
    else return usage(reply, n, "PUMP <seconds>|ON|OFF");
    return finish(robot, robot.cmdPump(s), reply, n);
  }
  if (!strcmp(c, "PROBE")) {
    if (!haveNums(t, 1, 1)) return usage(reply, n, "PROBE <rowY mm> [spray height mm]");
    return finish(robot, robot.cmdProbe(num(t, 1, 0) * MM, num(t, 2, cfg::SPRAY_HEIGHT * 1000.0f) * MM), reply, n);
  }
  if (!strcmp(c, "ROW")) {
    if (!haveNums(t, 1, 4)) return usage(reply, n, "ROW <count> <first mm> <spacing mm> <rowY mm> [spray height mm]");
    return finish(robot, robot.cmdRowMission((int)num(t, 1, 1), num(t, 2, 0) * MM, num(t, 3, 0) * MM, num(t, 4, 0) * MM,
                                             num(t, 5, cfg::SPRAY_HEIGHT * 1000.0f) * MM), reply, n);
  }
  if (!strcmp(c, "PICK")) {
    if (!haveNums(t, 1, 4)) return usage(reply, n, "PICK <x> <y> <z> <diameter mm>");
    const Vec3 p = {num(t, 1, 0) * MM, num(t, 2, 0) * MM, num(t, 3, 0) * MM};
    return finish(robot, robot.cmdPick(p, num(t, 4, 40) * MM), reply, n);
  }
  snprintf(reply, n, "ERR unknown command '%s'", c);
  return false;
}

}  // namespace agro
