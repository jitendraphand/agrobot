// Host-side unit tests for the hardware-independent control core.
//   make -C firmware/test_host test
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "agro_config.h"
#include "protocol.h"
#include "robot.h"
#include "robot_geometry.h"

using namespace agro;

static int g_fail = 0, g_checks = 0;
#define CHECK(cond)                                                              \
  do {                                                                           \
    ++g_checks;                                                                  \
    if (!(cond)) {                                                               \
      ++g_fail;                                                                  \
      printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);                   \
    }                                                                            \
  } while (0)
#define CHECK_NEAR(a, b, tol)                                                    \
  do {                                                                           \
    ++g_checks;                                                                  \
    if (fabs((double)(a) - (double)(b)) > (tol)) {                               \
      ++g_fail;                                                                  \
      printf("  FAIL %s:%d  %s = %g, expected %g\n", __FILE__, __LINE__, #a, (double)(a), (double)(b)); \
    }                                                                            \
  } while (0)

static float frand(float lo, float hi) { return lo + (hi - lo) * (float)rand() / (float)RAND_MAX; }

// ---------------------------------------------------------------- kinematics
static void test_fk_known() {
  // arm stretched straight forward: everything adds up along X
  ArmJoints q = {0, 0, 0, 0};
  ArmPose p = armFK(q, Tool::Gripper);
  CHECK_NEAR(p.p.x, geom::YAW_X + geom::SHOULDER_OFF + geom::L1 + geom::L2 + geom::TCP_GRIPPER[0], 1e-5);
  CHECK_NEAR(p.p.y, 0, 1e-6);
  CHECK_NEAR(p.p.z, geom::SHOULDER_Z, 1e-6);
  // yaw 90 deg swings it to +Y
  q.yaw = 90 * DEG;
  p = armFK(q, Tool::Gripper);
  CHECK_NEAR(p.p.x, geom::YAW_X, 1e-5);
  CHECK_NEAR(p.p.y, geom::SHOULDER_OFF + geom::L1 + geom::L2 + geom::TCP_GRIPPER[0], 1e-5);
  // straight up
  q = {0, 90 * DEG, 0, 0};
  p = armFK(q, Tool::Gripper);
  CHECK_NEAR(p.p.z, geom::SHOULDER_Z + geom::L1 + geom::L2 + geom::TCP_GRIPPER[0], 1e-5);
  CHECK_NEAR(p.pitch, 90 * DEG, 1e-5);
}

static void test_ik_roundtrip() {
  srand(7);
  int solved = 0;
  for (int i = 0; i < 20000; ++i) {
    ArmJoints q = {frand(geom::YAW_MIN, geom::YAW_MAX), frand(geom::SHOULDER_MIN, geom::SHOULDER_MAX),
                   frand(geom::ELBOW_MIN, geom::ELBOW_MAX), frand(geom::WRIST_MIN, geom::WRIST_MAX)};
    if (fabsf(q.elbow) < 0.05f) continue;                 // skip the singular stretched arm
    const Tool tool = (Tool)(i % 3);
    const ArmPose p = armFK(q, tool);
    // directly above / below the yaw axis the yaw angle is undefined: skip that singular column
    const float* off = toolOffset(tool);
    const float ddx = p.p.x - geom::YAW_X;
    if (ddx * ddx + p.p.y * p.p.y - off[1] * off[1] < 0.03f * 0.03f) continue;
    ArmJoints r;
    const IkResult res = armIK(p.p, p.pitch, tool, r, &q);
    CHECK(res == IkResult::Ok);
    if (res != IkResult::Ok) continue;
    ++solved;
    const ArmPose p2 = armFK(r, tool);
    CHECK_NEAR(p2.p.x, p.p.x, 2e-4);
    CHECK_NEAR(p2.p.y, p.p.y, 2e-4);
    CHECK_NEAR(p2.p.z, p.p.z, 2e-4);
    CHECK(jointsWithinLimits(r, 1e-4f));
    // seeded with the truth it must return the same branch
    CHECK_NEAR(r.shoulder, q.shoulder, 2e-3);
    CHECK_NEAR(r.elbow, q.elbow, 2e-3);
  }
  CHECK(solved > 15000);
}

static void test_ik_tasks() {
  ArmJoints q;
  // soil probe pushed 40 mm into the ground beside the left wheels
  CHECK(armIK(Vec3{geom::YAW_X, 0.295f, -0.040f}, 0, Tool::Probe, q) == IkResult::Ok);
  CHECK(q.yaw > 60 * DEG && q.yaw < 120 * DEG);
  CHECK(q.elbow < 0);                                     // elbow-up posture
  ArmPose p = armFK(q, Tool::Probe);
  CHECK_NEAR(p.p.z, -0.040f, 1e-4);
  CHECK_NEAR(p.pitch, 0, 1e-4);
  // mirror on the right-hand side
  ArmJoints qr;
  CHECK(armIK(Vec3{geom::YAW_X, -0.295f, -0.040f}, 0, Tool::Probe, qr) == IkResult::Ok);
  CHECK(qr.yaw < -60 * DEG);
  // basket is behind the arm: only reachable "over the shoulder"
  const Vec3 drop = {geom::BASKET_X, geom::BASKET_Y, geom::BASKET_RIM_Z + 0.075f};
  CHECK(armIK(drop, -72 * DEG, Tool::Gripper, q) == IkResult::Ok);
  CHECK(fabsf(q.yaw) < 5 * DEG);
  CHECK(q.elbow > 0);
  p = armFK(q, Tool::Gripper);
  CHECK_NEAR(p.p.x, drop.x, 1e-4);
  CHECK_NEAR(p.p.z, drop.z, 1e-4);
  CHECK_NEAR(p.pitch, -72 * DEG, 1e-3);
  // far out of reach
  CHECK(armIK(Vec3{1.5f, 0, 0.2f}, 0, Tool::Gripper, q) == IkResult::Unreachable);
}

static void test_grip() {
  CHECK_NEAR(gripAngleForDiameter(0.003f), 0.0f, 1e-3);   // pads meet 3 mm apart when closed
  const float a40 = gripAngleForDiameter(0.040f);
  CHECK(a40 > 15 * DEG && a40 < 30 * DEG);
  CHECK(gripAngleForDiameter(0.060f) > a40);
  CHECK(gripAngleForDiameter(0.500f) <= geom::GRIP_MAX + 1e-6f);
}

// --------------------------------------------------------------------- plant
// Minimal stand-in for the hardware: first-order DC motors, ideal encoders.
struct Plant {
  double wl = 0, wr = 0, angL = 0, angR = 0;
  double x = 0, y = 0, th = 0;
  int soilRaw = cfg::SOIL_RAW_DRY;
  float range = 9.9f;
  float batt = 12.0f;
  bool estop = false;
  double pumpSeconds = 0;
  int pumpStarts = 0;
  bool pumpWas = false;

  void step(Robot& r, Outputs& out, float dt) {
    Inputs in;
    in.dt = dt;
    in.encL = (int32_t)lround(angL / (2 * M_PI) * geom::ENC_TICKS_PER_REV);
    in.encR = (int32_t)lround(angR / (2 * M_PI) * geom::ENC_TICKS_PER_REV);
    in.imuValid = true;
    const double w = (wr - wl) * geom::WHEEL_RADIUS / (geom::TRACK * cfg::TRACK_SLIP_FACTOR);
    in.gyroZ = (float)w;
    in.battV = batt;
    in.soilRaw = soilRaw;
    in.rangeM = range;
    in.estop = estop;
    r.step(in, out);
    // motor: speed follows duty with a 60 ms time constant, with stiction
    const double tau = 0.06;
    auto motor = [&](double& wv, float duty) {
      double tgt = fabs(duty) < 0.03 ? 0.0 : duty * geom::MOTOR_NOLOAD_RAD_S;
      if (!out.motorsEnabled) tgt = 0;
      wv += (tgt - wv) * dt / tau;
    };
    motor(wl, out.dutyL);
    motor(wr, out.dutyR);
    angL += wl * dt;
    angR += wr * dt;
    const double v = 0.5 * (wl + wr) * geom::WHEEL_RADIUS;
    x += v * cos(th) * dt;
    y += v * sin(th) * dt;
    th += w * dt;
    if (out.pump) pumpSeconds += dt;
    if (out.pump && !pumpWas) ++pumpStarts;
    pumpWas = out.pump;
  }
};

static void test_drive_moves() {
  Robot r;
  Plant pl;
  Outputs out;
  char rep[512];
  CHECK(handleCommand(r, "MOVE 1000 250", rep, sizeof rep));
  float t = 0, vmax = 0;
  for (; t < 12 && (r.drive().busy() || t < 0.1f); t += 0.01f) {
    pl.step(r, out, 0.01f);
    if (r.drive().linVel() > vmax) vmax = r.drive().linVel();
  }
  CHECK(!r.drive().busy());
  CHECK_NEAR(pl.x, 1.0, 0.008);
  CHECK_NEAR(r.drive().odom().x, 1.0, 0.008);
  CHECK(vmax < 0.29f && vmax > 0.2f);   // <= 15 % speed overshoot
  CHECK(t < 7.0f);
  CHECK(handleCommand(r, "TURN 90", rep, sizeof rep));
  for (t = 0; t < 10 && (r.drive().busy() || t < 0.1f); t += 0.01f) pl.step(r, out, 0.01f);
  CHECK(!r.drive().busy());
  CHECK_NEAR(pl.th, M_PI / 2, 0.03);
  CHECK(r.state() == State::Idle);
}

static void test_watchdog_and_obstacle() {
  Robot r;
  Plant pl;
  Outputs out;
  char rep[256];
  CHECK(handleCommand(r, "VEL 300 0", rep, sizeof rep));
  for (int i = 0; i < 40; ++i) pl.step(r, out, 0.01f);
  CHECK(pl.wl > 1.0);
  // no further VEL commands: the watchdog must bring it to rest
  for (int i = 0; i < 300; ++i) pl.step(r, out, 0.01f);
  CHECK(fabs(pl.wl) < 0.05 && fabs(pl.wr) < 0.05);
  CHECK(r.state() == State::Idle);
  // obstacle: forward refused, reverse allowed
  pl.range = 0.15f;
  const double x0 = pl.x;
  for (int i = 0; i < 100; ++i) {
    handleCommand(r, "VEL 300 0", rep, sizeof rep);
    pl.step(r, out, 0.01f);
  }
  CHECK_NEAR(pl.x, x0, 0.01);
  for (int i = 0; i < 100; ++i) {
    handleCommand(r, "VEL -200 0", rep, sizeof rep);
    pl.step(r, out, 0.01f);
  }
  CHECK(pl.x < x0 - 0.05);
}

static void test_estop() {
  Robot r;
  Plant pl;
  Outputs out;
  char rep[256];
  handleCommand(r, "VEL 300 0", rep, sizeof rep);
  for (int i = 0; i < 30; ++i) pl.step(r, out, 0.01f);
  pl.estop = true;
  pl.step(r, out, 0.01f);
  CHECK(r.state() == State::EStop);
  CHECK(!out.motorsEnabled && !out.servosEnabled && !out.pump);
  CHECK(out.dutyL == 0 && out.dutyR == 0);
  CHECK(!handleCommand(r, "VEL 100 0", rep, sizeof rep));
  CHECK(!handleCommand(r, "RESET", rep, sizeof rep));       // button still pressed
  pl.estop = false;
  pl.step(r, out, 0.01f);
  CHECK(handleCommand(r, "RESET", rep, sizeof rep));
  CHECK(handleCommand(r, "VEL 100 0", rep, sizeof rep));
}

static void test_arm_motion() {
  Robot r;
  Plant pl;
  Outputs out;
  char rep[256];
  CHECK(handleCommand(r, "ARM P 90 300 100 0 P", rep, sizeof rep));
  float maxStep = 0, prev[4];
  pl.step(r, out, 0.01f);
  memcpy(prev, out.joint, sizeof prev);
  int n = 0;
  while (r.arm().busy() && n < 2000) {
    pl.step(r, out, 0.01f);
    for (int k = 0; k < 4; ++k) {
      maxStep = fmaxf(maxStep, fabsf(out.joint[k] - prev[k]));
      prev[k] = out.joint[k];
    }
    ++n;
  }
  CHECK(n < 2000);
  CHECK(maxStep < 0.045f);                    // < 4.5 rad/s: no step jumps handed to the servos
  ArmPose p = r.arm().pose(Tool::Probe);
  CHECK_NEAR(p.p.y, 0.300f, 1e-3);
  CHECK_NEAR(p.p.z, 0.100f, 1e-3);
  // straight line down: x and y must stay put the whole way
  CHECK(handleCommand(r, "ARM L 90 300 -30 0 P 50", rep, sizeof rep));
  float dev = 0;
  n = 0;
  while (r.arm().busy() && n < 2000) {
    pl.step(r, out, 0.01f);
    p = r.arm().pose(Tool::Probe);
    dev = fmaxf(dev, fmaxf(fabsf(p.p.x - 0.090f), fabsf(p.p.y - 0.300f)));
    ++n;
  }
  CHECK(dev < 5e-4f);
  CHECK_NEAR(p.p.z, -0.030f, 1e-3);
  CHECK(!handleCommand(r, "ARM P 2000 0 0 0", rep, sizeof rep));
  CHECK(strstr(rep, "ERR") == rep);
  CHECK(!handleCommand(r, "ARM J 0 200 0 0", rep, sizeof rep));
}

static void test_row_mission() {
  Robot r;
  Plant pl;
  Outputs out;
  char rep[1024];
  CHECK(handleCommand(r, "ROW 3 610 600 335", rep, sizeof rep));
  CHECK(r.state() == State::Mission);
  CHECK(!handleCommand(r, "VEL 100 0", rep, sizeof rep));    // teleop is locked out
  float t = 0;
  float minProbeZ = 1;
  for (; t < 240 && r.state() == State::Mission; t += 0.01f) {
    // plants at 0.61, 1.21, 1.81 m: the second one is "wet", the others dry
    const ArmPose probe = r.arm().pose(Tool::Probe);
    minProbeZ = fminf(minProbeZ, probe.p.z);
    const int plant = (int)lround((pl.x - 0.61) / 0.6);
    const bool inSoil = probe.p.z < -0.01f;
    pl.soilRaw = !inSoil ? cfg::SOIL_RAW_DRY : (plant == 1 ? 1700 : 2600);
    pl.step(r, out, 0.01f);
  }
  CHECK(r.state() == State::Idle);
  CHECK(t < 200);
  CHECK_NEAR(pl.x, 1.81, 0.02);
  CHECK_NEAR(minProbeZ, -cfg::PROBE_DEPTH, 2e-3);
  CHECK(r.soilLogCount() == 3);
  CHECK(r.soilLog(0) < cfg::SOIL_DRY_PCT);
  CHECK(r.soilLog(1) > cfg::SOIL_DRY_PCT);
  CHECK(r.soilLog(2) < cfg::SOIL_DRY_PCT);
  CHECK(pl.pumpStarts == 2);                                  // dry plants only
  CHECK_NEAR(pl.pumpSeconds, 2 * cfg::SPRAY_S, 0.1);
  r.statusJson(rep, sizeof rep);
  CHECK(strstr(rep, "\"state\":\"IDLE\"") != nullptr);
  CHECK(strstr(rep, "\"log\":[") != nullptr);
  printf("  row mission: %.1f s simulated, soil log %.0f / %.0f / %.0f %%\n", t, r.soilLog(0), r.soilLog(1), r.soilLog(2));
}

static void test_pick_program() {
  Robot r;
  Plant pl;
  Outputs out;
  char rep[256];
  CHECK(handleCommand(r, "PICK 90 305 290 42", rep, sizeof rep));
  float t = 0, gripAtFruit = -1;
  bool visitedBasket = false;
  for (; t < 120 && r.state() == State::Mission; t += 0.01f) {
    pl.step(r, out, 0.01f);
    const ArmPose g = r.arm().pose(Tool::Gripper);
    if (fabsf(g.p.y - 0.305f) < 0.002f && fabsf(g.p.z - 0.290f) < 0.002f && !r.arm().busy()) gripAtFruit = out.joint[4];
    if (fabsf(g.p.x - geom::BASKET_X) < 0.01f && fabsf(g.p.y) < 0.01f) visitedBasket = true;
  }
  CHECK(r.state() == State::Idle);
  CHECK(visitedBasket);
  CHECK(gripAtFruit > 0 && gripAtFruit < gripAngleForDiameter(0.042f));   // squeezed past contact
  CHECK(!handleCommand(r, "PICK 900 900 100 40", rep, sizeof rep));
  CHECK(strstr(rep, "reach") != nullptr);
}

static void test_protocol_errors() {
  Robot r;
  char rep[256];
  CHECK(handleCommand(r, "ping", rep, sizeof rep) && !strcmp(rep, "OK PONG"));
  CHECK(!handleCommand(r, "", rep, sizeof rep));
  CHECK(!handleCommand(r, "FLY 1 2", rep, sizeof rep));
  CHECK(!handleCommand(r, "VEL abc 1", rep, sizeof rep));
  CHECK(!handleCommand(r, "ROW 99 0 600 335", rep, sizeof rep));
  CHECK(handleCommand(r, "pump 1.5", rep, sizeof rep));
  CHECK(r.pumpOn());
  Outputs out;
  Inputs in;
  for (int i = 0; i < 200; ++i) r.step(in, out);
  CHECK(!r.pumpOn());                                          // timed out after 1.5 s
  CHECK(handleCommand(r, "GRIP OPEN", rep, sizeof rep));
  CHECK(handleCommand(r, "STATUS", rep, sizeof rep) && rep[0] == '{');
}

int main() {
  struct { const char* name; void (*fn)(); } tests[] = {
      {"fk_known", test_fk_known},       {"ik_roundtrip", test_ik_roundtrip}, {"ik_tasks", test_ik_tasks},
      {"grip", test_grip},               {"drive_moves", test_drive_moves},   {"watchdog_obstacle", test_watchdog_and_obstacle},
      {"estop", test_estop},             {"arm_motion", test_arm_motion},     {"row_mission", test_row_mission},
      {"pick_program", test_pick_program}, {"protocol_errors", test_protocol_errors},
  };
  for (auto& t : tests) {
    const int before = g_fail;
    printf("[ RUN ] %s\n", t.name);
    t.fn();
    printf("[ %s ] %s\n", g_fail == before ? " OK " : "FAIL", t.name);
  }
  printf("\n%d checks, %d failed\n", g_checks, g_fail);
  return g_fail ? 1 : 0;
}
