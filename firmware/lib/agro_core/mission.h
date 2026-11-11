// A mission is a short program of primitive steps executed by Robot.
#pragma once

#include <stdint.h>

#include "kinematics.h"

namespace agro {

enum class Op : uint8_t {
  End = 0,
  Drive,       // a = metres, b = speed
  Turn,        // a = radians, b = rate
  ArmJoints,   // a..d = yaw, shoulder, elbow, wrist (rad), e = speed scale
  ArmMove,     // a..c = x,y,z (m), d = pitch (rad), tool, e = speed scale
  ArmLinear,   // a..c = x,y,z (m), d = pitch (rad), tool, e = speed (m/s)
  Grip,        // a = finger angle (rad)
  Wait,        // a = seconds
  Pump,        // a = 1 on / 0 off
  SampleSoil,  // store the current soil reading in the log
  SkipIfWet,   // a = number of following steps to skip when the last sample is above the dry threshold
};

struct Step {
  Op op;
  Tool tool;
  float a, b, c, d, e;
};

class Program {
 public:
  static constexpr int CAPACITY = 128;
  void clear() { n_ = 0; overflow_ = false; }
  int size() const { return n_; }
  bool ok() const { return !overflow_ && !error_; }
  const char* error() const { return errorText_; }
  const Step& at(int i) const { return steps_[i]; }
  void fail(const char* why) { error_ = true; errorText_ = why; }
  void resetError() { error_ = false; errorText_ = ""; }

  void add(Op op, float a = 0, float b = 0, float c = 0, float d = 0, float e = 0, Tool tool = Tool::Gripper);
  void drive(float m, float speed) { add(Op::Drive, m, speed); }
  void turn(float rad, float rate) { add(Op::Turn, rad, rate); }
  void armJoints(const ArmJoints& q, float scale = 1.0f) { add(Op::ArmJoints, q.yaw, q.shoulder, q.elbow, q.wrist, scale); }
  void armMove(const Vec3& p, float pitch, Tool t, float scale = 1.0f) { add(Op::ArmMove, p.x, p.y, p.z, pitch, scale, t); }
  void armLinear(const Vec3& p, float pitch, Tool t, float speed) { add(Op::ArmLinear, p.x, p.y, p.z, pitch, speed, t); }
  void grip(float a) { add(Op::Grip, a); }
  void wait(float s) { add(Op::Wait, s); }
  void pump(bool on) { add(Op::Pump, on ? 1.0f : 0.0f); }

 private:
  Step steps_[CAPACITY];
  int n_ = 0;
  bool overflow_ = false;
  bool error_ = false;
  const char* errorText_ = "";
};

ArmJoints poseDeg(const float deg[4]);

// --- mission builders (append to the program) ---
// Probe the soil beside a plant standing `rowY` metres to the side of the
// robot (positive = left), abeam of the arm; spray it if the reading is dry.
void buildPlantCare(Program& prog, float rowY, float sprayHeight);
// Drive along a row and care for `count` plants.
void buildRowMission(Program& prog, int count, float firstDist, float spacing, float rowY, float sprayHeight);
// Pick a round fruit at p (robot frame) and drop it in the basket.
void buildPick(Program& prog, const Vec3& p, float diameter);

}  // namespace agro
