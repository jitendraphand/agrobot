// Smooth joint-space and straight-line Cartesian motion for the arm.
#pragma once

#include "kinematics.h"

namespace agro {

class ArmController {
 public:
  void reset(const ArmJoints& q, float grip);

  // Joint-space move; duration is set by the slowest joint.  speedScale 0.1..1.
  bool moveJoints(const ArmJoints& target, float speedScale = 1.0f);
  // Solve IK, then joint-space move.
  IkResult moveTo(const Vec3& p, float pitch, Tool tool, float speedScale = 1.0f);
  // Straight line of the tool centre point (IK solved every tick).
  IkResult moveLinear(const Vec3& p, float pitch, Tool tool, float speed);
  void setGrip(float angle);
  void hold();                       // freeze where we are

  void update(float dt);

  bool busy() const { return mode_ != Mode::Idle || gripBusy_; }
  bool fault() const { return fault_; }
  void clearFault() { fault_ = false; }
  const ArmJoints& joints() const { return q_; }
  float grip() const { return grip_; }
  ArmPose pose(Tool t) const { return armFK(q_, t); }

 private:
  enum class Mode : uint8_t { Idle, Joint, Linear };
  Mode mode_ = Mode::Idle;
  ArmJoints q_ = {0, 0, 0, 0};
  ArmJoints q0_ = {0, 0, 0, 0}, q1_ = {0, 0, 0, 0};
  Vec3 p0_ = {0, 0, 0}, p1_ = {0, 0, 0};
  float pitch0_ = 0, pitch1_ = 0;
  Tool tool_ = Tool::Gripper;
  float t_ = 0, T_ = 1;
  float grip_ = 0, gripTarget_ = 0;
  bool gripBusy_ = false;
  bool fault_ = false;
};

}  // namespace agro
