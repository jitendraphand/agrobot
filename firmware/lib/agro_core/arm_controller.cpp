#include "arm_controller.h"

#include "agro_config.h"
#include "robot_geometry.h"

namespace agro {

static ArmJoints clampJoints(ArmJoints q) {
  q.yaw = clampf(q.yaw, geom::YAW_MIN, geom::YAW_MAX);
  q.shoulder = clampf(q.shoulder, geom::SHOULDER_MIN, geom::SHOULDER_MAX);
  q.elbow = clampf(q.elbow, geom::ELBOW_MIN, geom::ELBOW_MAX);
  q.wrist = clampf(q.wrist, geom::WRIST_MIN, geom::WRIST_MAX);
  return q;
}

void ArmController::reset(const ArmJoints& q, float grip) {
  q_ = clampJoints(q);
  grip_ = gripTarget_ = clampf(grip, geom::GRIP_MIN, geom::GRIP_MAX);
  mode_ = Mode::Idle;
  gripBusy_ = false;
  fault_ = false;
}

bool ArmController::moveJoints(const ArmJoints& target, float speedScale) {
  speedScale = clampf(speedScale, 0.05f, 1.0f);
  q0_ = q_;
  q1_ = clampJoints(target);
  float T = cfg::MIN_MOVE_TIME;
  const float d[4] = {q1_.yaw - q0_.yaw, q1_.shoulder - q0_.shoulder, q1_.elbow - q0_.elbow, q1_.wrist - q0_.wrist};
  for (int i = 0; i < 4; ++i) {
    // quintic profile peaks at 1.875x the mean speed
    const float ti = 1.875f * fabsf(d[i]) / (cfg::JOINT_VEL[i] * speedScale);
    if (ti > T) T = ti;
  }
  T_ = T;
  t_ = 0;
  mode_ = Mode::Joint;
  return true;
}

IkResult ArmController::moveTo(const Vec3& p, float pitch, Tool tool, float speedScale) {
  ArmJoints q;
  const IkResult r = armIK(p, pitch, tool, q, nullptr);
  if (r != IkResult::Ok) return r;
  moveJoints(q, speedScale);
  return IkResult::Ok;
}

IkResult ArmController::moveLinear(const Vec3& p, float pitch, Tool tool, float speed) {
  ArmJoints q;
  const IkResult r = armIK(p, pitch, tool, q, &q_);      // make sure the end point is valid
  if (r != IkResult::Ok) return r;
  const ArmPose now = armFK(q_, tool);
  p0_ = now.p;
  pitch0_ = now.pitch;
  p1_ = p;
  pitch1_ = pitch;
  tool_ = tool;
  const float dx = p.x - p0_.x, dy = p.y - p0_.y, dz = p.z - p0_.z;
  const float dist = sqrtf(dx * dx + dy * dy + dz * dz);
  speed = clampf(speed <= 0 ? cfg::LINEAR_VEL : speed, 0.005f, 0.3f);
  float T = 1.875f * dist / speed;
  const float tp = 1.875f * fabsf(wrapPi(pitch1_ - pitch0_)) / cfg::JOINT_VEL[3];
  if (tp > T) T = tp;
  if (T < cfg::MIN_MOVE_TIME) T = cfg::MIN_MOVE_TIME;
  T_ = T;
  t_ = 0;
  mode_ = Mode::Linear;
  return IkResult::Ok;
}

void ArmController::setGrip(float angle) {
  gripTarget_ = clampf(angle, geom::GRIP_MIN, geom::GRIP_MAX);
  gripBusy_ = true;
}

void ArmController::hold() {
  mode_ = Mode::Idle;
  gripTarget_ = grip_;
  gripBusy_ = false;
}

void ArmController::update(float dt) {
  if (mode_ == Mode::Joint) {
    t_ += dt;
    const float s = smooth5(t_ / T_);
    q_.yaw = q0_.yaw + s * (q1_.yaw - q0_.yaw);
    q_.shoulder = q0_.shoulder + s * (q1_.shoulder - q0_.shoulder);
    q_.elbow = q0_.elbow + s * (q1_.elbow - q0_.elbow);
    q_.wrist = q0_.wrist + s * (q1_.wrist - q0_.wrist);
    if (t_ >= T_) {
      q_ = q1_;
      mode_ = Mode::Idle;
    }
  } else if (mode_ == Mode::Linear) {
    t_ += dt;
    const float s = smooth5(t_ / T_);
    Vec3 p = {p0_.x + s * (p1_.x - p0_.x), p0_.y + s * (p1_.y - p0_.y), p0_.z + s * (p1_.z - p0_.z)};
    const float pitch = pitch0_ + s * wrapPi(pitch1_ - pitch0_);
    ArmJoints q;
    if (armIK(p, pitch, tool_, q, &q_) == IkResult::Ok) {
      q_ = q;
    } else {
      fault_ = true;               // left the workspace mid-line: stop here
      mode_ = Mode::Idle;
    }
    if (t_ >= T_) mode_ = Mode::Idle;
  }
  if (gripBusy_) {
    const float step = cfg::JOINT_VEL[4] * dt;
    const float e = gripTarget_ - grip_;
    if (fabsf(e) <= step) {
      grip_ = gripTarget_;
      gripBusy_ = false;
    } else {
      grip_ += e > 0 ? step : -step;
    }
  }
}

}  // namespace agro
