#include "robot.h"

#include <stdio.h>

#include "agro_config.h"
#include "robot_geometry.h"

namespace agro {

Robot::Robot() { reset(); }

void Robot::reset() {
  drive_.reset();
  arm_.reset(poseDeg(cfg::POSE_HOME), 0.17f);
  prog_.clear();
  prog_.resetError();
  state_ = State::Idle;
  pc_ = 0;
  stepActive_ = false;
  pumpOn_ = false;
  pumpT_ = 0;
  soilN_ = 0;
  time_ = 0;
  error_ = "";
}

const char* Robot::stateName() const {
  switch (state_) {
    case State::Idle:    return "IDLE";
    case State::Teleop:  return "TELEOP";
    case State::Mission: return "MISSION";
    case State::EStop:   return "ESTOP";
    default:             return "FAULT";
  }
}

void Robot::fault(const char* why) {
  error_ = why;
  drive_.stop();
  arm_.hold();
  pumpOn_ = false;
  state_ = State::Fault;
}

bool Robot::ready(const char* what) {
  if (state_ == State::EStop) {
    error_ = "e-stop active (send RESET)";
    return false;
  }
  if (state_ == State::Mission) {
    error_ = "mission running (send STOP first)";
    return false;
  }
  if (state_ == State::Fault) {       // a new command clears a previous fault
    state_ = State::Idle;
    arm_.clearFault();
  }
  (void)what;
  error_ = "";
  return true;
}

// ------------------------------------------------------------------ commands
bool Robot::cmdVelocity(float v, float w) {
  if (!ready("vel")) return false;
  if (in_.battV < cfg::BATT_CRIT_V) {
    error_ = "battery critical";
    return false;
  }
  drive_.setVelocity(v, w);
  state_ = State::Teleop;
  return true;
}

bool Robot::cmdMove(float metres, float speed) {
  if (!ready("move")) return false;
  drive_.driveDistance(metres, speed);
  state_ = State::Teleop;
  return true;
}

bool Robot::cmdTurn(float radians, float rate) {
  if (!ready("turn")) return false;
  drive_.turnAngle(radians, rate);
  state_ = State::Teleop;
  return true;
}

bool Robot::cmdArmJoints(const ArmJoints& q, float speedScale) {
  if (!ready("arm")) return false;
  if (!jointsWithinLimits(q, 1e-3f)) {
    error_ = "joint limit";
    return false;
  }
  arm_.moveJoints(q, speedScale);
  return true;
}

static const char* ikText(IkResult r) {
  return r == IkResult::Unreachable ? "target out of reach" : "target needs a joint beyond its limit";
}

bool Robot::cmdArmPose(const Vec3& p, float pitch, Tool tool) {
  if (!ready("arm")) return false;
  const IkResult r = arm_.moveTo(p, pitch, tool);
  if (r != IkResult::Ok) {
    error_ = ikText(r);
    return false;
  }
  return true;
}

bool Robot::cmdArmLinear(const Vec3& p, float pitch, Tool tool, float speed) {
  if (!ready("arm")) return false;
  const IkResult r = arm_.moveLinear(p, pitch, tool, speed);
  if (r != IkResult::Ok) {
    error_ = ikText(r);
    return false;
  }
  return true;
}

bool Robot::cmdGrip(float angle) {
  if (!ready("grip")) return false;
  arm_.setGrip(angle);
  return true;
}

bool Robot::cmdPump(float seconds) {
  if (state_ == State::EStop) {
    error_ = "e-stop active (send RESET)";
    return false;
  }
  pumpOn_ = seconds != 0.0f;
  pumpT_ = seconds > 0 ? seconds : 0;
  return true;
}

bool Robot::startProgram() {
  if (!prog_.ok()) {
    error_ = prog_.error()[0] ? prog_.error() : "mission too long";
    prog_.clear();
    return false;
  }
  pc_ = 0;
  stepActive_ = false;
  state_ = State::Mission;
  return true;
}

bool Robot::cmdRowMission(int count, float firstDist, float spacing, float rowY, float sprayHeight) {
  if (!ready("mission")) return false;
  if (count < 1 || count > 8) {
    error_ = "count must be 1..8";
    return false;
  }
  prog_.clear();
  prog_.resetError();
  soilN_ = 0;
  buildRowMission(prog_, count, firstDist, spacing, rowY, sprayHeight);
  return startProgram();
}

bool Robot::cmdProbe(float rowY, float sprayHeight) {
  if (!ready("probe")) return false;
  prog_.clear();
  prog_.resetError();
  buildPlantCare(prog_, rowY, sprayHeight);
  prog_.armJoints(poseDeg(cfg::POSE_HOME));
  return startProgram();
}

bool Robot::cmdPick(const Vec3& p, float diameter) {
  if (!ready("pick")) return false;
  prog_.clear();
  prog_.resetError();
  buildPick(prog_, p, diameter);
  return startProgram();
}

bool Robot::cmdHome() {
  if (!ready("home")) return false;
  arm_.moveJoints(poseDeg(cfg::POSE_HOME));
  return true;
}

bool Robot::cmdStow() {
  if (!ready("stow")) return false;
  arm_.moveJoints(poseDeg(cfg::POSE_STOW));
  return true;
}

void Robot::cmdStop() {
  drive_.stop();
  arm_.hold();
  pumpOn_ = false;
  if (state_ != State::EStop) state_ = State::Idle;
  prog_.clear();
  pc_ = 0;
  stepActive_ = false;
}

void Robot::cmdEStop() {
  cmdStop();
  state_ = State::EStop;
  error_ = "e-stop";
}

bool Robot::cmdResetEStop() {
  if (in_.estop) {
    error_ = "release the e-stop button first";
    return false;
  }
  state_ = State::Idle;
  arm_.clearFault();
  error_ = "";
  return true;
}

// ------------------------------------------------------------------ mission
void Robot::beginStep(const Step& s) {
  switch (s.op) {
    case Op::Drive: drive_.driveDistance(s.a, s.b); break;
    case Op::Turn:  drive_.turnAngle(s.a, s.b); break;
    case Op::ArmJoints: arm_.moveJoints(ArmJoints{s.a, s.b, s.c, s.d}, s.e > 0 ? s.e : 1.0f); break;
    case Op::ArmMove:
      if (arm_.moveTo(Vec3{s.a, s.b, s.c}, s.d, s.tool, s.e > 0 ? s.e : 1.0f) != IkResult::Ok) fault("mission: arm target unreachable");
      break;
    case Op::ArmLinear:
      if (arm_.moveLinear(Vec3{s.a, s.b, s.c}, s.d, s.tool, s.e) != IkResult::Ok) fault("mission: linear target unreachable");
      break;
    case Op::Grip: arm_.setGrip(s.a); break;
    case Op::Wait: waitT_ = s.a; break;
    case Op::Pump: pumpOn_ = s.a > 0.5f; pumpT_ = 0; break;
    case Op::SampleSoil:
      if (soilN_ < cfg::MAX_SOIL_LOG) soilLog_[soilN_++] = soilPct_;
      break;
    case Op::SkipIfWet:
      if (soilN_ > 0 && soilLog_[soilN_ - 1] >= cfg::SOIL_DRY_PCT) pc_ += (int)(s.a + 0.5f);
      break;
    default: break;
  }
}

bool Robot::stepDone(const Step& s) const {
  switch (s.op) {
    case Op::Drive:
    case Op::Turn: return !drive_.busy();
    case Op::ArmJoints:
    case Op::ArmMove:
    case Op::ArmLinear:
    case Op::Grip: return !arm_.busy();
    case Op::Wait: return waitT_ <= 0;
    default: return true;
  }
}

void Robot::runMission() {
  // run instantaneous steps back to back, but never more than a few per tick
  for (int guard = 0; guard < 8 && state_ == State::Mission; ++guard) {
    if (pc_ >= prog_.size()) {
      state_ = State::Idle;
      prog_.clear();
      pc_ = 0;
      return;
    }
    const Step& s = prog_.at(pc_);
    if (!stepActive_) {
      beginStep(s);
      stepActive_ = true;
      if (state_ != State::Mission) return;
    }
    if (!stepDone(prog_.at(pc_))) return;
    stepActive_ = false;
    ++pc_;
  }
}

// --------------------------------------------------------------------- step
void Robot::step(const Inputs& in, Outputs& out) {
  in_ = in;
  const float dt = in.dt;
  time_ += dt;

  // sensors -> engineering units
  const float span = (float)(cfg::SOIL_RAW_DRY - cfg::SOIL_RAW_WET);
  soilPct_ = clampf(100.0f * (float)(cfg::SOIL_RAW_DRY - in.soilRaw) / span, 0.0f, 100.0f);
  battLow_ = in.battV < cfg::BATT_LOW_V;

  // safety interlocks
  if (in.estop && state_ != State::EStop) cmdEStop();
  if (state_ != State::EStop && state_ != State::Fault) {
    if (in.imuValid && (fabsf(in.roll) > cfg::TILT_LIMIT_RAD || fabsf(in.pitch) > cfg::TILT_LIMIT_RAD)) fault("tilt limit");
    else if (in.battV < cfg::BATT_CRIT_V && drive_.mode() != Drive::Mode::Idle) fault("battery critical");
    else if (arm_.fault()) fault("arm left its workspace");
  }
  drive_.inhibitForward(in.rangeM < cfg::OBSTACLE_STOP_M);

  if (waitT_ > 0) waitT_ -= dt;
  if (state_ == State::Mission) runMission();

  if (pumpOn_ && pumpT_ > 0) {
    pumpT_ -= dt;
    if (pumpT_ <= 0) pumpOn_ = false;
  }

  const bool live = state_ != State::EStop;
  if (state_ == State::EStop || state_ == State::Fault) drive_.stop();
  drive_.update(dt, in.encL, in.encR, in.gyroZ, in.imuValid);
  if (live) arm_.update(dt);
  if (state_ == State::Teleop && drive_.mode() == Drive::Mode::Idle) state_ = State::Idle;

  out.dutyL = live ? drive_.dutyL() : 0.0f;
  out.dutyR = live ? drive_.dutyR() : 0.0f;
  out.motorsEnabled = live && state_ != State::Fault;
  const ArmJoints& q = arm_.joints();
  out.joint[0] = q.yaw;
  out.joint[1] = q.shoulder;
  out.joint[2] = q.elbow;
  out.joint[3] = q.wrist;
  out.joint[4] = arm_.grip();
  out.servosEnabled = live;
  out.pump = live && pumpOn_;
  out.led = (state_ == State::EStop || state_ == State::Fault) ? 3 : (battLow_ ? 2 : (state_ == State::Idle ? 0 : 1));
}

int Robot::statusJson(char* buf, size_t n) const {
  const Pose2D& o = drive_.odom();
  const ArmJoints& q = arm_.joints();
  const ArmPose tcp = arm_.pose(Tool::Gripper);
  int k = snprintf(buf, n,
                   "{\"t\":%.2f,\"state\":\"%s\",\"err\":\"%s\",\"x\":%.3f,\"y\":%.3f,\"th\":%.1f,\"v\":%.3f,\"w\":%.2f,"
                   "\"batt\":%.2f,\"soil\":%.0f,\"range\":%.2f,\"temp\":%.1f,\"hum\":%.0f,\"pump\":%d,"
                   "\"arm\":[%.1f,%.1f,%.1f,%.1f],\"grip\":%.1f,\"tcp\":[%.0f,%.0f,%.0f],\"busy\":%d,"
                   "\"step\":%d,\"steps\":%d,\"log\":[",
                   time_, stateName(), error_, o.x, o.y, o.th / DEG, drive_.linVel(), drive_.angVel(),
                   in_.battV, soilPct_, in_.rangeM, in_.tempC, in_.humidity, pumpOn_ ? 1 : 0,
                   q.yaw / DEG, q.shoulder / DEG, q.elbow / DEG, q.wrist / DEG, arm_.grip() / DEG,
                   tcp.p.x * 1000.0f, tcp.p.y * 1000.0f, tcp.p.z * 1000.0f,
                   (arm_.busy() || drive_.busy()) ? 1 : 0, pc_, prog_.size());
  for (int i = 0; i < soilN_ && k > 0 && (size_t)k < n; ++i) {
    k += snprintf(buf + k, n - k, i ? ",%.0f" : "%.0f", soilLog_[i]);
  }
  if (k > 0 && (size_t)k < n) k += snprintf(buf + k, n - k, "]}");
  return k;
}

}  // namespace agro
