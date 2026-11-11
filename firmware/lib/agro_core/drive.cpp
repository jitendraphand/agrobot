#include "drive.h"

#include "agro_config.h"
#include "robot_geometry.h"

namespace agro {

float WheelPI::update(float target, float measured, float dt) {
  const float err = target - measured;
  const float ff = target / geom::MOTOR_NOLOAD_RAD_S;
  float u = ff + cfg::WHEEL_KP * err + cfg::WHEEL_KI * integ_;
  // integrate only while not saturated (anti-windup)
  if ((u < 1.0f && u > -1.0f) || (u >= 1.0f && err < 0) || (u <= -1.0f && err > 0)) {
    integ_ += err * dt;
    integ_ = clampf(integ_, -1.2f, 1.2f);
  }
  u = ff + cfg::WHEEL_KP * err + cfg::WHEEL_KI * integ_;
  if (target == 0.0f && fabsf(measured) < 0.15f) {   // parked: release the motors
    integ_ = 0;
    return 0.0f;
  }
  if (u > 0.005f) u += cfg::WHEEL_DEADBAND;
  else if (u < -0.005f) u -= cfg::WHEEL_DEADBAND;
  return clampf(u, -1.0f, 1.0f);
}

void Drive::reset() {
  *this = Drive();
}

void Drive::setVelocity(float v, float w) {
  vTarget_ = clampf(v, -cfg::MAX_LIN_VEL, cfg::MAX_LIN_VEL);
  wTarget_ = clampf(w, -cfg::MAX_ANG_VEL, cfg::MAX_ANG_VEL);
  cmdAge_ = 0;
  mode_ = Mode::Velocity;
}

void Drive::driveDistance(float metres, float speed) {
  goalDist_ = metres;
  travelled_ = 0;
  moveSpeed_ = clampf(fabsf(speed), 0.03f, cfg::MAX_LIN_VEL);
  goalHeading_ = odom_.th;
  settle_ = 0;
  mode_ = Mode::Distance;
}

void Drive::turnAngle(float radians, float rate) {
  goalHeading_ = odom_.th + radians;
  turnRate_ = clampf(fabsf(rate), 0.1f, cfg::MAX_ANG_VEL);
  settle_ = 0;
  mode_ = Mode::Turn;
}

void Drive::stop() {
  mode_ = Mode::Idle;
  vTarget_ = wTarget_ = 0;
}

void Drive::command(float v, float w, float dt) {
  if (inhibitFwd_ && v > 0) v = 0;
  // acceleration limits
  const float dv = cfg::LIN_ACCEL * dt, dw = cfg::ANG_ACCEL * dt;
  vCmd_ += clampf(v - vCmd_, -dv, dv);
  wCmd_ += clampf(w - wCmd_, -dw, dw);
  const float half = 0.5f * geom::TRACK * cfg::TRACK_SLIP_FACTOR;
  const float wl = (vCmd_ - wCmd_ * half) / geom::WHEEL_RADIUS;
  const float wr = (vCmd_ + wCmd_ * half) / geom::WHEEL_RADIUS;
  dutyL_ = piL_.update(wl, wL_, dt);
  dutyR_ = piR_.update(wr, wR_, dt);
}

void Drive::update(float dt, int32_t ticksL, int32_t ticksR, float gyroZ, bool gyroValid) {
  if (dt <= 0) return;
  if (first_) {
    lastL_ = ticksL;
    lastR_ = ticksR;
    first_ = false;
  }
  // ---- measurement ----
  const float k = 2.0f * PI_F / geom::ENC_TICKS_PER_REV;
  const float dL = (float)(ticksL - lastL_) * k;      // wheel rotation, rad
  const float dR = (float)(ticksR - lastR_) * k;
  lastL_ = ticksL;
  lastR_ = ticksR;
  const float alpha = 0.35f;                          // low-pass on the speed estimate
  wL_ += alpha * (dL / dt - wL_);
  wR_ += alpha * (dR / dt - wR_);
  const float ds = 0.5f * (dL + dR) * geom::WHEEL_RADIUS;
  const float dthEnc = (dR - dL) * geom::WHEEL_RADIUS / (geom::TRACK * cfg::TRACK_SLIP_FACTOR);
  const float dth = gyroValid ? gyroZ * dt : dthEnc;  // the gyro is far better than slipping wheels
  odom_.x += ds * cosf(odom_.th + 0.5f * dth);
  odom_.y += ds * sinf(odom_.th + 0.5f * dth);
  odom_.th += dth;
  travelled_ += ds;
  vMeas_ = 0.5f * (wL_ + wR_) * geom::WHEEL_RADIUS;
  wMeas_ = gyroValid ? gyroZ : (wR_ - wL_) * geom::WHEEL_RADIUS / (geom::TRACK * cfg::TRACK_SLIP_FACTOR);

  // ---- control ----
  switch (mode_) {
    case Mode::Velocity: {
      cmdAge_ += dt;
      if (cmdAge_ > cfg::CMD_TIMEOUT) {               // link lost: coast to a stop
        vTarget_ = wTarget_ = 0;
        if (fabsf(vCmd_) < 0.01f && fabsf(wCmd_) < 0.02f) mode_ = Mode::Idle;
      }
      command(vTarget_, wTarget_, dt);
      break;
    }
    case Mode::Distance: {
      const float rem = goalDist_ - travelled_;
      float v = clampf(cfg::POS_KP * rem, -moveSpeed_, moveSpeed_);
      if (inhibitFwd_ && v > 0) v = 0;
      const float w = clampf(cfg::HEADING_KP * wrapPi(goalHeading_ - odom_.th), -0.8f, 0.8f);
      command(v, w, dt);
      if (fabsf(rem) < cfg::MOVE_TOL && fabsf(vMeas_) < 0.01f) {
        settle_ += dt;
        if (settle_ > 0.15f) mode_ = Mode::Idle;
      } else {
        settle_ = 0;
      }
      break;
    }
    case Mode::Turn: {
      const float err = goalHeading_ - odom_.th;
      const float w = clampf(cfg::HEADING_KP * err, -turnRate_, turnRate_);
      command(0, w, dt);
      if (fabsf(err) < cfg::TURN_TOL && fabsf(wMeas_) < 0.05f) {
        settle_ += dt;
        if (settle_ > 0.15f) mode_ = Mode::Idle;
      } else {
        settle_ = 0;
      }
      break;
    }
    case Mode::Idle:
    default:
      command(0, 0, dt);
      break;
  }
}

}  // namespace agro
