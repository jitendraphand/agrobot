// Skid-steer drive: wheel-speed PI control, odometry, distance / turn moves.
#pragma once

#include <stdint.h>

#include "kinematics.h"

namespace agro {

struct Pose2D {
  float x = 0, y = 0, th = 0;
};

class WheelPI {
 public:
  // returns duty in [-1, 1]
  float update(float target, float measured, float dt);
  void reset() { integ_ = 0; }

 private:
  float integ_ = 0;
};

class Drive {
 public:
  enum class Mode : uint8_t { Idle, Velocity, Distance, Turn };

  void reset();

  // --- commands ---
  void setVelocity(float v, float w);                 // teleop: m/s, rad/s (watchdog applies)
  void driveDistance(float metres, float speed);      // straight move with heading hold
  void turnAngle(float radians, float rate);          // spin in place
  void stop();

  // --- periodic update ---
  // ticksL / ticksR: cumulative encoder counts (forward positive on both sides)
  void update(float dt, int32_t ticksL, int32_t ticksR, float gyroZ, bool gyroValid);

  // --- outputs ---
  float dutyL() const { return dutyL_; }
  float dutyR() const { return dutyR_; }
  const Pose2D& odom() const { return odom_; }
  float linVel() const { return vMeas_; }
  float angVel() const { return wMeas_; }
  Mode mode() const { return mode_; }
  bool busy() const { return mode_ == Mode::Distance || mode_ == Mode::Turn; }
  float wheelSpeedL() const { return wL_; }
  float wheelSpeedR() const { return wR_; }
  void inhibitForward(bool b) { inhibitFwd_ = b; }

 private:
  void command(float v, float w, float dt);

  Mode mode_ = Mode::Idle;
  bool first_ = true;
  bool inhibitFwd_ = false;
  int32_t lastL_ = 0, lastR_ = 0;
  float wL_ = 0, wR_ = 0;            // measured wheel speeds (rad/s, filtered)
  float vMeas_ = 0, wMeas_ = 0;
  float vCmd_ = 0, wCmd_ = 0;        // ramped set-points
  float vTarget_ = 0, wTarget_ = 0;  // teleop targets
  float cmdAge_ = 0;
  float dutyL_ = 0, dutyR_ = 0;
  WheelPI piL_, piR_;
  Pose2D odom_;
  // distance / turn move state
  float goalDist_ = 0, travelled_ = 0, moveSpeed_ = 0;
  float goalHeading_ = 0, turnRate_ = 0;
  float settle_ = 0;
};

}  // namespace agro
