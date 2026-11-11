// Arm forward / inverse kinematics and small maths helpers.
//
// Joint convention (radians), identical in CAD, simulation and firmware:
//   yaw      0 = arm pointing forward (+X), positive = towards the left (+Y)
//   shoulder 0 = upper arm horizontal,      positive = up
//   elbow    0 = forearm in line with the upper arm, positive = up
//   wrist    0 = gripper in line with the forearm,   positive = up
//
// Cartesian targets are expressed in the robot frame (origin on the ground
// under the chassis centre, +X forward, +Y left, +Z up), in metres.
// "pitch" is the elevation of the gripper axis: 0 = horizontal pointing away
// from the arm base, -pi/2 = straight down.
#pragma once

#include <math.h>
#include <stdint.h>

namespace agro {

constexpr float PI_F = 3.14159265358979f;
constexpr float DEG = PI_F / 180.0f;

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline float wrapPi(float a) {
  while (a > PI_F) a -= 2.0f * PI_F;
  while (a <= -PI_F) a += 2.0f * PI_F;
  return a;
}
// quintic smooth-step: zero velocity and acceleration at both ends
inline float smooth5(float s) {
  s = clampf(s, 0.0f, 1.0f);
  return s * s * s * (10.0f + s * (-15.0f + 6.0f * s));
}

struct Vec3 {
  float x, y, z;
};

struct ArmJoints {
  float yaw, shoulder, elbow, wrist;
};

enum class Tool : uint8_t { Gripper = 0, Probe = 1, Nozzle = 2 };

struct ArmPose {
  Vec3 p;        // tool centre point, robot frame
  float pitch;   // see header comment
};

enum class IkResult : uint8_t { Ok = 0, Unreachable, JointLimit };

const float* toolOffset(Tool t);
bool jointsWithinLimits(const ArmJoints& q, float margin = 0.0f);

// Forward kinematics of the chosen tool centre point.
ArmPose armFK(const ArmJoints& q, Tool tool);

// Inverse kinematics.  Tries the four arm configurations (reach forward /
// reach back over the shoulder, elbow up / down) and returns the valid one
// closest to `seed` (or the most natural one if seed is null).
IkResult armIK(const Vec3& p, float pitch, Tool tool, ArmJoints& out, const ArmJoints* seed = nullptr);

// Finger opening angle (rad) at which the pads just touch an object of the
// given diameter held at the grasp centre.
float gripAngleForDiameter(float diameter);

}  // namespace agro
