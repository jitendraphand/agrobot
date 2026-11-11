#include "kinematics.h"

#include "robot_geometry.h"

namespace agro {

const float* toolOffset(Tool t) {
  switch (t) {
    case Tool::Probe:  return geom::TCP_PROBE;
    case Tool::Nozzle: return geom::TCP_NOZZLE;
    default:           return geom::TCP_GRIPPER;
  }
}

bool jointsWithinLimits(const ArmJoints& q, float margin) {
  return q.yaw >= geom::YAW_MIN - margin && q.yaw <= geom::YAW_MAX + margin &&
         q.shoulder >= geom::SHOULDER_MIN - margin && q.shoulder <= geom::SHOULDER_MAX + margin &&
         q.elbow >= geom::ELBOW_MIN - margin && q.elbow <= geom::ELBOW_MAX + margin &&
         q.wrist >= geom::WRIST_MIN - margin && q.wrist <= geom::WRIST_MAX + margin;
}

ArmPose armFK(const ArmJoints& q, Tool tool) {
  const float* t = toolOffset(tool);
  const float a1 = q.shoulder;
  const float a2 = a1 + q.elbow;
  const float phi = a2 + q.wrist;                 // tool axis angle in the arm plane
  // wrist in the arm plane: rho = distance from the yaw axis, z = height
  float rho = geom::SHOULDER_OFF + geom::L1 * cosf(a1) + geom::L2 * cosf(a2);
  float z = geom::SHOULDER_Z + geom::L1 * sinf(a1) + geom::L2 * sinf(a2);
  rho += t[0] * cosf(phi) - t[2] * sinf(phi);
  z += t[0] * sinf(phi) + t[2] * cosf(phi);
  const float cy = cosf(q.yaw), sy = sinf(q.yaw);
  ArmPose out;
  out.p.x = geom::YAW_X + rho * cy - t[1] * sy;
  out.p.y = rho * sy + t[1] * cy;
  out.p.z = z;
  // Report pitch with the "pointing away from the base" convention.
  float pitch = wrapPi(phi);
  if (rho < 0.0f) pitch = wrapPi(PI_F - phi);   // reaching back over the shoulder
  out.pitch = pitch;
  return out;
}

static float jointDistance(const ArmJoints& a, const ArmJoints& b) {
  return fabsf(a.yaw - b.yaw) * 1.5f + fabsf(a.shoulder - b.shoulder) + fabsf(a.elbow - b.elbow) +
         fabsf(a.wrist - b.wrist) * 0.5f;
}

IkResult armIK(const Vec3& p, float pitch, Tool tool, ArmJoints& out, const ArmJoints* seed) {
  const float* t = toolOffset(tool);
  const float dx = p.x - geom::YAW_X;
  const float dy = p.y;
  const float d2 = dx * dx + dy * dy - t[1] * t[1];
  if (d2 < 1e-8f) return IkResult::Unreachable;
  const float dh = sqrtf(d2);

  bool anyReach = false;
  bool found = false;
  float bestCost = 1e9f;
  ArmJoints best = {0, 0, 0, 0};

  for (int back = 0; back < 2; ++back) {
    const float rho = back ? -dh : dh;                       // signed radius in the arm plane
    const float yaw = wrapPi(atan2f(dy, dx) - atan2f(t[1], rho));
    const float phi = back ? (PI_F - pitch) : pitch;         // tool angle in the arm plane
    // wrist centre in the arm plane, relative to the shoulder axis
    const float wr = rho - t[0] * cosf(phi) + t[2] * sinf(phi) - geom::SHOULDER_OFF;
    const float wz = p.z - t[0] * sinf(phi) - t[2] * cosf(phi) - geom::SHOULDER_Z;
    const float D2 = wr * wr + wz * wz;
    float c2 = (D2 - geom::L1 * geom::L1 - geom::L2 * geom::L2) / (2.0f * geom::L1 * geom::L2);
    if (c2 > 1.0f || c2 < -1.0f) {
      if (c2 > 1.0005f || c2 < -1.0005f) continue;           // out of reach
      c2 = clampf(c2, -1.0f, 1.0f);
    }
    anyReach = true;
    const float e = acosf(c2);
    for (int s = 0; s < 2; ++s) {
      ArmJoints q;
      q.yaw = yaw;
      q.elbow = s ? e : -e;
      q.shoulder = wrapPi(atan2f(wz, wr) - atan2f(geom::L2 * sinf(q.elbow), geom::L1 + geom::L2 * cosf(q.elbow)));
      q.wrist = wrapPi(phi - q.shoulder - q.elbow);
      if (!jointsWithinLimits(q)) continue;
      // preference when unseeded: forward reach, elbow above the wrist line
      float cost = seed ? jointDistance(q, *seed) : (back ? 2.0f : 0.0f) + ((back ? -q.elbow : q.elbow) > 0.0f ? 1.0f : 0.0f);
      if (cost < bestCost) {
        bestCost = cost;
        best = q;
        found = true;
      }
    }
  }
  if (!found) return anyReach ? IkResult::JointLimit : IkResult::Unreachable;
  out = best;
  return IkResult::Ok;
}

float gripAngleForDiameter(float diameter) {
  // Each pad face is a straight line GRIP_PAD_INSET from its pivot; the pivot
  // sits GRIP_PIVOT_HALF off the centre line and the object is centred
  // GRIP_PAD_REACH ahead of the pivots.  With opening angle a, the distance
  // from the object centre to the pad face is
  //     reach*sin(a) + half*cos(a) - inset
  // and contact happens when that equals the object radius.
  const float r = 0.5f * diameter;
  const float A = geom::GRIP_PAD_REACH, B = geom::GRIP_PIVOT_HALF;
  const float R = sqrtf(A * A + B * B);
  const float s = clampf((r + geom::GRIP_PAD_INSET) / R, -1.0f, 1.0f);
  const float a = asinf(s) - atan2f(B, A);
  return clampf(a, geom::GRIP_MIN, geom::GRIP_MAX);
}

}  // namespace agro
