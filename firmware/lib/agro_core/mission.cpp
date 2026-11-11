#include "mission.h"

#include "agro_config.h"
#include "robot_geometry.h"

namespace agro {

void Program::add(Op op, float a, float b, float c, float d, float e, Tool tool) {
  if (n_ >= CAPACITY - 1) {
    overflow_ = true;
    return;
  }
  steps_[n_++] = Step{op, tool, a, b, c, d, e};
  steps_[n_] = Step{Op::End, Tool::Gripper, 0, 0, 0, 0, 0};
}

ArmJoints poseDeg(const float deg[4]) {
  return ArmJoints{deg[0] * DEG, deg[1] * DEG, deg[2] * DEG, deg[3] * DEG};
}

static ArmJoints carryPose(float side) {
  ArmJoints q = poseDeg(cfg::POSE_CARRY);
  q.yaw *= side;
  return q;
}

void buildPlantCare(Program& prog, float rowY, float sprayHeight) {
  const float side = rowY >= 0 ? 1.0f : -1.0f;
  // 40 mm on the robot side of the stem, level with the arm's yaw axis
  const Vec3 above = {geom::YAW_X, rowY - side * 0.040f, cfg::PROBE_CLEAR};
  const Vec3 inSoil = {above.x, above.y, -cfg::PROBE_DEPTH};
  const Vec3 spray = {geom::YAW_X, rowY - side * 0.030f, sprayHeight};
  ArmJoints q;
  if (armIK(above, 0.0f, Tool::Probe, q) != IkResult::Ok || armIK(inSoil, 0.0f, Tool::Probe, q) != IkResult::Ok) {
    prog.fail("probe point out of reach");
    return;
  }
  if (armIK(spray, 0.0f, Tool::Nozzle, q) != IkResult::Ok) {
    prog.fail("spray point out of reach");
    return;
  }
  prog.armJoints(carryPose(side));
  prog.armMove(above, 0.0f, Tool::Probe);
  prog.armLinear(inSoil, 0.0f, Tool::Probe, 0.05f);
  prog.wait(cfg::PROBE_SETTLE_S);
  prog.add(Op::SampleSoil);
  prog.armLinear(above, 0.0f, Tool::Probe, 0.07f);
  prog.add(Op::SkipIfWet, 5);                       // skip the five spray steps
  prog.armMove(spray, 0.0f, Tool::Nozzle);
  prog.pump(true);
  prog.wait(cfg::SPRAY_S);
  prog.pump(false);
  prog.wait(0.3f);
  prog.armJoints(carryPose(side));
}

void buildRowMission(Program& prog, int count, float firstDist, float spacing, float rowY, float sprayHeight) {
  for (int i = 0; i < count; ++i) {
    const float d = (i == 0) ? firstDist : spacing;
    if (fabsf(d) > 0.001f) prog.drive(d, 0.25f);
    prog.wait(0.3f);
    buildPlantCare(prog, rowY, sprayHeight);
    if (!prog.ok()) return;
  }
  prog.armJoints(poseDeg(cfg::POSE_HOME));
}

void buildPick(Program& prog, const Vec3& p, float diameter) {
  const float dx = p.x - geom::YAW_X, dy = p.y;
  const float r = sqrtf(dx * dx + dy * dy);
  if (r < 0.05f) {
    prog.fail("fruit too close to the arm axis");
    return;
  }
  const float ex = dx / r, ey = dy / r;
  const float side = dy >= 0 ? 1.0f : -1.0f;
  const Vec3 pre = {p.x - 0.05f * ex, p.y - 0.05f * ey, p.z};
  const Vec3 lift = {p.x, p.y, p.z + 0.05f};
  const Vec3 back = {pre.x, pre.y, p.z + 0.05f};
  const Vec3 drop = {geom::BASKET_X, geom::BASKET_Y, geom::BASKET_RIM_Z + 0.075f};
  const float dropPitch = -72.0f * DEG;
  ArmJoints q, qDrop;
  if (armIK(pre, 0.0f, Tool::Gripper, q) != IkResult::Ok || armIK(p, 0.0f, Tool::Gripper, q) != IkResult::Ok ||
      armIK(lift, 0.0f, Tool::Gripper, q) != IkResult::Ok) {
    prog.fail("fruit out of reach");
    return;
  }
  if (armIK(drop, dropPitch, Tool::Gripper, qDrop) != IkResult::Ok) {
    prog.fail("basket out of reach");
    return;
  }
  const float contact = gripAngleForDiameter(diameter);
  ArmJoints carry0 = poseDeg(cfg::POSE_CARRY);
  carry0.yaw = 0;
  prog.grip(clampf(contact + 0.30f, geom::GRIP_MIN, geom::GRIP_MAX));
  prog.armJoints(carryPose(side));
  prog.armMove(pre, 0.0f, Tool::Gripper);
  prog.armLinear(p, 0.0f, Tool::Gripper, 0.05f);
  prog.grip(clampf(contact - cfg::GRIP_SQUEEZE, geom::GRIP_MIN, geom::GRIP_MAX));
  prog.wait(0.5f);
  prog.armLinear(lift, 0.0f, Tool::Gripper, 0.04f);
  prog.armLinear(back, 0.0f, Tool::Gripper, 0.06f);
  prog.armJoints(carryPose(side), 0.6f);
  prog.armJoints(carry0, 0.6f);
  prog.armJoints(qDrop, 0.6f);
  prog.wait(0.3f);
  prog.grip(clampf(contact + 0.35f, geom::GRIP_MIN, geom::GRIP_MAX));
  prog.wait(0.6f);
  prog.armJoints(carry0);
  prog.armJoints(poseDeg(cfg::POSE_HOME));
  prog.grip(0.17f);
}

}  // namespace agro
