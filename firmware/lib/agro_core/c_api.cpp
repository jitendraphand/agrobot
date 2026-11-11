// Plain-C interface to the control core, used by the simulator (ctypes) so
// that the very same code that runs on the ESP32 drives the physics model.
// Not compiled into the firmware.
#include <string.h>

#include "protocol.h"
#include "robot.h"

extern "C" {

struct AgroIn {
  float dt;
  int encL, encR;
  int imuValid;
  float gyroZ, roll, pitch;
  float battV;
  int soilRaw;
  float rangeM;
  int estop;
};

struct AgroOut {
  float dutyL, dutyR;
  int motorsEnabled;
  float joint[5];
  int servosEnabled;
  int pump;
  int led;
  int state;
  int busy;
  float odomX, odomY, odomTh;
};

void* agro_create() { return new agro::Robot(); }
void agro_destroy(void* h) { delete static_cast<agro::Robot*>(h); }

void agro_step(void* h, const AgroIn* in, AgroOut* out) {
  agro::Robot* r = static_cast<agro::Robot*>(h);
  agro::Inputs i;
  i.dt = in->dt;
  i.encL = in->encL;
  i.encR = in->encR;
  i.imuValid = in->imuValid != 0;
  i.gyroZ = in->gyroZ;
  i.roll = in->roll;
  i.pitch = in->pitch;
  i.battV = in->battV;
  i.soilRaw = in->soilRaw;
  i.rangeM = in->rangeM;
  i.estop = in->estop != 0;
  agro::Outputs o;
  r->step(i, o);
  out->dutyL = o.dutyL;
  out->dutyR = o.dutyR;
  out->motorsEnabled = o.motorsEnabled;
  for (int k = 0; k < 5; ++k) out->joint[k] = o.joint[k];
  out->servosEnabled = o.servosEnabled;
  out->pump = o.pump;
  out->led = o.led;
  out->state = (int)r->state();
  out->busy = (r->arm().busy() || r->drive().busy()) ? 1 : 0;
  out->odomX = r->drive().odom().x;
  out->odomY = r->drive().odom().y;
  out->odomTh = r->drive().odom().th;
}

int agro_command(void* h, const char* line, char* reply, int n) {
  return agro::handleCommand(*static_cast<agro::Robot*>(h), line, reply, (size_t)n) ? 1 : 0;
}

int agro_ik(float x, float y, float z, float pitch, int tool, float* q4) {
  agro::ArmJoints q;
  const agro::IkResult r = agro::armIK(agro::Vec3{x, y, z}, pitch, (agro::Tool)tool, q);
  if (r == agro::IkResult::Ok) {
    q4[0] = q.yaw;
    q4[1] = q.shoulder;
    q4[2] = q.elbow;
    q4[3] = q.wrist;
  }
  return (int)r;
}

void agro_fk(const float* q4, int tool, float* xyzp) {
  const agro::ArmPose p = agro::armFK(agro::ArmJoints{q4[0], q4[1], q4[2], q4[3]}, (agro::Tool)tool);
  xyzp[0] = p.p.x;
  xyzp[1] = p.p.y;
  xyzp[2] = p.p.z;
  xyzp[3] = p.pitch;
}

float agro_grip_angle(float diameter) { return agro::gripAngleForDiameter(diameter); }

}  // extern "C"
