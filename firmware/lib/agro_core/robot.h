// Top-level controller.  Hardware independent: the Arduino sketch (or the
// simulator) fills Inputs from real sensors, calls step(), and applies Outputs.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "arm_controller.h"
#include "drive.h"
#include "mission.h"

namespace agro {

struct Inputs {
  float dt = 0.01f;         // seconds since the previous step
  int32_t encL = 0;         // cumulative encoder ticks, forward positive
  int32_t encR = 0;
  bool imuValid = false;
  float gyroZ = 0;          // rad/s, positive = turning left
  float roll = 0, pitch = 0;  // rad, from the accelerometer
  float battV = 12.0f;      // volts
  int soilRaw = 0;          // ADC counts from the capacitive probe
  float rangeM = 9.9f;      // ultrasonic range, metres (>= 4 means "nothing")
  bool estop = false;       // e-stop button pressed
  float tempC = 0, humidity = 0;
};

struct Outputs {
  float dutyL = 0, dutyR = 0;     // -1..1 to the left / right motor drivers
  bool motorsEnabled = false;
  float joint[5] = {0, 0, 0, 0, 0};   // rad: yaw, shoulder, elbow, wrist, grip
  bool servosEnabled = false;
  bool pump = false;
  uint8_t led = 0;                // 0 idle, 1 working, 2 warning, 3 fault
};

enum class State : uint8_t { Idle = 0, Teleop, Mission, EStop, Fault };

class Robot {
 public:
  Robot();
  void reset();

  void step(const Inputs& in, Outputs& out);

  // ---- commands (all return false + set lastError() when rejected) ----
  bool cmdVelocity(float v, float w);
  bool cmdMove(float metres, float speed);
  bool cmdTurn(float radians, float rate);
  bool cmdArmJoints(const ArmJoints& q, float speedScale = 1.0f);
  bool cmdArmPose(const Vec3& p, float pitch, Tool tool);
  bool cmdArmLinear(const Vec3& p, float pitch, Tool tool, float speed);
  bool cmdGrip(float angle);
  bool cmdPump(float seconds);            // 0 = off, <0 = on until told otherwise
  bool cmdRowMission(int count, float firstDist, float spacing, float rowY, float sprayHeight);
  bool cmdProbe(float rowY, float sprayHeight);
  bool cmdPick(const Vec3& p, float diameter);
  bool cmdHome();
  bool cmdStow();
  void cmdStop();                         // stop everything, stay enabled
  void cmdEStop();                        // latch: needs cmdResetEStop()
  bool cmdResetEStop();

  // ---- state ----
  State state() const { return state_; }
  const char* stateName() const;
  const char* lastError() const { return error_; }
  const Drive& drive() const { return drive_; }
  const ArmController& arm() const { return arm_; }
  float soilPercent() const { return soilPct_; }
  int soilLogCount() const { return soilN_; }
  float soilLog(int i) const { return soilLog_[i]; }
  int missionStep() const { return pc_; }
  int missionLength() const { return prog_.size(); }
  bool pumpOn() const { return pumpOn_; }
  float uptime() const { return time_; }
  const Inputs& lastInputs() const { return in_; }
  // one-line JSON telemetry; returns characters written
  int statusJson(char* buf, size_t n) const;

 private:
  bool ready(const char* what);
  bool startProgram();
  void runMission();
  void beginStep(const Step& s);
  bool stepDone(const Step& s) const;
  void fault(const char* why);

  Drive drive_;
  ArmController arm_;
  Program prog_;
  State state_ = State::Idle;
  Inputs in_;
  int pc_ = 0;
  bool stepActive_ = false;
  float waitT_ = 0;
  float time_ = 0;
  bool pumpOn_ = false;
  float pumpT_ = 0;
  float soilPct_ = 0;
  float soilLog_[32];
  int soilN_ = 0;
  bool battLow_ = false;
  const char* error_ = "";
};

}  // namespace agro
