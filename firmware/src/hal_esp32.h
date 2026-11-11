// Hardware abstraction for the ESP32: fills agro::Inputs, applies agro::Outputs.
#pragma once

#include "robot.h"

namespace hal {

struct ServoCal {
  float zeroUs;      // pulse width at joint angle 0
  float usPerRad;    // signed: flips the direction of rotation
  float minUs, maxUs;
};

void begin();
void readInputs(agro::Inputs& in, float dt);
void applyOutputs(const agro::Outputs& out);

// servo calibration (persisted in NVS)
ServoCal& servoCal(int joint);
void saveCalibration();
void loadCalibration();
bool imuOk();
bool pwmOk();

}  // namespace hal
