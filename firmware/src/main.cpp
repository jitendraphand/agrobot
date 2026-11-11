// AgroBot Mk1 firmware - ESP32 DevKit V1
//
// All behaviour lives in lib/agro_core (portable C++, unit-tested on a PC and
// run in the MuJoCo simulation).  This file only wires that core to the
// hardware: a 100 Hz control tick, the USB serial console and the web UI.
#include <Arduino.h>

#include "agro_config.h"
#include "hal_esp32.h"
#include "protocol.h"
#include "robot.h"
#include "web_server.h"

static agro::Robot robot;
static agro::Inputs inputs;
static agro::Outputs outputs;
static char reply[1024];

static const uint32_t TICK_US = (uint32_t)(1000000.0f / agro::cfg::CONTROL_HZ);
static uint32_t nextTick = 0;

// Servo calibration commands are hardware-specific, so they are handled here
// rather than in the portable core:
//   CAL SHOW                       print the table
//   CAL <joint 0-4> <zero_us> <us_per_deg>
//   CAL SAVE                       store in flash
static bool handleCal(const char* line) {
  if (strncasecmp(line, "CAL", 3) != 0) return false;
  const char* p = line + 3;
  while (*p == ' ') ++p;
  if (strncasecmp(p, "SAVE", 4) == 0) {
    hal::saveCalibration();
    Serial.println("OK saved");
  } else if (strncasecmp(p, "SHOW", 4) == 0 || *p == 0) {
    static const char* names[5] = {"yaw", "shoulder", "elbow", "wrist", "grip"};
    for (int j = 0; j < 5; ++j) {
      const hal::ServoCal& c = hal::servoCal(j);
      Serial.printf("%d %-8s zero %.0f us   %.2f us/deg   limits %.0f..%.0f us\n", j, names[j], c.zeroUs,
                    c.usPerRad * agro::DEG, c.minUs, c.maxUs);
    }
  } else {
    int j = -1;
    float zero = 0, perDeg = 0;
    if (sscanf(p, "%d %f %f", &j, &zero, &perDeg) == 3 && j >= 0 && j < 5 && zero > 300 && zero < 2700) {
      hal::servoCal(j).zeroUs = zero;
      hal::servoCal(j).usPerRad = perDeg / agro::DEG;
      Serial.println("OK (CAL SAVE to keep)");
    } else {
      Serial.println("ERR usage: CAL SHOW | CAL SAVE | CAL <joint 0-4> <zero_us> <us_per_deg>");
    }
  }
  return true;
}

static void pollSerial() {
  static char line[128];
  static size_t len = 0;
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (len == 0) continue;
      line[len] = 0;
      len = 0;
      if (!handleCal(line)) {
        agro::handleCommand(robot, line, reply, sizeof(reply));
        Serial.println(reply);
      }
    } else if (len < sizeof(line) - 1) {
      line[len++] = c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nAgroBot Mk1");
  hal::begin();
  Serial.printf("PCA9685 %s, MPU6050 %s\n", hal::pwmOk() ? "ok" : "MISSING", hal::imuOk() ? "ok" : "MISSING");
  net::begin(robot);
  nextTick = micros() + TICK_US;
}

void loop() {
  const uint32_t now = micros();
  if ((int32_t)(now - nextTick) >= 0) {
    // fixed-rate control tick; if we ever fall behind, skip rather than burst
    static uint32_t last = 0;
    const float dt = last ? (float)(now - last) * 1e-6f : 1.0f / agro::cfg::CONTROL_HZ;
    last = now;
    nextTick += TICK_US;
    if ((int32_t)(now - nextTick) > (int32_t)(4 * TICK_US)) nextTick = now + TICK_US;
    hal::readInputs(inputs, dt);
    robot.step(inputs, outputs);
    hal::applyOutputs(outputs);
  }
  pollSerial();
  net::poll();
}
