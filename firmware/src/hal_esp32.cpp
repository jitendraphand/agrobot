#include "hal_esp32.h"

#include <Arduino.h>
#include <Preferences.h>
#include <Wire.h>

#include "kinematics.h"
#include "pins.h"

namespace hal {

// ----------------------------------------------------------------- encoders
static volatile int32_t g_encL = 0, g_encR = 0;
static volatile uint8_t g_stL = 0, g_stR = 0;
// quadrature transition table: index = (previous AB << 2) | current AB
static const int8_t QUAD[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

static void IRAM_ATTR isrEncL() {
  const uint8_t s = (uint8_t)((digitalRead(PIN_ENC_L_A) << 1) | digitalRead(PIN_ENC_L_B));
  g_encL += QUAD[(g_stL << 2) | s];
  g_stL = s;
}
static void IRAM_ATTR isrEncR() {
  const uint8_t s = (uint8_t)((digitalRead(PIN_ENC_R_A) << 1) | digitalRead(PIN_ENC_R_B));
  g_encR += QUAD[(g_stR << 2) | s];
  g_stR = s;
}
// Set to -1 if a side counts backwards when the robot drives forward.
static const int ENC_SIGN_L = 1;
static const int ENC_SIGN_R = -1;      // right motor is mirrored

// --------------------------------------------------------------- ultrasonic
static volatile uint32_t g_echoRise = 0, g_echoWidth = 0;
static volatile bool g_echoNew = false;
static void IRAM_ATTR isrEcho() {
  if (digitalRead(PIN_US_ECHO)) {
    g_echoRise = micros();
  } else {
    g_echoWidth = micros() - g_echoRise;
    g_echoNew = true;
  }
}

// --------------------------------------------------------------- motor PWM
static const int PWM_FREQ = 20000;     // above hearing
static const int PWM_BITS = 10;
static const int PWM_MAX = (1 << PWM_BITS) - 1;
static const int MOTOR_PINS[4] = {PIN_L_RPWM, PIN_L_LPWM, PIN_R_RPWM, PIN_R_LPWM};

static void pwmBegin() {
  for (int i = 0; i < 4; ++i) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(MOTOR_PINS[i], PWM_FREQ, PWM_BITS);
#else
    ledcSetup(i, PWM_FREQ, PWM_BITS);
    ledcAttachPin(MOTOR_PINS[i], i);
#endif
  }
}
static void pwmWrite(int index, int value) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(MOTOR_PINS[index], value);
#else
  ledcWrite(index, value);
#endif
}
// Flip if a side runs backwards on a positive duty.
static const int MOTOR_SIGN_L = 1;
static const int MOTOR_SIGN_R = -1;    // right motors are mirrored

static void motorWrite(int fwdIdx, int revIdx, float duty) {
  const int v = (int)(fabsf(duty) * PWM_MAX + 0.5f);
  pwmWrite(fwdIdx, duty > 0 ? v : 0);
  pwmWrite(revIdx, duty < 0 ? v : 0);
}

// ------------------------------------------------------------------ PCA9685
static const uint8_t PCA_ADDR = 0x40;
static bool g_pcaOk = false;

static bool i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}
static bool i2cRead(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, (int)n) != n) return false;
  for (uint8_t i = 0; i < n; ++i) buf[i] = (uint8_t)Wire.read();
  return true;
}

static const float PCA_OSC_HZ = 25000000.0f;   // trim per board if pulses are a few % off
static const float SERVO_HZ = 50.0f;

static void pcaBegin() {
  g_pcaOk = i2cWrite8(PCA_ADDR, 0x00, 0x10);                    // MODE1: sleep
  const uint8_t prescale = (uint8_t)(PCA_OSC_HZ / (4096.0f * SERVO_HZ) - 1.0f + 0.5f);
  i2cWrite8(PCA_ADDR, 0xFE, prescale);
  i2cWrite8(PCA_ADDR, 0x00, 0x20);                              // wake, auto-increment
  delay(1);
  i2cWrite8(PCA_ADDR, 0x01, 0x04);                              // MODE2: totem pole
}
static void pcaWriteUs(int channel, float us) {
  // us < 0 -> output fully off (servo goes limp)
  uint16_t off = 0x1000;
  if (us >= 0) off = (uint16_t)(us * 4096.0f * SERVO_HZ / 1e6f + 0.5f) & 0x0FFF;
  Wire.beginTransmission(PCA_ADDR);
  Wire.write((uint8_t)(0x06 + 4 * channel));
  Wire.write((uint8_t)0);
  Wire.write((uint8_t)0);
  Wire.write((uint8_t)(off & 0xFF));
  Wire.write((uint8_t)(off >> 8));
  Wire.endTransmission();
}

// ----------------------------------------------------------------- servo cal
// Defaults assume: yaw + elbow = 270 deg servos, the rest 180 deg servos, all
// driven 500..2500 us.  Every robot needs its own zeroUs: see docs/firmware.md.
static const float US_PER_RAD_180 = 2000.0f / agro::PI_F;
static const float US_PER_RAD_270 = 2000.0f / (1.5f * agro::PI_F);
static ServoCal g_cal[5] = {
    {1500.0f, US_PER_RAD_270, 500.0f, 2500.0f},     // yaw:      centre = straight ahead
    {500.0f, US_PER_RAD_180, 500.0f, 2500.0f},      // shoulder: 0 deg = horizontal forward
    {1611.0f, US_PER_RAD_270, 500.0f, 2500.0f},     // elbow:    -150..+120 deg
    {1500.0f, US_PER_RAD_180, 500.0f, 2500.0f},     // wrist:    centre = in line
    {900.0f, US_PER_RAD_180, 600.0f, 2000.0f},      // gripper:  0 = closed
};
ServoCal& servoCal(int joint) { return g_cal[joint < 0 ? 0 : (joint > 4 ? 4 : joint)]; }

void saveCalibration() {
  Preferences p;
  p.begin("agrobot", false);
  p.putBytes("servocal", g_cal, sizeof(g_cal));
  p.end();
}
void loadCalibration() {
  Preferences p;
  p.begin("agrobot", true);
  if (p.getBytesLength("servocal") == sizeof(g_cal)) p.getBytes("servocal", g_cal, sizeof(g_cal));
  p.end();
}

// ------------------------------------------------------------------ MPU6050
static const uint8_t MPU_ADDR = 0x68;
static bool g_imuOk = false;
static float g_gyroBiasZ = 0;
static const float GYRO_LSB = 65.5f;     // +-500 deg/s
static const float ACC_LSB = 8192.0f;    // +-4 g

static bool mpuRead(float& ax, float& ay, float& az, float& gz) {
  uint8_t b[14];
  if (!i2cRead(MPU_ADDR, 0x3B, b, 14)) return false;
  ax = (float)(int16_t)((b[0] << 8) | b[1]) / ACC_LSB;
  ay = (float)(int16_t)((b[2] << 8) | b[3]) / ACC_LSB;
  az = (float)(int16_t)((b[4] << 8) | b[5]) / ACC_LSB;
  gz = (float)(int16_t)((b[12] << 8) | b[13]) / GYRO_LSB * agro::DEG;
  return true;
}
static void mpuBegin() {
  g_imuOk = i2cWrite8(MPU_ADDR, 0x6B, 0x01);      // wake, PLL on gyro X
  if (!g_imuOk) return;
  i2cWrite8(MPU_ADDR, 0x1A, 0x03);                // DLPF 44 Hz
  i2cWrite8(MPU_ADDR, 0x1B, 0x08);                // gyro +-500 deg/s
  i2cWrite8(MPU_ADDR, 0x1C, 0x08);                // accel +-4 g
  delay(100);
  // gyro bias: the robot must be standing still while it boots
  float sum = 0, ax, ay, az, gz;
  int n = 0;
  for (int i = 0; i < 200; ++i) {
    if (mpuRead(ax, ay, az, gz)) {
      sum += gz;
      ++n;
    }
    delay(3);
  }
  g_gyroBiasZ = n ? sum / (float)n : 0;
}

// -------------------------------------------------------------------- AHT20
static const uint8_t AHT_ADDR = 0x38;
static bool g_ahtOk = false;
static float g_tempC = 0, g_hum = 0;
static uint32_t g_ahtT = 0;
static bool g_ahtPending = false;

static void ahtBegin() {
  Wire.beginTransmission(AHT_ADDR);
  Wire.write((uint8_t)0xBE);
  Wire.write((uint8_t)0x08);
  Wire.write((uint8_t)0x00);
  g_ahtOk = Wire.endTransmission() == 0;
}
static void ahtPoll(uint32_t now) {
  if (!g_ahtOk) return;
  if (!g_ahtPending && now - g_ahtT > 2000) {
    Wire.beginTransmission(AHT_ADDR);
    Wire.write((uint8_t)0xAC);
    Wire.write((uint8_t)0x33);
    Wire.write((uint8_t)0x00);
    Wire.endTransmission();
    g_ahtPending = true;
    g_ahtT = now;
  } else if (g_ahtPending && now - g_ahtT > 90) {
    uint8_t b[6];
    if (Wire.requestFrom((int)AHT_ADDR, 6) == 6) {
      for (int i = 0; i < 6; ++i) b[i] = (uint8_t)Wire.read();
      const uint32_t rh = ((uint32_t)b[1] << 12) | ((uint32_t)b[2] << 4) | (b[3] >> 4);
      const uint32_t t = (((uint32_t)b[3] & 0x0F) << 16) | ((uint32_t)b[4] << 8) | b[5];
      g_hum = (float)rh * 100.0f / 1048576.0f;
      g_tempC = (float)t * 200.0f / 1048576.0f - 50.0f;
    }
    g_ahtPending = false;
    g_ahtT = now;
  }
}

// -------------------------------------------------------------------- public
bool imuOk() { return g_imuOk; }
bool pwmOk() { return g_pcaOk; }

void begin() {
  pinMode(PIN_MOTOR_EN, OUTPUT);
  digitalWrite(PIN_MOTOR_EN, LOW);
  pinMode(PIN_PUMP, OUTPUT);
  digitalWrite(PIN_PUMP, LOW);
  pinMode(PIN_SERVO_OE, OUTPUT);
  digitalWrite(PIN_SERVO_OE, HIGH);              // servo outputs off until the first valid command
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  pinMode(PIN_ESTOP, INPUT_PULLUP);
  pinMode(PIN_US_TRIG, OUTPUT);
  digitalWrite(PIN_US_TRIG, LOW);
  pinMode(PIN_US_ECHO, INPUT);
  pinMode(PIN_ENC_L_A, INPUT_PULLUP);
  pinMode(PIN_ENC_L_B, INPUT_PULLUP);
  pinMode(PIN_ENC_R_A, INPUT_PULLUP);
  pinMode(PIN_ENC_R_B, INPUT_PULLUP);

  pwmBegin();
  for (int i = 0; i < 4; ++i) pwmWrite(i, 0);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrEncL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_B), isrEncL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrEncR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_B), isrEncR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_US_ECHO), isrEcho, CHANGE);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_SOIL, ADC_11db);
  analogSetPinAttenuation(PIN_BATT, ADC_11db);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  pcaBegin();
  mpuBegin();
  ahtBegin();
  loadCalibration();
}

void readInputs(agro::Inputs& in, float dt) {
  static float battFilt = 0, rangeFilt = 9.9f, rollF = 0, pitchF = 0;
  static uint32_t lastPing = 0, lastEcho = 0;
  const uint32_t now = millis();

  in.dt = dt;
  noInterrupts();
  const int32_t el = g_encL, er = g_encR;
  const bool echoNew = g_echoNew;
  const uint32_t width = g_echoWidth;
  g_echoNew = false;
  interrupts();
  in.encL = ENC_SIGN_L * el;
  in.encR = ENC_SIGN_R * er;

  // IMU
  float ax, ay, az, gz;
  if (g_imuOk && mpuRead(ax, ay, az, gz)) {
    in.imuValid = true;
    in.gyroZ = gz - g_gyroBiasZ;
    // board mounted flat, X forward, Y left, Z up
    const float roll = atan2f(ay, az);
    const float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    rollF += 0.05f * (roll - rollF);              // accelerometer tilt, heavily filtered
    pitchF += 0.05f * (pitch - pitchF);
    in.roll = rollF;
    in.pitch = pitchF;
  } else {
    in.imuValid = false;
    in.gyroZ = 0;
    in.roll = in.pitch = 0;
  }

  // battery
  const float vb = (float)analogReadMilliVolts(PIN_BATT) * 0.001f * BATT_DIVIDER;
  battFilt = battFilt == 0 ? vb : battFilt + 0.02f * (vb - battFilt);
  in.battV = battFilt;

  // soil probe (raw counts; converted to % by the core using agro_config.h)
  in.soilRaw = analogRead(PIN_SOIL);

  // ultrasonic: ping every 60 ms, echo measured by interrupt
  if (echoNew) {
    const float m = (float)width * 0.0001715f;    // us -> metres (343 m/s, there and back)
    if (m > 0.02f && m < 4.0f) rangeFilt += 0.5f * (m - rangeFilt);
    lastEcho = now;
  }
  if (now - lastEcho > 300) rangeFilt = 9.9f;     // nothing in range (or sensor missing)
  if (now - lastPing >= 60) {
    lastPing = now;
    digitalWrite(PIN_US_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_US_TRIG, LOW);
  }
  in.rangeM = rangeFilt;

  in.estop = digitalRead(PIN_ESTOP) == HIGH;      // NC contact opened (or wire broken)

  ahtPoll(now);
  in.tempC = g_tempC;
  in.humidity = g_hum;
}

void applyOutputs(const agro::Outputs& out) {
  // drive motors
  digitalWrite(PIN_MOTOR_EN, out.motorsEnabled ? HIGH : LOW);
  motorWrite(0, 1, out.motorsEnabled ? (float)MOTOR_SIGN_L * out.dutyL : 0.0f);
  motorWrite(2, 3, out.motorsEnabled ? (float)MOTOR_SIGN_R * out.dutyR : 0.0f);

  // servos at 50 Hz
  static uint32_t lastServo = 0;
  const uint32_t now = millis();
  if (now - lastServo >= 20) {
    lastServo = now;
    digitalWrite(PIN_SERVO_OE, out.servosEnabled ? LOW : HIGH);
    for (int j = 0; j < 5; ++j) {
      const ServoCal& c = g_cal[j];
      const float us = agro::clampf(c.zeroUs + c.usPerRad * out.joint[j], c.minUs, c.maxUs);
      pcaWriteUs(SERVO_CHANNEL[j], out.servosEnabled ? us : -1.0f);
    }
  }

  digitalWrite(PIN_PUMP, out.pump ? HIGH : LOW);

  // LED: off = idle, on = working, slow blink = low battery, fast blink = fault / e-stop
  bool led = false;
  switch (out.led) {
    case 1: led = true; break;
    case 2: led = (now / 500) & 1; break;
    case 3: led = (now / 100) & 1; break;
    default: break;
  }
  digitalWrite(PIN_LED, led ? HIGH : LOW);
  digitalWrite(PIN_BUZZER, (out.led == 3 && ((now / 100) % 10) == 0) ? HIGH : LOW);
}

}  // namespace hal
