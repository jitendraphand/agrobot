// AgroBot Mk1 - ESP32 DevKit V1 pin map.  Keep in sync with docs/wiring.md.
#pragma once

// I2C bus: PCA9685 (0x40), MPU6050 (0x68), AHT20 (0x38)
constexpr int PIN_SDA = 21;
constexpr int PIN_SCL = 22;

// BTS7960 motor drivers (one per side; the two motors of a side are in parallel)
constexpr int PIN_L_RPWM = 25;   // left  forward PWM
constexpr int PIN_L_LPWM = 26;   // left  reverse PWM
constexpr int PIN_R_RPWM = 32;   // right forward PWM
constexpr int PIN_R_LPWM = 33;   // right reverse PWM
constexpr int PIN_MOTOR_EN = 27; // R_EN + L_EN of both drivers tied together

// Hall encoders of the two REAR motors (power them from 3V3 so the signals are 3.3 V)
constexpr int PIN_ENC_L_A = 18;
constexpr int PIN_ENC_L_B = 19;
constexpr int PIN_ENC_R_A = 16;
constexpr int PIN_ENC_R_B = 17;

// HC-SR04 (ECHO is 5 V: use a 1k / 2k divider)
constexpr int PIN_US_TRIG = 23;
constexpr int PIN_US_ECHO = 39;  // input only (VN)

// Analog inputs (ADC1 only - ADC2 is unusable while WiFi is on)
constexpr int PIN_SOIL = 34;     // capacitive soil moisture sensor AOUT
constexpr int PIN_BATT = 35;     // battery through a 33k / 10k divider

// Outputs
constexpr int PIN_PUMP = 13;     // MOSFET module driving the 12 V pump
constexpr int PIN_SERVO_OE = 14; // PCA9685 /OE (low = outputs enabled)
constexpr int PIN_LED = 2;       // on-board LED
constexpr int PIN_BUZZER = 4;    // active buzzer (optional)

// Input
constexpr int PIN_ESTOP = 15;    // NC e-stop contact to GND, internal pull-up: open = STOP

// PCA9685 channel of each joint: yaw, shoulder, elbow, wrist, gripper
constexpr int SERVO_CHANNEL[5] = {0, 1, 2, 3, 4};

// Battery divider: Vbatt = Vadc * (R_TOP + R_BOTTOM) / R_BOTTOM
constexpr float BATT_DIVIDER = (33.0f + 10.0f) / 10.0f;
