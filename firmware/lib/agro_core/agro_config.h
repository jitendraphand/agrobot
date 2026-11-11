// AgroBot Mk1 - tunable parameters of the control core.
// Geometry lives in robot_geometry.h (generated from the CAD).
#pragma once

namespace agro {
namespace cfg {

// ---- loop timing ----
constexpr float CONTROL_HZ = 100.0f;

// ---- drive ----
constexpr float MAX_LIN_VEL = 0.40f;        // m/s   (motor no-load is ~0.52 m/s)
constexpr float MAX_ANG_VEL = 1.6f;         // rad/s
constexpr float LIN_ACCEL = 0.6f;           // m/s^2
constexpr float ANG_ACCEL = 4.0f;           // rad/s^2
// Skid-steer robots turn less than the geometric track suggests because the
// tyres must slip sideways.  Effective track = TRACK * TRACK_SLIP_FACTOR.
// Calibrate on your surface: command TURN 360 and adjust until it is 360.
constexpr float TRACK_SLIP_FACTOR = 1.45f;
constexpr float WHEEL_KP = 0.10f;           // duty per rad/s of wheel-speed error
constexpr float WHEEL_KI = 0.90f;           // duty per rad of integrated error
constexpr float WHEEL_DEADBAND = 0.04f;     // duty needed to overcome gearbox stiction
constexpr float POS_KP = 2.5f;              // 1/s   distance loop
constexpr float HEADING_KP = 3.0f;          // 1/s   heading hold / turn loop
constexpr float MOVE_TOL = 0.0025f;         // m
constexpr float TURN_TOL = 0.012f;          // rad (~0.7 deg)
constexpr float CMD_TIMEOUT = 0.6f;         // s: teleop velocity watchdog

// ---- arm ----
constexpr float JOINT_VEL[5] = {1.2f, 0.9f, 1.4f, 2.0f, 2.5f};   // rad/s: yaw, shoulder, elbow, wrist, grip
constexpr float LINEAR_VEL = 0.08f;         // m/s default Cartesian speed
constexpr float MIN_MOVE_TIME = 0.35f;      // s
constexpr float GRIP_SQUEEZE = 0.07f;       // rad closed beyond first contact (~4 deg)

// named poses: yaw, shoulder, elbow, wrist (degrees)
constexpr float POSE_HOME[4] = {0.0f, 115.0f, -105.0f, -10.0f};
constexpr float POSE_STOW[4] = {0.0f, 150.0f, -150.0f, 0.0f};
constexpr float POSE_CARRY[4] = {90.0f, 100.0f, -95.0f, -5.0f};

// ---- safety ----
constexpr float BATT_LOW_V = 10.2f;         // 3S: warn
constexpr float BATT_CRIT_V = 9.6f;         // 3S: stop driving
constexpr float OBSTACLE_STOP_M = 0.22f;    // ultrasonic: refuse to drive forward closer than this
constexpr float TILT_LIMIT_RAD = 0.52f;     // ~30 deg roll / pitch -> stop

// ---- agronomy ----
constexpr float SOIL_DRY_PCT = 35.0f;       // spray when volumetric reading is below this
constexpr int SOIL_RAW_DRY = 2900;          // ADC counts in dry air  (calibrate!)
constexpr int SOIL_RAW_WET = 1250;          // ADC counts in a glass of water (calibrate!)
constexpr float PROBE_DEPTH = 0.040f;       // m into the soil
constexpr float PROBE_CLEAR = 0.070f;       // m above the soil while travelling to the spot
constexpr float PROBE_SETTLE_S = 1.0f;
constexpr float SPRAY_S = 2.0f;
constexpr float SPRAY_HEIGHT = 0.40f;       // nozzle height above the soil (just over the canopy)

constexpr int MAX_SOIL_LOG = 24;

}  // namespace cfg
}  // namespace agro
