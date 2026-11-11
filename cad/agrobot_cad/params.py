"""AgroBot Mk1 - single source of truth for every dimension.

Units: millimetres, degrees.  Robot frame: +X forward, +Y left, +Z up,
origin on the ground under the centre of the chassis.

Everything downstream is generated from this file:
  * CadQuery parts  -> STEP / STL          (cad/build.py)
  * kinematic manifest -> MuJoCo model     (simulation/gen_mjcf.py)
  * Blender scenes                         (blender/build_scene.py)
  * firmware geometry header               (tools/gen_firmware_header.py)
"""
from math import cos, radians, sqrt

# --------------------------------------------------------------------------
# Printer envelope (Elegoo Neptune 4 Max)
# --------------------------------------------------------------------------
PRINTER = "Elegoo Neptune 4 Max"
BED_X, BED_Y, BED_Z = 420.0, 420.0, 480.0
BED_MARGIN = 10.0            # keep-out for skirt / brim / bed clips

# --------------------------------------------------------------------------
# Fasteners (all metric, all easy to find in any Indian hardware market)
# --------------------------------------------------------------------------
M3_CLEAR = 3.4               # clearance hole
M3_INSERT = 4.0              # hole for M3 x 5 (OD 4.6) brass heat-set insert
M3_INSERT_DEPTH = 6.5
M3_SELF_TAP = 2.7            # pilot when threading straight into PETG
M3_HEAD_D = 6.2
M4_CLEAR = 4.4

# --------------------------------------------------------------------------
# Standard-size servo envelope (MG996R / DS3218 / DS3235 all share it)
# Servo frame: origin on the output axis at the tab seating plane,
# +Z out of the output shaft, +X from the output towards the far end of the case.
# --------------------------------------------------------------------------
SERVO_BODY_L = 40.5
SERVO_BODY_W = 20.0
SERVO_POCKET_L = 41.0        # printed pocket (body + clearance)
SERVO_POCKET_W = 20.6
SERVO_BELOW = 28.0           # tab seat -> bottom of case
SERVO_ABOVE = 10.0           # tab top -> top of case
SERVO_TAB_T = 2.5
SERVO_TAB_L = 54.5
SERVO_HOLE_DX = 49.5         # mounting hole spacing along the case
SERVO_HOLE_DY = 10.0         # mounting hole spacing across the case
SERVO_AXIS_OFF = 10.25       # output axis -> centre of case (along X)
SERVO_HORN_FACE = 18.0       # tab seat -> face of the metal horn disc
HORN_D = 21.0                # 25T aluminium disc horn
HORN_T = 3.0
HORN_PCD = 14.0              # 4 x M3 on this pitch circle  (verify on your horn!)
SERVO_WALL = 5.0             # wall the tabs screw into

# --------------------------------------------------------------------------
# Drive motor: JGB37-520 12 V gear motor with hall encoder (37 mm gearbox)
# --------------------------------------------------------------------------
MOTOR_D = 37.0
MOTOR_GEARBOX_L = 24.0
MOTOR_CAN_D = 34.0
MOTOR_TOTAL_L = 76.0         # face -> end of encoder board
MOTOR_SHAFT_OFF = 7.0        # shaft is 7 mm off the gearbox centre
MOTOR_SHAFT_D = 6.0
MOTOR_SHAFT_L = 22.0         # face -> shaft tip (including boss)
MOTOR_BOSS_D = 12.0
MOTOR_BOSS_H = 6.0
MOTOR_PCD = 31.0             # 6 x M3 on this circle, centred on the gearbox
MOTOR_GEAR_RATIO = 90.0      # ~66 rpm no-load at 12 V  (verify on your motor)
MOTOR_ENC_PPR = 11.0         # pulses / motor rev / channel
MOTOR_NOLOAD_RPM = 66.0
MOTOR_STALL_NM = 2.4         # at the output shaft, 12 V (approx. datasheet)

# Hex wheel coupler, 6 mm bore, 12 mm across flats
COUPLER_AF = 12.0
COUPLER_L = 18.0

# --------------------------------------------------------------------------
# Chassis
# --------------------------------------------------------------------------
TRAY_L = 380.0
TRAY_W = 250.0
TRAY_H = 80.0
TRAY_WALL = 4.0
TRAY_FLOOR = 4.0
TRAY_CORNER_R = 18.0
TRAY_Z0 = 55.0               # ground clearance
MOTOR_PAD_T = 2.0            # extra wall thickness behind each motor face

DECK_T = 5.0
DECK_Z0 = TRAY_Z0 + TRAY_H   # 135
DECK_TOP = DECK_Z0 + DECK_T  # 140

WHEEL_D = 150.0
WHEEL_R = WHEEL_D / 2
WHEEL_W = 50.0
WHEEL_GAP = 8.0              # tray wall -> inboard face of wheel
AXLE_Z = WHEEL_R             # 75
WHEELBASE = 260.0
AXLE_X = WHEELBASE / 2       # 130
WHEEL_Y = TRAY_W / 2 + WHEEL_GAP + WHEEL_W / 2   # 158
TRACK = 2 * WHEEL_Y          # 316
RIM_D = 124.0                # tyre seat
RIM_FLANGE_D = 128.0
RIM_FLANGE_W = 2.5
TYRE_BASE_D = 141.0
TYRE_LUGS = 18

# deck / tray screw posts (x, y)
POSTS = [(177, 112), (177, -112), (-177, 112), (-177, -112),
         (0, 116), (0, -116), (181, 0)]

# electronics plate (inside tray)
EPLATE_X0, EPLATE_X1 = -38.0, 104.0
EPLATE_W = 210.0
EPLATE_T = 3.0
EPLATE_BOSS_H = 8.0
EPLATE_HOLES = [(-30, 95), (-30, -95), (96, 95), (96, -95)]
GRID_PITCH = 10.0

# battery bay (3S Li-ion / LiPo up to 140 x 50 x 30)
BATT_L, BATT_W, BATT_H = 138.0, 47.0, 26.0
BATT_X = -115.0              # centre of the pack
BATT_CLAMP_X = -54.0

# --------------------------------------------------------------------------
# Arm
# --------------------------------------------------------------------------
YAW_X = 90.0                 # yaw axis position on the deck
BALL_D = 6.35                # 1/4" bicycle bearing balls
RACE_R = 50.0                # ball pitch radius
RACE_N_BALLS = 24
RING_RI = 40.0
RING_RO = 70.0
RING_BOLT_R = 64.5
RING_H = 15.0
GROOVE_DEPTH = 3.0           # 90 deg V groove
TT_R = 58.0                  # turntable disc radius
TT_T = 6.0
TT_Z0 = DECK_TOP + SERVO_HORN_FACE          # 158: underside of turntable
TT_TOP = TT_Z0 + TT_T                        # 164
SHOULDER_X = 28.0            # shoulder axis ahead of the yaw axis
SHOULDER_H = 40.0            # shoulder axis above turntable top
SHOULDER_Z = TT_TOP + SHOULDER_H             # 204

# servo "box" cross-section shared by tower and forearm
BOX_Y1 = 8.0                 # +Y face (tab seat)
BOX_Y0 = -25.0               # -Y face
BOX_HALF_H = 15.2            # half height (servo width + walls)
PLATE_IN = BOX_Y1 + SERVO_HORN_FACE          # 26: inner face of fork plates
PLATE_T = 6.0
PLATE_OUT = PLATE_IN + PLATE_T               # 32
JOURNAL_D = 12.0
JOURNAL_CLEAR = 0.3

L1 = 200.0                   # shoulder -> elbow
L2 = 180.0                   # elbow -> wrist
ARM_PLATE_W = 44.0

WRIST_WEB_X = 30.0           # wrist axis -> inner face of the fork web
WRIST_ARM_W = 40.0
GRIP_X = 80.0                # wrist axis -> gripper servo axis
GRIP_GEAR_MODULE = 2.0
GRIP_GEAR_TEETH = 14
GRIP_DY = GRIP_GEAR_MODULE * GRIP_GEAR_TEETH / 2   # 14: finger pivots at +-14
FINGER_T = 8.0
PAD_T = 5.0                  # TPU pad thickness in front of the finger blade
PAD_H = 22.0                 # pad height (across the finger)
PAD_X0, PAD_X1 = 33.0, 69.0  # pad extent along the finger, from the pivot
PAD_FACE = GRIP_DY - 1.5     # pivot line -> pad face (pads meet 3 mm apart when closed)
PAD_DISH_R = 24.0            # spherical dish that cradles round fruit
PAD_DISH_DEPTH = 2.5
PALM_TOP_Z = -FINGER_T / 2 - SERVO_HORN_FACE       # -22
L3 = GRIP_X + 50.0           # wrist axis -> grasp centre (130)

# tool centre points in the wrist (tool) frame: x along gripper, y left, z up
TCP_GRIPPER = (L3, 0.0, 0.0)
TCP_PROBE = (22.0, -38.1, -118.0)    # tip of the soil-moisture blade
TCP_NOZZLE = (22.0, 44.0, -48.0)     # spray nozzle outlet (sprays along -Z)

# joint limits (degrees) in the kinematic convention used everywhere:
#   yaw      : 0 = arm forward, + = to the left (CCW from above)
#   shoulder : 0 = upper arm horizontal forward, + = up
#   elbow    : 0 = forearm in line with upper arm, + = up
#   wrist    : 0 = gripper in line with forearm, + = up
#   gripper  : 0 = closed, + = each finger opening angle
LIM_YAW = (-135.0, 135.0)
LIM_SHOULDER = (0.0, 180.0)
LIM_ELBOW = (-150.0, 120.0)     # 270 deg servo: lets the arm reach back over itself to the basket
LIM_WRIST = (-90.0, 90.0)
LIM_GRIP = (0.0, 50.0)

# --------------------------------------------------------------------------
# Deck accessories
# --------------------------------------------------------------------------
BASKET_X, BASKET_Y = -49.0, 0.0
BASKET_L, BASKET_W, BASKET_H = 118.0, 150.0, 55.0
MAST_X, MAST_Y, MAST_H = -160.0, -98.0, 190.0
BOTTLE_X, BOTTLE_Y, BOTTLE_D, BOTTLE_H = -150.0, 80.0, 66.0, 200.0
PUMP_X, PUMP_Y = -150.0, -14.0
CABLE_HOLE = (35.0, 100.0, 20.0)     # x, y, dia

# --------------------------------------------------------------------------
# Materials (for mass / filament estimates)
# --------------------------------------------------------------------------
RHO_PETG = 1.27e-3           # g / mm^3
RHO_TPU = 1.21e-3
FILL_FACTOR = 0.55           # 4 perimeters + 25 % gyroid ~ 55 % of solid mass


def race_ball_gap():
    """Vertical gap between race faces for the chosen ball and V-groove."""
    r = BALL_D / 2
    return 2 * (r * sqrt(2) - GROOVE_DEPTH)
