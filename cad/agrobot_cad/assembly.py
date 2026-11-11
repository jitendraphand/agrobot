"""Bill of parts, kinematic tree and placement of everything.

`LINKS` describes the kinematic tree (used verbatim by the MuJoCo generator,
Blender and the firmware geometry header).  `instances()` lists every solid,
printed or bought, with the link it is rigidly attached to.
"""
from dataclasses import dataclass, field
from math import cos, sin, radians

import cadquery as cq

from . import params as P
from . import parts as PT
from . import vitamins as V
from .helpers import frame, in_frame, polar

# --------------------------------------------------------------------------
# Kinematic tree.  pos = joint origin in the parent frame (mm), axis in the
# link's own frame, rot = fixed rotation of the link frame w.r.t. the parent
# (axis, degrees).  Zero pose: arm stretched straight forward, gripper closed.
# --------------------------------------------------------------------------
LINKS = {
    "chassis":   dict(parent=None, pos=(0, 0, 0), rot=None, axis=None, joint=None),
    "wheel_fl":  dict(parent="chassis", pos=(P.AXLE_X, P.WHEEL_Y, P.AXLE_Z), rot=None, axis=(0, 1, 0), joint="wheel_fl"),
    "wheel_rl":  dict(parent="chassis", pos=(-P.AXLE_X, P.WHEEL_Y, P.AXLE_Z), rot=None, axis=(0, 1, 0), joint="wheel_rl"),
    "wheel_fr":  dict(parent="chassis", pos=(P.AXLE_X, -P.WHEEL_Y, P.AXLE_Z), rot=((0, 0, 1), 180.0), axis=(0, -1, 0), joint="wheel_fr"),
    "wheel_rr":  dict(parent="chassis", pos=(-P.AXLE_X, -P.WHEEL_Y, P.AXLE_Z), rot=((0, 0, 1), 180.0), axis=(0, -1, 0), joint="wheel_rr"),
    "turntable": dict(parent="chassis", pos=(P.YAW_X, 0, P.TT_Z0), rot=None, axis=(0, 0, 1), joint="yaw", limits=P.LIM_YAW),
    "upper_arm": dict(parent="turntable", pos=(P.SHOULDER_X, 0, P.TT_T + P.SHOULDER_H), rot=None, axis=(0, -1, 0), joint="shoulder", limits=P.LIM_SHOULDER),
    "forearm":   dict(parent="upper_arm", pos=(P.L1, 0, 0), rot=None, axis=(0, -1, 0), joint="elbow", limits=P.LIM_ELBOW),
    "tool":      dict(parent="forearm", pos=(P.L2, 0, 0), rot=None, axis=(0, -1, 0), joint="wrist", limits=P.LIM_WRIST),
    "finger_drive": dict(parent="tool", pos=(P.GRIP_X, P.GRIP_DY, 0), rot=None, axis=(0, 0, 1), joint="grip", limits=P.LIM_GRIP),
    "finger_idle":  dict(parent="tool", pos=(P.GRIP_X, -P.GRIP_DY, 0), rot=None, axis=(0, 0, -1), joint="grip_mimic", limits=P.LIM_GRIP),
}

# colours (linear-ish RGB 0..1)
GREEN = (0.16, 0.42, 0.20)
CHARCOAL = (0.13, 0.14, 0.15)
YELLOW = (0.93, 0.70, 0.08)
BLACK = (0.03, 0.03, 0.03)
ORANGE = (0.92, 0.38, 0.06)
GREY = (0.62, 0.64, 0.66)
STEEL = (0.72, 0.73, 0.75)
BLUE = (0.10, 0.25, 0.65)
PCB = (0.05, 0.30, 0.16)
WATER = (0.55, 0.78, 0.92)
WHITE = (0.90, 0.90, 0.88)
RED = (0.75, 0.08, 0.06)

RX = lambda a: ((1, 0, 0), a)
RY = lambda a: ((0, 1, 0), a)
RZ = lambda a: ((0, 0, 1), a)


@dataclass
class PartDef:
    """One unique solid."""
    name: str
    shape: object                 # cq.Workplane in its modelling frame
    color: tuple
    printed: bool = True
    material: str = "PETG"
    print_rot: list = field(default_factory=list)   # rotations to reach print orientation
    mass_g: float = None          # for vitamins; printed parts are computed
    note: str = ""
    placements: list = field(default_factory=list)  # [(link, cq.Location)]


def _loc(pos=(0, 0, 0), rot=None):
    if rot is None:
        return cq.Location(cq.Vector(*pos))
    return cq.Location(cq.Vector(*pos), cq.Vector(*rot[0]), rot[1])


I = _loc()


def part_defs():
    """Build every solid.  Returns an ordered list of PartDef."""
    d = []

    def add(name, shape, color, placements, **kw):
        d.append(PartDef(name=name, shape=shape, color=color, placements=placements, **kw))

    wheels = ["wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"]

    # ---------------- printed ----------------
    add("chassis_tray", PT.chassis_tray(), GREEN, [("chassis", I)],
        note="Main tub. 4 perimeters, 25 % infill, no supports.")
    add("top_deck", PT.top_deck(), CHARCOAL, [("chassis", I)],
        note="Lid / arm deck. Print flat, 5 perimeters.")
    add("electronics_plate", PT.electronics_plate(), GREY, [("chassis", I)],
        note="10 mm M3 grid for standoffs / zip ties.")
    add("battery_clamp", PT.battery_clamp(), ORANGE, [("chassis", I)], print_rot=[RX(180)],
        note="Printed upside down (open side up).")
    add("front_bumper", PT.front_bumper(), ORANGE, [("chassis", I)], print_rot=[RY(90)],
        note="Front face on the bed. Houses the HC-SR04.")
    add("wheel_rim", PT.wheel_rim(), YELLOW, [(w, I) for w in wheels], print_rot=[RX(-90)],
        note="Outboard face on the bed. 5 perimeters, 40 % infill.")
    add("wheel_tyre", PT.wheel_tyre(), BLACK, [(w, I) for w in wheels], print_rot=[RX(-90)],
        material="TPU 95A", note="TPU 95A, 3 perimeters, 15 % infill. PETG works on hard floors.")
    add("race_ring", PT.race_ring(), CHARCOAL, [("chassis", I)],
        note="Lower ball race. 0.12-0.16 mm layers for a smooth groove.")
    add("turntable_clip", PT.turntable_clip(), ORANGE,
        [("chassis", _loc((P.YAW_X, 0, 0), RZ(a)) * _loc((-P.YAW_X, 0, 0))) for a in (30, 150, 270)],
        print_rot=[RX(180)], note="Printed lip-down. 100 % infill.")
    add("turntable", PT.turntable(), ORANGE, [("turntable", I)],
        note="Disc on the bed. Support only under the journal boss.")
    add("upper_arm_drive_plate", PT.upper_arm_drive_plate(), ORANGE, [("upper_arm", I)], print_rot=[RX(-90)],
        note="Flat. 6 perimeters (effectively solid).")
    add("upper_arm_idler_plate", PT.upper_arm_idler_plate(), ORANGE, [("upper_arm", I)], print_rot=[RX(90)],
        note="Flat. 6 perimeters (effectively solid).")
    add("upper_arm_brace", PT.upper_arm_brace(), CHARCOAL, [("upper_arm", I)], print_rot=[RY(-90)],
        note="Duct axis vertical.")
    add("forearm", PT.forearm(), ORANGE, [("forearm", I)],
        note="As modelled (bottom wall on bed). Support only under the two journal bosses.")
    add("wrist_fork", PT.wrist_fork(), CHARCOAL, [("tool", I)], print_rot=[RY(90)],
        note="Web on the bed, arms pointing up.")
    add("gripper_palm", PT.gripper_palm(), CHARCOAL, [("tool", I)],
        note="Plate on the bed.")
    add("finger_drive", PT.finger_drive(), ORANGE, [("finger_drive", I)],
        note="Flat, 100 % infill, 0.12 mm layers for clean gear teeth.")
    add("finger_idle", PT.finger_idle(), ORANGE, [("finger_idle", I)],
        note="Flat, 100 % infill, 0.12 mm layers for clean gear teeth.")
    add("finger_pad", PT.finger_pad(), BLACK, [("finger_drive", I), ("finger_idle", _loc((0, 0, 0), RX(180)))],
        print_rot=[RY(-90)], material="TPU 95A", note="TPU 95A, standing on end, 100 % infill. Press on, add a drop of CA glue.")
    add("probe_clip", PT.probe_clip(), GREY, [("tool", I)], print_rot=[RX(-90)],
        note="Back face on the bed.")
    add("nozzle_clip", PT.nozzle_clip(), GREY, [("tool", I)], print_rot=[RX(90)],
        note="Back face on the bed.")
    add("harvest_basket", PT.harvest_basket(), GREEN, [("chassis", I)],
        note="Vase-like, 3 perimeters.")
    add("camera_mast", PT.camera_mast(), CHARCOAL, [("chassis", I)], print_rot=[RY(-90)],
        note="Lying on its back for strength.")
    cam_loc = _loc((P.MAST_X + PT.CAM_HINGE_DX, P.MAST_Y, P.DECK_TOP + P.MAST_H - 10), RY(PT.CAM_TILT))
    add("camera_holder", PT.camera_holder(), ORANGE, [("chassis", cam_loc)], print_rot=[RY(-90)],
        note="Back plate on the bed.")
    add("bottle_holder", PT.bottle_holder(), GREEN, [("chassis", I)],
        note="Upright.")
    add("pump_saddle", PT.pump_saddle(), GREY, [("chassis", I)],
        note="Flat.")

    # ---------------- vitamins ----------------
    def servo_at(link, origin, xdir, zdir):
        return (link, cq.Location(frame(origin, xdir, zdir)))

    add("servo_std", V.servo(), BLACK, [
        servo_at("chassis", (P.YAW_X, 0, P.DECK_TOP), (-1, 0, 0), (0, 0, 1)),                        # yaw   DS3218
        servo_at("turntable", (P.SHOULDER_X, P.BOX_Y1, P.TT_T + P.SHOULDER_H), (0, 0, -1), (0, 1, 0)),  # shoulder DS3235
        servo_at("forearm", (0, P.BOX_Y1, 0), (1, 0, 0), (0, 1, 0)),                                  # elbow DS3218
        servo_at("forearm", (P.L2, P.BOX_Y1, 0), (-1, 0, 0), (0, 1, 0)),                              # wrist MG996R
        servo_at("tool", (P.GRIP_X, P.GRIP_DY, P.PALM_TOP_Z), (-1, 0, 0), (0, 0, 1)),                 # gripper MG996R
    ], printed=False, mass_g=60.0)

    add("gearmotor", V.gearmotor(), STEEL, [
        ("chassis", cq.Location(frame((sx * P.AXLE_X, sy * (P.TRAY_W / 2 - P.TRAY_WALL - P.MOTOR_PAD_T), P.AXLE_Z),
                                      (0, 0, 1), (0, sy, 0)))) for sx in (1, -1) for sy in (1, -1)
    ], printed=False, mass_g=215.0)
    add("hex_coupler", V.hex_coupler(), STEEL, [(w, I) for w in wheels], printed=False, mass_g=12.0)
    add("battery_3s", V.battery(), BLUE, [("chassis", I)], printed=False, mass_g=420.0)
    add("bottle_500ml", V.bottle(), WATER, [("chassis", I)], printed=False, mass_g=520.0)
    add("bottle_cap", V.bottle_cap(), WHITE, [("chassis", I)], printed=False, mass_g=3.0)
    add("pump_r385", V.pump(), CHARCOAL, [("chassis", I)], printed=False, mass_g=95.0)
    add("hcsr04", V.hcsr04(), BLUE, [("chassis", I)], printed=False, mass_g=9.0)
    add("esp32cam", V.esp32cam(), PCB, [("chassis", cam_loc)], printed=False, mass_g=10.0)
    add("soil_probe", V.soil_probe(), BLACK, [("tool", I)], printed=False, mass_g=8.0)
    add("spray_nozzle", V.nozzle(), YELLOW, [("tool", I)], printed=False, mass_g=6.0)
    add("race_balls", V.balls(), STEEL, [("chassis", I)], printed=False, mass_g=25.0)
    board_mass = dict(esp32_devkit=10, pca9685=9, bts7960_left=66, bts7960_right=66, buck_5v_lm2596=12,
                      buck_servo_xl4016=45, mosfet_pump=6, mpu6050=2, fuse_holder=15)
    board_col = dict(bts7960_left=RED, bts7960_right=RED, buck_5v_lm2596=BLUE, buck_servo_xl4016=BLUE,
                     fuse_holder=BLACK)
    for name, shp in V.boards().items():
        add(name, shp, board_col.get(name, PCB), [("chassis", I)], printed=False, mass_g=board_mass[name])
    return d


def default_pose():
    """A presentable 'ready' pose used for the assembly STEP and hero renders (deg)."""
    return dict(yaw=0.0, shoulder=115.0, elbow=-105.0, wrist=-10.0, grip=20.0, grip_mimic=20.0)


def link_world_locations(pose=None):
    """Forward kinematics: cq.Location of every link frame in the robot frame."""
    pose = pose or {}
    out = {}

    def solve(name):
        if name in out:
            return out[name]
        L = LINKS[name]
        if L["parent"] is None:
            out[name] = cq.Location()
            return out[name]
        loc = solve(L["parent"]) * _loc(L["pos"], L["rot"])
        q = pose.get(L["joint"], 0.0)
        if L["joint"] in ("grip_mimic",):
            q = pose.get("grip", q)
        if q:
            loc = loc * cq.Location(cq.Vector(0, 0, 0), cq.Vector(*L["axis"]), q)
        out[name] = loc
        return loc

    for n in LINKS:
        solve(n)
    return out
