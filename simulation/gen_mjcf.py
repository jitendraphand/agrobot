#!/usr/bin/env python3
"""Generate MuJoCo models of AgroBot straight from the CAD manifest.

    python simulation/gen_mjcf.py          # writes simulation/model/*.xml

Visual geometry = the exported STL files (exact CAD).
Collision geometry = a handful of primitives (fast and stable).
Mass / inertia = per-part masses from the manifest applied to the meshes.
"""
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "cad"))
from agrobot_cad import params as P   # noqa: E402

EXPORT = os.path.join(ROOT, "cad", "export")
MODEL_DIR = os.path.join(HERE, "model")
MM = 0.001

# drive motor model (DC motor, linear torque-speed curve)
MOTOR_STALL = P.MOTOR_STALL_NM
MOTOR_W0 = P.MOTOR_NOLOAD_RPM * 2 * math.pi / 60.0

# servo models: stall torque [N m] (datasheet at ~6.8 V), position gain, damping
SERVOS = {
    "yaw":      dict(tau=2.0, kp=40.0, kv=2.0),     # DS3218
    "shoulder": dict(tau=3.3, kp=90.0, kv=4.0),     # DS3235
    "elbow":    dict(tau=2.0, kp=50.0, kv=2.0),     # DS3218
    "wrist":    dict(tau=1.0, kp=20.0, kv=0.6),     # MG996R
    "grip":     dict(tau=1.0, kp=8.0, kv=0.2),      # MG996R
}


def f(v):
    return " ".join(f"{x:.6g}" for x in v)


def load_manifest():
    with open(os.path.join(EXPORT, "manifest.json")) as fh:
        return json.load(fh)


def collision_geoms(link):
    """Primitive collision shapes per link (link frame, metres)."""
    g = []
    box = lambda n, c, h, **k: g.append(dict(type="box", name=n, pos=c, size=h, **k))
    if link == "chassis":
        zc = (P.TRAY_Z0 + P.DECK_TOP) / 2 * MM
        box("c_tray", (0, 0, zc), (P.TRAY_L / 2 * MM, P.TRAY_W / 2 * MM, (P.DECK_TOP - P.TRAY_Z0) / 2 * MM))
        box("c_bumper", ((P.TRAY_L / 2 + 7) * MM, 0, (P.TRAY_Z0 + 40) * MM), (0.007, 0.112, 0.02))
        # basket: floor is the deck, four walls
        bx, by, bl, bw, bh = P.BASKET_X * MM, P.BASKET_Y * MM, P.BASKET_L * MM, P.BASKET_W * MM, P.BASKET_H * MM
        z = P.DECK_TOP * MM + bh / 2
        t = 0.0024
        box("c_bask_f", (bx + bl / 2 - t / 2, by, z), (t / 2, bw / 2, bh / 2))
        box("c_bask_r", (bx - bl / 2 + t / 2, by, z), (t / 2, bw / 2, bh / 2))
        box("c_bask_l", (bx, by + bw / 2 - t / 2, z), (bl / 2, t / 2, bh / 2))
        box("c_bask_rt", (bx, by - bw / 2 + t / 2, z), (bl / 2, t / 2, bh / 2))
        box("c_bask_floor", (bx, by, P.DECK_TOP * MM + 0.0015), (bl / 2, bw / 2, 0.0015))
        g.append(dict(type="cylinder", name="c_bottle", pos=(P.BOTTLE_X * MM, P.BOTTLE_Y * MM, (P.DECK_TOP + 100) * MM),
                      size=(P.BOTTLE_D / 2 * MM + 0.003, 0.1)))
        box("c_mast", ((P.MAST_X - 14) * MM, P.MAST_Y * MM, (P.DECK_TOP + P.MAST_H / 2) * MM), (0.008, 0.01, P.MAST_H / 2 * MM))
        g.append(dict(type="cylinder", name="c_ring", pos=(P.YAW_X * MM, 0, (P.DECK_TOP + P.RING_H / 2) * MM),
                      size=(P.RING_RO * MM, P.RING_H / 2 * MM)))
    elif link.startswith("wheel"):
        g.append(dict(type="cylinder", name=f"c_{link}", pos=(0, 0, 0), size=(P.WHEEL_R * MM, (P.WHEEL_W - 5) / 2 * MM),
                      quat=(0.7071068, 0.7071068, 0, 0), cls="wheel"))
    elif link == "turntable":
        zs = (P.TT_T + P.SHOULDER_H) * MM
        box("c_tower", (P.SHOULDER_X * MM, (P.BOX_Y0 + P.BOX_Y1) / 2 * MM, (P.TT_T + 30) * MM),
            (P.BOX_HALF_H * MM, (P.BOX_Y1 - P.BOX_Y0) / 2 * MM, 0.03))
        g.append(dict(type="cylinder", name="c_disc", pos=(0, 0, P.TT_T / 2 * MM), size=(P.TT_R * MM, P.TT_T / 2 * MM)))
    elif link == "upper_arm":
        for s, n in ((1, "a"), (-1, "b")):
            yc = s * (P.PLATE_IN + P.PLATE_T / 2) * MM
            box(f"c_uarm_{n}", (P.L1 / 2 * MM, yc, 0), (P.L1 / 2 * MM, P.PLATE_T / 2 * MM, P.ARM_PLATE_W / 2 * MM))
        box("c_uarm_brace", (P.L1 / 2 * MM, 0, 0), (0.02, P.PLATE_IN * MM, 0.013))
    elif link == "forearm":
        box("c_forearm", (P.L2 / 2 * MM, (P.BOX_Y0 + P.BOX_Y1) / 2 * MM, 0),
            ((P.L2 / 2 + 21) * MM, (P.BOX_Y1 - P.BOX_Y0) / 2 * MM, P.BOX_HALF_H * MM))
    elif link == "tool":
        box("c_fork_web", ((P.WRIST_WEB_X + 5.5) * MM, 0, 0), (0.0055, P.PLATE_OUT * MM, P.WRIST_ARM_W / 2 * MM))
        box("c_palm", (0.070, 0, (P.PALM_TOP_Z - 2.5) * MM), (0.034, 0.030, 0.0025))
        box("c_grip_servo", ((P.GRIP_X - P.SERVO_AXIS_OFF) * MM, P.GRIP_DY * MM, (P.PALM_TOP_Z - 9) * MM),
            (0.0205, 0.010, 0.019))
    elif link in ("finger_drive", "finger_idle"):
        s = 1 if link == "finger_drive" else -1
        # TPU pad: a flat face plus four bumps standing in for the rim of the
        # spherical dish (MuJoCo has no concave primitives)
        yf = -s * P.PAD_FACE * MM
        xc = (P.PAD_X0 + P.PAD_X1) / 2 * MM
        box(f"c_{link}", (xc, yf + s * P.PAD_T / 2 * MM, 0), ((P.PAD_X1 - P.PAD_X0) / 2 * MM, P.PAD_T / 2 * MM, P.PAD_H / 2 * MM), cls="pad")
        k = 0
        for (dx, dz) in ((0.009, 0), (-0.009, 0), (0, 0.009), (0, -0.009)):
            g.append(dict(type="sphere", name=f"c_{link}_rim{k}", pos=(xc + dx, yf + s * 0.0005, dz), size=(0.0025,), cls="pad"))
            k += 1
        box(f"c_{link}_root", (0.014, s * 0.002, 0), (0.016, 0.008, P.FINGER_T / 2 * MM), cls="pad")
    return g


def robot_xml(man, pose=None, base_pos=(0, 0, 0.002), free=True):
    """<body> tree for the robot plus the assets it needs."""
    links = man["links"]
    children = {}
    for n, L in links.items():
        children.setdefault(L["parent"], []).append(n)
    by_link = {}
    for prt in man["parts"]:
        for i, pl in enumerate(prt["placements"]):
            by_link.setdefault(pl["link"], []).append((prt, i, pl))

    assets, mats = [], {}
    for prt in man["parts"]:
        assets.append(f'<mesh name="{prt["name"]}" file="{prt["mesh"]}" scale="0.001 0.001 0.001"/>')
        c = prt["color"]
        key = "m_" + prt["name"]
        alpha = 0.55 if prt["name"] == "bottle_500ml" else 1.0
        metal = prt["name"] in ("gearmotor", "hex_coupler", "race_balls")
        spec = 0.6 if metal else (0.05 if "tyre" in prt["name"] else 0.25)
        shin = 0.7 if metal else 0.3
        assets.append(f'<material name="{key}" rgba="{c[0]:.3f} {c[1]:.3f} {c[2]:.3f} {alpha}" specular="{spec}" shininess="{shin}"/>')

    def body(name, indent):
        L = links[name]
        sp = " " * indent
        out = []
        if L["parent"] is None:
            out.append(f'{sp}<body name="{name}" pos="{f(base_pos)}">')
            if free:
                out.append(f'{sp} <freejoint name="root"/>')
            out.append(f'{sp} <site name="imu" pos="0.02 0 {(P.TRAY_Z0 + 30) * MM:.4f}" size="0.005"/>')
            cam_x, cam_z = (P.MAST_X + 20) * MM, (P.DECK_TOP + P.MAST_H - 8) * MM
            # onboard ESP32-CAM: looks forward, tilted down
            tilt = math.radians(28.0)
            out.append(f'{sp} <camera name="onboard" pos="{cam_x:.4f} {P.MAST_Y * MM:.4f} {cam_z:.4f}" '
                       f'xyaxes="0 -1 0 {math.sin(tilt):.4f} 0 {math.cos(tilt):.4f}" fovy="62"/>')
        else:
            pos = [v * MM for v in L["pos"]]
            out.append(f'{sp}<body name="{name}" pos="{f(pos)}" quat="{f(L["quat"])}">')
            j = L["joint"]
            ax = L["axis"]
            if j.startswith("wheel"):
                out.append(f'{sp} <joint name="{j}" class="wheel" axis="{f(ax)}"/>')
            else:
                lo, hi = [math.radians(v) for v in L["limits"]]
                cls = "finger" if j.startswith("grip") else "arm"
                out.append(f'{sp} <joint name="{j}" class="{cls}" axis="{f(ax)}" range="{lo:.5f} {hi:.5f}"/>')
            if name == "tool":
                for tn, tv in man["tcp"].items():
                    out.append(f'{sp} <site name="tcp_{tn}" pos="{f([v * MM for v in tv])}" size="0.004" rgba="1 0 0 0"/>')
        for (prt, i, pl) in by_link.get(name, []):
            pos = [v * MM for v in pl["pos"]]
            m = max(prt["mass_g"], 0.5) / 1000.0
            if prt["name"] == "bottle_500ml":
                m = 0.35      # two-thirds full
            out.append(f'{sp} <geom class="visual" mesh="{prt["name"]}" material="m_{prt["name"]}" '
                       f'pos="{f(pos)}" quat="{f(pl["quat"])}" mass="{m:.5f}"/>')
        for cg in collision_geoms(name):
            cls = cg.get("cls", "collision")
            q = f' quat="{f(cg["quat"])}"' if "quat" in cg else ""
            out.append(f'{sp} <geom class="{cls}" name="{cg["name"]}" type="{cg["type"]}" pos="{f(cg["pos"])}" size="{f(cg["size"])}"{q}/>')
        for ch in children.get(name, []):
            out.extend(body(ch, indent + 1))
        out.append(f'{sp}</body>')
        return out

    root = [n for n, L in links.items() if L["parent"] is None][0]
    return "\n".join(assets), "\n".join(body(root, 2))


ACTUATORS = """
  <actuator>
    <!-- DC gear motors: ctrl = PWM duty (-1..1); torque = stall*duty - (stall/w0)*omega -->
    <general name="m_fl" joint="wheel_fl" class="motor"/>
    <general name="m_rl" joint="wheel_rl" class="motor"/>
    <general name="m_fr" joint="wheel_fr" class="motor"/>
    <general name="m_rr" joint="wheel_rr" class="motor"/>
    <!-- hobby servos: ctrl = commanded angle (rad) -->
{servos}
  </actuator>
  <equality>
    <joint joint1="grip_mimic" joint2="grip" polycoef="0 1 0 0 0" solref="0.004 1"/>
  </equality>
  <sensor>
    <jointpos name="enc_fl" joint="wheel_fl"/>
    <jointpos name="enc_rl" joint="wheel_rl"/>
    <jointpos name="enc_fr" joint="wheel_fr"/>
    <jointpos name="enc_rr" joint="wheel_rr"/>
    <gyro name="gyro" site="imu"/>
    <accelerometer name="accel" site="imu"/>
    <framepos name="tcp_gripper_pos" objtype="site" objname="tcp_gripper"/>
    <framepos name="tcp_probe_pos" objtype="site" objname="tcp_probe"/>
    <framepos name="tcp_nozzle_pos" objtype="site" objname="tcp_nozzle"/>
    <actuatorfrc name="tau_shoulder" actuator="s_shoulder"/>
    <actuatorfrc name="tau_elbow" actuator="s_elbow"/>
    <actuatorfrc name="tau_wrist" actuator="s_wrist"/>
    <actuatorfrc name="tau_yaw" actuator="s_yaw"/>
  </sensor>
"""


def actuators_xml():
    lines = []
    lim = dict(yaw=P.LIM_YAW, shoulder=P.LIM_SHOULDER, elbow=P.LIM_ELBOW, wrist=P.LIM_WRIST, grip=P.LIM_GRIP)
    for j, s in SERVOS.items():
        lo, hi = [math.radians(v) for v in lim[j]]
        lines.append(f'    <position name="s_{j}" joint="{j}" kp="{s["kp"]}" kv="{s["kv"]}" '
                     f'ctrlrange="{lo:.5f} {hi:.5f}" forcerange="{-s["tau"]} {s["tau"]}"/>')
    return ACTUATORS.format(servos="\n".join(lines))


HEADER = """<mujoco model="{name}">
  <!-- GENERATED by simulation/gen_mjcf.py from cad/export/manifest.json - do not edit -->
  <compiler angle="radian" meshdir="../../cad/export" texturedir="../assets" autolimits="true"/>
  <option timestep="0.002" integrator="implicitfast" cone="elliptic" impratio="10" noslip_iterations="4"/>
  <size memory="64M"/>
  <visual>
    <global offwidth="1920" offheight="1080" fovy="42"/>
    <quality shadowsize="4096" offsamples="4"/>
    <headlight ambient="0.38 0.38 0.40" diffuse="0.25 0.25 0.25" specular="0 0 0"/>
    <map znear="0.01" zfar="60" shadowclip="2.5"/>
    <rgba haze="0.78 0.86 0.93 1"/>
  </visual>
  <default>
    <default class="visual">
      <geom type="mesh" contype="0" conaffinity="0" group="2"/>
    </default>
    <default class="collision">
      <geom contype="1" conaffinity="1" group="3" density="0" rgba="1 0 0 0.3" friction="0.8 0.01 0.001"/>
    </default>
    <default class="wheel">
      <geom contype="1" conaffinity="1" group="3" density="0" rgba="0 0 1 0.3" friction="1.1 0.01 0.002" condim="4"/>
      <joint type="hinge" damping="0.02" frictionloss="0.04" armature="0.008"/>
    </default>
    <default class="pad">
      <geom contype="1" conaffinity="1" group="3" density="0" rgba="0 1 0 0.3" friction="1.6 0.05 0.002" condim="4"
            solref="0.006 1" solimp="0.95 0.99 0.001"/>
    </default>
    <default class="arm">
      <joint type="hinge" damping="0.35" frictionloss="0.05" armature="0.012"/>
    </default>
    <default class="finger">
      <joint type="hinge" damping="0.05" frictionloss="0.01" armature="0.003"/>
    </default>
    <default class="motor">
      <general ctrlrange="-1 1" gaintype="fixed" gainprm="{stall}" biastype="affine" biasprm="0 0 {bias}"/>
    </default>
  </default>
"""


def soil_texture(path, size=1024, seed=3):
    """Procedural tilled-soil texture (so no binary art assets are needed)."""
    from PIL import Image
    rng = np.random.default_rng(seed)
    img = np.zeros((size, size), dtype=np.float32)
    for octave, amp in ((4, 0.5), (8, 0.25), (16, 0.14), (64, 0.08), (256, 0.06)):
        n = rng.random((octave + 1, octave + 1)).astype(np.float32)
        up = np.asarray(Image.fromarray((n * 255).astype(np.uint8)).resize((size, size), Image.BICUBIC), dtype=np.float32) / 255
        img += amp * up
    img = (img - img.min()) / (img.max() - img.min())
    # furrows running along X
    y = np.linspace(0, 1, size)[:, None]
    furrow = 0.5 + 0.5 * np.cos(2 * np.pi * y * 8)
    v = 0.72 * img + 0.28 * furrow
    base = np.array([0.36, 0.24, 0.15])
    dark = np.array([0.17, 0.11, 0.07])
    rgb = dark[None, None, :] + (base - dark)[None, None, :] * v[:, :, None]
    Image.fromarray((np.clip(rgb, 0, 1) * 255).astype(np.uint8)).save(path)


def plant_xml(name, x, y, h=0.34, seed=0, fruit_at=None):
    """A stylised tomato-like plant: stem, leaves, optional fruit perch."""
    rng = np.random.default_rng(seed)
    out = [f'<body name="{name}" pos="{x:.4f} {y:.4f} 0">',
           f' <geom type="capsule" fromto="0 0 0 0 0 {h:.3f}" size="0.006" material="stem" contype="0" conaffinity="0"/>',
           f' <geom type="cylinder" pos="0 0 0.004" size="0.05 0.004" material="mulch" contype="0" conaffinity="0"/>']
    n_leaf = 9
    for i in range(n_leaf):
        z = h * (0.28 + 0.7 * i / (n_leaf - 1))
        a = i * 2.39996 + rng.uniform(-0.3, 0.3)          # golden angle phyllotaxis
        r = 0.085 * (1.0 - 0.45 * i / n_leaf)
        # keep foliage out of the corridor the arm uses (towards -Y, the robot side)
        if fruit_at is not None and math.sin(a) < -0.2 and abs(z - fruit_at[2]) < 0.12:
            a = -a
        dx, dy = math.cos(a), math.sin(a)
        droop = rng.uniform(0.15, 0.4)
        ex, ey, ez = dx * r, dy * r, -droop * r
        cx, cy, cz = ex * 0.55, ey * 0.55, z + ez * 0.55
        yaw = math.atan2(dy, dx)
        pitch = math.atan2(-ez, r)
        q = quat_mul(axis_quat((0, 0, 1), yaw), axis_quat((0, 1, 0), pitch))
        out.append(f' <geom type="ellipsoid" pos="{cx:.4f} {cy:.4f} {cz:.4f}" quat="{f(q)}" '
                   f'size="{r * 0.55:.4f} {r * 0.26:.4f} 0.004" material="leaf{i % 2}" contype="0" conaffinity="0"/>')
        out.append(f' <geom type="capsule" fromto="0 0 {z:.4f} {ex * 0.3:.4f} {ey * 0.3:.4f} {z + ez * 0.3:.4f}" size="0.003" '
                   f'material="stem" contype="0" conaffinity="0"/>')
    if fruit_at is not None:
        fx, fy, fz = fruit_at
        # a truss (side branch) ending in a small cup that the fruit rests in
        out.append(f' <geom type="capsule" fromto="0 0 {fz - 0.05:.4f} {fx:.4f} {fy:.4f} {fz - 0.034:.4f}" size="0.004" '
                   f'material="stem" contype="0" conaffinity="0"/>')
        out.append(f' <geom name="{name}_cup" type="cylinder" pos="{fx:.4f} {fy:.4f} {fz - 0.030:.4f}" size="0.011 0.004" '
                   f'material="stem" friction="1 0.05 0.01" condim="4"/>')
        for k in range(3):
            a = k * 2.094
            out.append(f' <geom type="sphere" pos="{fx + 0.012 * math.cos(a):.4f} {fy + 0.012 * math.sin(a):.4f} {fz - 0.024:.4f}" '
                       f'size="0.005" material="stem" friction="1 0.05 0.01"/>')
    out.append('</body>')
    return "\n".join(out)


def axis_quat(axis, ang):
    s = math.sin(ang / 2)
    return (math.cos(ang / 2), axis[0] * s, axis[1] * s, axis[2] * s)


def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)


SCENE_ASSETS = """
    <texture type="skybox" builtin="gradient" rgb1="0.50 0.69 0.90" rgb2="0.93 0.95 0.96" width="512" height="3072"/>
    <texture name="soil" type="2d" file="soil.png"/>
    <material name="soil" texture="soil" texrepeat="{rep} {rep}" texuniform="false" specular="0.02" shininess="0.02" reflectance="0"/>
    <material name="stem" rgba="0.24 0.42 0.12 1" specular="0.1"/>
    <material name="leaf0" rgba="0.13 0.47 0.14 1" specular="0.15"/>
    <material name="leaf1" rgba="0.20 0.55 0.16 1" specular="0.15"/>
    <material name="mulch" rgba="0.20 0.13 0.08 1" specular="0"/>
    <material name="fruit" rgba="0.85 0.10 0.06 1" specular="0.6" shininess="0.8"/>
    <material name="fruit_green" rgba="0.45 0.62 0.15 1" specular="0.5" shininess="0.6"/>
    <material name="marker" rgba="0.95 0.95 0.92 1"/>
    <material name="wood" rgba="0.55 0.40 0.24 1" specular="0.05"/>
"""

# crop row used by the mission demo: plants on the robot's left, 0.30 m off its centre line
ROW_Y = 0.335
PLANT_XS = [0.70, 1.30, 1.90]
FRUIT_PLANT = 2                      # index of the plant carrying the ripe fruit
FRUIT_R = 0.021
FRUIT_OFFSET = (0.100, -0.030, 0.290)  # fruit centre relative to the plant base (towards the robot)


def row_scene(man):
    assets, body = robot_xml(man)
    parts = []
    parts.append('<light name="sun" directional="true" pos="-2 -3 6" dir="0.35 0.45 -1" diffuse="0.95 0.92 0.85" '
                 'specular="0.3 0.3 0.3" castshadow="true"/>')
    parts.append('<geom name="ground" type="plane" size="12 12 0.1" material="soil" friction="1.0 0.02 0.002" condim="4"/>')
    seed = 10
    # treated row (left of the robot) and two background rows
    for i, x in enumerate(PLANT_XS):
        fruit = FRUIT_OFFSET if i == FRUIT_PLANT else None
        parts.append(plant_xml(f"plant{i}", x, ROW_Y, h=0.36, seed=seed + i, fruit_at=fruit))
    k = 0
    for row_y in (ROW_Y, ROW_Y + 0.75, -0.62, -0.62 - 0.75):
        for x in np.arange(-1.1, 4.4, 0.6):
            if row_y == ROW_Y and any(abs(x - px) < 0.05 for px in PLANT_XS):
                continue
            k += 1
            parts.append(plant_xml(f"bg{k}", float(x) + 0.1, row_y, h=0.30 + 0.08 * ((k * 7) % 5) / 5, seed=100 + k))
    # ripe fruit: a free body resting in the cup
    fx = PLANT_XS[FRUIT_PLANT] + FRUIT_OFFSET[0]
    fy = ROW_Y + FRUIT_OFFSET[1]
    fz = FRUIT_OFFSET[2] + 0.001
    parts.append(f'<body name="fruit" pos="{fx:.4f} {fy:.4f} {fz:.4f}"><freejoint name="fruit"/>'
                 f'<geom name="fruit" type="sphere" size="{FRUIT_R}" mass="0.040" material="fruit" condim="6" '
                 f'friction="1.4 0.02 0.002" solref="0.006 1" solimp="0.95 0.99 0.001"/>'
                 f'<geom type="cylinder" pos="0 0 {FRUIT_R:.4f}" size="0.006 0.0015" material="stem" contype="0" conaffinity="0" mass="0.0005"/></body>')
    # row markers
    for i, x in enumerate(PLANT_XS):
        parts.append(f'<geom type="box" pos="{x:.3f} {ROW_Y + 0.14:.3f} 0.06" size="0.002 0.02 0.06" material="wood" contype="0" conaffinity="0"/>')
        parts.append(f'<geom type="box" pos="{x:.3f} {ROW_Y + 0.14:.3f} 0.135" size="0.003 0.035 0.022" material="marker" contype="0" conaffinity="0"/>')
    world = "\n    ".join(parts)
    return HEADER.format(name="agrobot_row", stall=MOTOR_STALL, bias=-MOTOR_STALL / MOTOR_W0) + f"""
  <asset>
{SCENE_ASSETS.format(rep=10)}
{assets}
  </asset>
  <worldbody>
    {world}
{body}
  </worldbody>
{actuators_xml()}
</mujoco>
"""


TERRAIN_X0 = 2.0      # world x of the centre of the height field (it spans x = -1 .. 5 m)


def terrain_heights(n=257, size=6.0, seed=5):
    """Rolling field with ridges, in metres (returned as n x n array)."""
    rng = np.random.default_rng(seed)
    ys = np.linspace(-size / 2, size / 2, n)
    X, Y = np.meshgrid(ys + TERRAIN_X0, ys)   # row index = y, column index = x (world coordinates)
    h = np.zeros_like(X)
    for (kx, ky, a) in ((1.3, 0.4, 0.012), (0.5, 1.7, 0.010), (2.9, 2.2, 0.006), (5.5, 1.0, 0.004)):
        h += a * np.sin(kx * X * 2 * math.pi / 2.0 + rng.uniform(0, 6)) * np.cos(ky * Y * 2 * math.pi / 2.0 + rng.uniform(0, 6))
    # furrows / ridges across the direction of travel, 25 mm high
    ridge = 0.025 * np.clip(np.cos((X - 0.9) * 2 * math.pi / 0.42), 0, 1) ** 2
    ridge *= ((X > 0.55) & (X < 2.3))
    h += ridge
    # a 12 degree ramp + plateau further on
    ramp = np.clip((X - 2.9) * math.tan(math.radians(12.0)), 0, 0.16) * (np.abs(Y) < 0.9)
    h += ramp
    # flat pad at the start
    pad = np.exp(-((X + 0.0) ** 2 + Y ** 2) / 0.12)
    h = h * (1 - pad)
    h -= h.min()
    return h


def terrain_scene(man):
    h = terrain_heights()
    hmax = float(h.max())
    os.makedirs(os.path.join(HERE, "assets"), exist_ok=True)
    # MuJoCo custom height-field file: int32 nrow, int32 ncol, float32 data (row = y, column = x)
    with open(os.path.join(HERE, "assets", "terrain.bin"), "wb") as fh:
        fh.write(np.array(h.shape, dtype=np.int32).tobytes())
        fh.write((h / hmax).astype(np.float32).tobytes())
    np.save(os.path.join(HERE, "assets", "terrain.npy"), h.astype(np.float32))
    # start height at the origin
    n = h.shape[0]
    z0 = float(h[n // 2, int(round((0.0 - TERRAIN_X0 + 3.0) / 6.0 * (n - 1)))])
    assets, body = robot_xml(man, base_pos=(0, 0, z0 + 0.004))
    world = "\n    ".join([
        '<light name="sun" directional="true" pos="-2 -3 6" dir="0.35 0.45 -1" diffuse="0.95 0.92 0.85" specular="0.3 0.3 0.3" castshadow="true"/>',
        f'<geom name="ground" type="hfield" hfield="field" pos="{TERRAIN_X0} 0 0" material="soil" friction="1.0 0.02 0.002" condim="4"/>',
    ])
    return HEADER.format(name="agrobot_terrain", stall=MOTOR_STALL, bias=-MOTOR_STALL / MOTOR_W0) + f"""
  <asset>
{SCENE_ASSETS.format(rep=6)}
    <hfield name="field" file="../../simulation/assets/terrain.bin" size="3 3 {hmax:.4f} 0.05"/>
{assets}
  </asset>
  <worldbody>
    {world}
{body}
  </worldbody>
{actuators_xml()}
</mujoco>
"""


def bare_scene(man):
    assets, body = robot_xml(man)
    return HEADER.format(name="agrobot_bare", stall=MOTOR_STALL, bias=-MOTOR_STALL / MOTOR_W0) + f"""
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.75 0.80 0.86" rgb2="0.96 0.96 0.96" width="512" height="3072"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.86 0.87 0.88" rgb2="0.80 0.81 0.82" width="512" height="512" mark="edge" markrgb="0.7 0.7 0.7"/>
    <material name="grid" texture="grid" texrepeat="10 10" texuniform="true" reflectance="0.08"/>
{assets}
  </asset>
  <worldbody>
    <light name="sun" directional="true" pos="-2 -3 6" dir="0.35 0.45 -1" diffuse="0.9 0.9 0.9" castshadow="true"/>
    <geom name="ground" type="plane" size="6 6 0.1" material="grid" friction="1.0 0.02 0.002" condim="4"/>
{body}
  </worldbody>
{actuators_xml()}
</mujoco>
"""


def main():
    man = load_manifest()
    os.makedirs(MODEL_DIR, exist_ok=True)
    os.makedirs(os.path.join(HERE, "assets"), exist_ok=True)
    soil_texture(os.path.join(HERE, "assets", "soil.png"))
    for name, fn in (("agrobot_bare", bare_scene), ("agrobot_row", row_scene), ("agrobot_terrain", terrain_scene)):
        with open(os.path.join(MODEL_DIR, name + ".xml"), "w") as fh:
            fh.write(fn(man))
        print("wrote", name + ".xml")


if __name__ == "__main__":
    main()
