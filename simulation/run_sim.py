#!/usr/bin/env python3
"""Firmware-in-the-loop simulation of AgroBot in MuJoCo.

    MUJOCO_GL=egl python simulation/run_sim.py row      # crop-row mission video
    MUJOCO_GL=egl python simulation/run_sim.py terrain  # rough-ground driving video
    python simulation/run_sim.py row --no-video         # physics + report only

Outputs go to simulation/output/:
    <scenario>.mp4         rendered video
    <scenario>_traj.npz    body poses per frame (replayed by Blender)
    <scenario>_report.json measured results (tracking error, torques, ...)

The controller is the real firmware core (C++), loaded through ctypes; see
agro_fw.py.  The simulator only plays the role of the hardware:
motors, servos, encoders, IMU, battery, soil probe.
"""
import argparse
import json
import math
import os
import subprocess
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "cad"))
import agro_fw                      # noqa: E402
import gen_mjcf                     # noqa: E402
from agrobot_cad import params as P  # noqa: E402

OUT = os.path.join(HERE, "output")
CONTROL_DT = 0.01                   # firmware tick (100 Hz)
FPS = 30
ENC_TICKS = P.MOTOR_ENC_PPR * 4 * P.MOTOR_GEAR_RATIO
SOIL_RAW_DRY, SOIL_RAW_WET = 2900, 1250      # must match agro_config.h
ARM_JOINTS = ["yaw", "shoulder", "elbow", "wrist", "grip"]


class Sim:
    def __init__(self, model_name, soil_moisture=None, seed=1):
        self.m = mujoco.MjModel.from_xml_path(os.path.join(HERE, "model", model_name + ".xml"))
        self.d = mujoco.MjData(self.m)
        self.fw = agro_fw.Firmware()
        self.rng = np.random.default_rng(seed)
        self.soil_moisture = soil_moisture or []         # [(x, y, percent)] wet-bulb zones
        m = self.m
        self.act = {n: m.actuator(n).id for n in ["m_fl", "m_rl", "m_fr", "m_rr"] + ["s_" + j for j in ARM_JOINTS]}
        self.qadr = {j: m.joint(j).qposadr[0] for j in ARM_JOINTS + ["grip_mimic", "wheel_rl", "wheel_rr", "wheel_fl", "wheel_fr"]}
        self.dadr = {j: m.joint(j).dofadr[0] for j in ARM_JOINTS}
        self.chassis = m.body("chassis").id
        self.site = {n: m.site(n).id for n in ("tcp_gripper", "tcp_probe", "tcp_nozzle")}
        self.gyro_adr = m.sensor("gyro").adr[0]
        self.steps_per_tick = int(round(CONTROL_DT / m.opt.timestep))
        self.t_log, self.log = [], []
        self.events = []
        self.pump_on = False
        self.frames_pose = []
        self.max_tau = {j: 0.0 for j in ARM_JOINTS}
        self.track_err = {j: 0.0 for j in ARM_JOINTS[:4]}
        # start in the firmware's power-on pose
        out = self.fw.step()
        for i, j in enumerate(ARM_JOINTS):
            self.d.qpos[self.qadr[j]] = out.joint[i]
            self.d.ctrl[self.act["s_" + j]] = out.joint[i]
        self.d.qpos[self.qadr["grip_mimic"]] = out.joint[4]
        mujoco.mj_forward(m, self.d)
        self.settle(0.6)

    # ------------------------------------------------------------ sensors
    def chassis_pose(self):
        return self.d.xpos[self.chassis].copy(), self.d.xmat[self.chassis].reshape(3, 3).copy()

    def to_robot_frame(self, p_world):
        pos, R = self.chassis_pose()
        return R.T @ (np.asarray(p_world) - pos)

    def soil_raw(self):
        """Capacitive probe model: reads 'air' unless the blade tip is in the ground."""
        tip = self.d.site_xpos[self.site["tcp_probe"]]
        ground = self.ground_height(tip[0], tip[1])
        depth = ground - tip[2]
        if depth < 0.008:
            pct = 0.0
        else:
            pct = 30.0
            best = 1e9
            for (x, y, v) in self.soil_moisture:
                dist = math.hypot(tip[0] - x, tip[1] - y)
                if dist < best:
                    best, pct = dist, v
            pct *= min(1.0, depth / 0.035)               # partial insertion reads low
        raw = SOIL_RAW_DRY - pct / 100.0 * (SOIL_RAW_DRY - SOIL_RAW_WET)
        return int(raw + self.rng.normal(0, 8))

    def ground_height(self, x, y):
        return 0.0

    def read_inputs(self):
        inp = self.fw.inp
        inp.dt = CONTROL_DT
        # one encoder per side (rear motors), as wired on the real robot
        inp.encL = int(round(self.d.qpos[self.qadr["wheel_rl"]] / (2 * math.pi) * ENC_TICKS))
        inp.encR = int(round(self.d.qpos[self.qadr["wheel_rr"]] / (2 * math.pi) * ENC_TICKS))
        gyro = self.d.sensordata[self.gyro_adr:self.gyro_adr + 3]
        inp.imuValid = 1
        inp.gyroZ = float(gyro[2] + self.rng.normal(0, 0.002))
        _, R = self.chassis_pose()
        inp.roll = float(math.atan2(R[2, 1], R[2, 2]))
        inp.pitch = float(-math.asin(max(-1.0, min(1.0, R[2, 0]))))
        inp.battV = 11.6
        inp.soilRaw = self.soil_raw()
        inp.rangeM = 9.9
        inp.estop = 0

    # --------------------------------------------------------------- step
    def tick(self):
        """One firmware tick + the physics that happens during it."""
        self.read_inputs()
        out = self.fw.step()
        en = 1.0 if out.motorsEnabled else 0.0
        c = self.d.ctrl
        c[self.act["m_fl"]] = c[self.act["m_rl"]] = out.dutyL * en
        c[self.act["m_fr"]] = c[self.act["m_rr"]] = out.dutyR * en
        if out.servosEnabled:
            for i, j in enumerate(ARM_JOINTS):
                c[self.act["s_" + j]] = out.joint[i]
        if bool(out.pump) != self.pump_on:
            self.pump_on = bool(out.pump)
            self.events.append((round(self.d.time, 2), "pump on" if self.pump_on else "pump off"))
        for _ in range(self.steps_per_tick):
            mujoco.mj_step(self.m, self.d)
        for i, j in enumerate(ARM_JOINTS):
            tau = abs(self.d.actuator_force[self.act["s_" + j]])
            self.max_tau[j] = max(self.max_tau[j], tau)
            if i < 4:
                self.track_err[j] = max(self.track_err[j], abs(self.d.qpos[self.qadr[j]] - out.joint[i]))
        return out

    def settle(self, seconds):
        for _ in range(int(seconds / CONTROL_DT)):
            self.tick()

    def command(self, line):
        ok, reply = self.fw.command(line)
        self.events.append((round(self.d.time, 2), f"> {line}   {reply}"))
        print(f"[{self.d.time:7.2f}s] > {line:42s} {reply}")
        return ok

    def body_poses(self):
        return np.concatenate([self.d.xpos[1:].ravel(), self.d.xquat[1:].ravel()]).astype(np.float32)


# ----------------------------------------------------------------- rendering
class Recorder:
    """Offscreen renderer -> ffmpeg, with a keyframed chase camera and overlays."""

    def __init__(self, sim, path, width=1280, height=720, pip=True):
        self.sim = sim
        self.w, self.h = width, height
        self.r = mujoco.Renderer(sim.m, height, width)
        self.pip = mujoco.Renderer(sim.m, 240, 320) if pip else None
        self.cam = mujoco.MjvCamera()
        self.opt = mujoco.MjvOption()
        self.opt.geomgroup[3] = 0
        self.keys = []                 # (time, azimuth, elevation, distance, (dx,dy,dz))
        self.look = None
        self.caption = ""
        self.sub = ""
        self.droplets = []
        from PIL import ImageFont
        self.font = self._font(26)
        self.small = self._font(18)
        cmd = ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{width}x{height}",
               "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-preset", "medium", "-crf", "23", "-pix_fmt", "yuv420p",
               "-movflags", "+faststart", path]
        self.ff = subprocess.Popen(cmd, stdin=subprocess.PIPE)
        self.n = 0

    @staticmethod
    def _font(size):
        from PIL import ImageFont
        for f in ("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf"):
            if os.path.exists(f):
                return ImageFont.truetype(f, size)
        return ImageFont.load_default()

    def key(self, t, az, el, dist, off=(0.1, 0.0, 0.18)):
        self.keys.append((t, az, el, dist, off))

    def _camera(self, t):
        ks = self.keys
        if t <= ks[0][0]:
            k = ks[0][1:]
        elif t >= ks[-1][0]:
            k = ks[-1][1:]
        else:
            for a, b in zip(ks, ks[1:]):
                if a[0] <= t <= b[0]:
                    s = (t - a[0]) / (b[0] - a[0])
                    s = s * s * (3 - 2 * s)
                    k = (a[1] + s * (b[1] - a[1]), a[2] + s * (b[2] - a[2]), a[3] + s * (b[3] - a[3]),
                         tuple(a[4][i] + s * (b[4][i] - a[4][i]) for i in range(3)))
                    break
        az, el, dist, off = k
        pos, R = self.sim.chassis_pose()
        yaw = math.atan2(R[1, 0], R[0, 0])
        target = pos + np.array([off[0] * math.cos(yaw) - off[1] * math.sin(yaw),
                                 off[0] * math.sin(yaw) + off[1] * math.cos(yaw), off[2]])
        # low-pass the look-at point so bumps do not shake the camera
        self.look = target if self.look is None else self.look + 0.12 * (target - self.look)
        self.cam.azimuth, self.cam.elevation, self.cam.distance = az, el, dist
        self.cam.lookat[:] = self.look

    def _spray(self, scn):
        """Draw the spray as droplets falling from the nozzle while the pump runs."""
        sim = self.sim
        rng = sim.rng
        if sim.pump_on:
            p0 = sim.d.site_xpos[sim.site["tcp_nozzle"]].copy()
            for _ in range(14):
                a = rng.uniform(0, 2 * math.pi)
                sp = rng.uniform(0.0, 0.35)
                self.droplets.append([p0.copy(), np.array([sp * math.cos(a), sp * math.sin(a), -1.6]), 0.0])
        dt = 1.0 / FPS
        alive = []
        for dr in self.droplets:
            dr[1][2] -= 9.81 * dt
            dr[0] += dr[1] * dt
            dr[2] += dt
            if dr[0][2] > 0.005 and dr[2] < 0.6:
                alive.append(dr)
        self.droplets = alive
        for dr in self.droplets:
            if scn.ngeom >= scn.maxgeom:
                break
            g = scn.geoms[scn.ngeom]
            mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, np.array([0.004, 0, 0]), dr[0], np.eye(3).ravel(),
                                np.array([0.55, 0.80, 1.0, 0.75], dtype=np.float32))
            scn.ngeom += 1

    def frame(self):
        from PIL import Image, ImageDraw
        sim = self.sim
        self._camera(sim.d.time)
        self.r.update_scene(sim.d, camera=self.cam, scene_option=self.opt)
        self._spray(self.r.scene)
        img = Image.fromarray(self.r.render())
        draw = ImageDraw.Draw(img, "RGBA")
        if self.pip is not None:
            self.pip.update_scene(sim.d, camera="onboard", scene_option=self.opt)
            pip = Image.fromarray(self.pip.render())
            x0, y0 = self.w - 320 - 20, 20
            draw.rectangle([x0 - 3, y0 - 3, x0 + 322, y0 + 242], fill=(255, 255, 255, 230))
            img.paste(pip, (x0, y0))
            draw.rectangle([x0, y0 + 214, x0 + 320, y0 + 240], fill=(0, 0, 0, 150))
            draw.text((x0 + 8, y0 + 217), "onboard camera (ESP32-CAM)", font=self.small, fill=(255, 255, 255, 255))
        # status panel
        draw.rectangle([18, 18, 560, 108], fill=(0, 0, 0, 140))
        draw.text((30, 24), self.caption, font=self.font, fill=(255, 255, 255, 255))
        draw.text((30, 58), self.sub, font=self.small, fill=(215, 235, 215, 255))
        draw.text((30, 82), f"t = {sim.d.time:5.1f} s    firmware state: {sim.fw.state}", font=self.small, fill=(200, 200, 200, 255))
        self.ff.stdin.write(np.asarray(img).tobytes())
        self.n += 1

    def close(self):
        self.ff.stdin.close()
        self.ff.wait()


# ------------------------------------------------------------------ scenarios
def run(sim, rec, until, caption=None, sub=None, max_time=120.0, traj=True):
    """Advance until `until()` is true.  Records video frames and trajectory."""
    if rec is not None and caption is not None:
        rec.caption = caption
    if rec is not None and sub is not None:
        rec.sub = sub
    t_end = sim.d.time + max_time
    per_frame = int(round(1.0 / FPS / CONTROL_DT))        # ticks per video frame (3.33 -> use time)
    next_frame = getattr(sim, "_next_frame", sim.d.time)
    while sim.d.time < t_end:
        sim.tick()
        if sim.d.time >= next_frame:
            next_frame += 1.0 / FPS
            if traj:
                sim.frames_pose.append(sim.body_poses())
                sim.t_log.append(sim.d.time)
                sim.log.append((sim.pump_on, sim.fw.out.state))
            if rec is not None:
                if callable(rec.sub_fn):
                    rec.sub = rec.sub_fn()
                rec.frame()
        if until():
            break
    sim._next_frame = next_frame
    return sim.d.time < t_end


def scenario_row(video):
    plants = [(x, gen_mjcf.ROW_Y) for x in gen_mjcf.PLANT_XS]
    moisture = [(plants[0][0], plants[0][1], 24.0), (plants[1][0], plants[1][1], 66.0), (plants[2][0], plants[2][1], 27.0)]
    sim = Sim("agrobot_row", soil_moisture=moisture)
    rec = None
    if video:
        rec = Recorder(sim, os.path.join(OUT, "row_mission.mp4"))
        rec.sub_fn = None
        # (time, azimuth, elevation, distance, look-at offset in the robot frame)
        for k in [(0, 215, -18, 1.9, (0.2, 0.1, 0.15)), (5, 235, -22, 1.5, (0.15, 0.15, 0.15)),
                  (12, 250, -20, 1.15, (0.1, 0.2, 0.12)), (26, 300, -24, 1.2, (0.1, 0.2, 0.15)),
                  (38, 240, -28, 1.25, (0.1, 0.2, 0.15)), (52, 205, -16, 1.2, (0.1, 0.2, 0.2)),
                  (66, 250, -22, 1.1, (0.1, 0.2, 0.2)), (78, 285, -25, 1.0, (0.08, 0.2, 0.25)),
                  (92, 320, -30, 1.15, (0.0, 0.1, 0.25)), (104, 250, -20, 1.6, (0.1, 0.0, 0.2))]:
            rec.key(*k)

    fw = sim.fw
    fruit_id = sim.m.body("fruit").id
    report = dict(scenario="row", plants=[], events=sim.events)
    fruit0 = sim.d.xpos[fruit_id].copy()

    def sub():
        log = json.loads(fw.command("STATUS")[1]).get("log", [])
        txt = "soil moisture: " + "  ".join(f"plant {i + 1}: {v:.0f}%" for i, v in enumerate(log)) if log else "soil moisture: -"
        return txt + ("     PUMP ON" if sim.pump_on else "")

    if rec:
        rec.sub_fn = sub
    run(sim, rec, lambda: sim.d.time > 1.0, "AgroBot Mk1 - crop row mission", max_time=3)
    first = (plants[0][0] - P.YAW_X * 0.001) * 1000
    spacing = (plants[1][0] - plants[0][0]) * 1000
    assert sim.command(f"ROW 3 {first:.0f} {spacing:.0f} {gen_mjcf.ROW_Y * 1000:.0f}")

    probe_site = sim.site["tcp_probe"]
    stops = []
    state = dict(min_probe=1.0, last_pc=-1)

    def mission_running():
        tip = sim.d.site_xpos[probe_site]
        state["min_probe"] = min(state["min_probe"], tip[2])
        return fw.state != "MISSION"

    ok = run(sim, rec, mission_running, "Row mission: probe soil, spray if dry", max_time=150)
    status = json.loads(fw.command("STATUS")[1])
    pos, R = sim.chassis_pose()
    report["row_mission"] = dict(completed=bool(ok and fw.state == "IDLE"), duration_s=round(sim.d.time, 1),
                                 soil_log_pct=status["log"], true_moisture_pct=[m[2] for m in moisture],
                                 final_x_m=round(float(pos[0]), 4), expected_x_m=round(plants[2][0] - P.YAW_X * 0.001, 4),
                                 lateral_drift_m=round(float(pos[1]), 4),
                                 heading_drift_deg=round(math.degrees(math.atan2(R[1, 0], R[0, 0])), 2),
                                 odometry_x_m=round(status["x"], 4), deepest_probe_m=round(float(state["min_probe"]), 4),
                                 pump_events=[e for e in sim.events if "pump" in e[1]])
    print("row mission:", json.dumps(report["row_mission"], indent=1))

    # ---- harvest: fruit position would come from a vision system; here we read it from the simulator ----
    f_world = sim.d.xpos[fruit_id].copy()
    f_robot = sim.to_robot_frame(f_world)
    dia = 2 * gen_mjcf.FRUIT_R * 1000
    assert sim.command(f"PICK {f_robot[0] * 1000:.0f} {f_robot[1] * 1000:.0f} {f_robot[2] * 1000:.0f} {dia:.0f}")
    lifted = dict(max_z=0.0)

    def pick_running():
        lifted["max_z"] = max(lifted["max_z"], sim.d.xpos[fruit_id][2])
        return fw.state != "MISSION"

    if rec:
        rec.sub_fn = lambda: "harvest: pick ripe fruit -> basket (fruit position given, no vision in the loop)"
    ok = run(sim, rec, pick_running, "Harvest: pick and place in the basket", max_time=90)
    run(sim, rec, lambda: False, max_time=1.5)
    f_end = sim.to_robot_frame(sim.d.xpos[fruit_id])
    in_basket = (abs(f_end[0] - P.BASKET_X * 0.001) < P.BASKET_L * 0.0005 and abs(f_end[1] - P.BASKET_Y * 0.001) < P.BASKET_W * 0.0005
                 and f_end[2] > P.DECK_TOP * 0.001 - 0.005)
    report["harvest"] = dict(completed=bool(ok), fruit_start_world=[round(float(v), 3) for v in fruit0],
                             fruit_max_height_m=round(lifted["max_z"], 3),
                             fruit_end_robot_frame=[round(float(v), 3) for v in f_end], in_basket=bool(in_basket))
    print("harvest:", json.dumps(report["harvest"], indent=1))

    # ---- drive away ----
    if rec:
        rec.sub_fn = lambda: "done - driving on"
    sim.command("MOVE 500 250")
    run(sim, rec, lambda: fw.state == "IDLE", "Mission complete", max_time=8)
    run(sim, rec, lambda: False, max_time=1.0)
    f_end = sim.to_robot_frame(sim.d.xpos[fruit_id])
    report["harvest"]["still_in_basket_after_driving"] = bool(abs(f_end[0] - P.BASKET_X * 0.001) < P.BASKET_L * 0.0005
                                                              and abs(f_end[1]) < P.BASKET_W * 0.0005 and f_end[2] > 0.13)
    finish(sim, rec, report, "row_mission")
    return report


class TerrainSim(Sim):
    def __init__(self, *a, **k):
        self.h = np.load(os.path.join(HERE, "assets", "terrain.npy"))
        super().__init__(*a, **k)

    def ground_height(self, x, y):
        n = self.h.shape[0]
        i = int(round((y + 3.0) / 6.0 * (n - 1)))
        j = int(round((x - gen_mjcf.TERRAIN_X0 + 3.0) / 6.0 * (n - 1)))
        return float(self.h[min(max(i, 0), n - 1), min(max(j, 0), n - 1)])


def scenario_terrain(video):
    sim = TerrainSim("agrobot_terrain")
    rec = None
    if video:
        rec = Recorder(sim, os.path.join(OUT, "terrain_test.mp4"))
        rec.sub_fn = None
        for k in [(0, 200, -18, 1.8, (0.1, 0, 0.15)), (6, 250, -14, 1.5, (0.1, 0, 0.12)), (14, 290, -18, 1.5, (0.1, 0, 0.12)),
                  (22, 230, -22, 1.7, (0.1, 0, 0.15)), (30, 180, -25, 1.9, (0, 0, 0.15)), (40, 130, -20, 2.0, (0, 0, 0.15))]:
            rec.key(*k)
    fw = sim.fw
    report = dict(scenario="terrain", events=sim.events)
    stats = dict(max_roll=0.0, max_pitch=0.0, max_speed=0.0)

    def sub():
        pos, R = sim.chassis_pose()
        roll = math.degrees(math.atan2(R[2, 1], R[2, 2]))
        pitch = math.degrees(-math.asin(max(-1, min(1, R[2, 0]))))
        v = float(np.linalg.norm(sim.d.qvel[0:2]))
        stats["max_roll"] = max(stats["max_roll"], abs(roll))
        stats["max_pitch"] = max(stats["max_pitch"], abs(pitch))
        stats["max_speed"] = max(stats["max_speed"], v)
        return f"speed {v:4.2f} m/s   pitch {pitch:+5.1f} deg   roll {roll:+5.1f} deg   duty L/R {fw.out.dutyL:+.2f} {fw.out.dutyR:+.2f}"

    class Sub:
        pass
    if rec:
        rec.sub_fn = sub
    else:
        Sub.fn = sub

    def idle():
        if rec is None:
            sub()
        return fw.state == "IDLE"

    run(sim, rec, lambda: sim.d.time > 1.0, "AgroBot Mk1 - rough ground test", max_time=3)
    sim.command("ARM STOW")
    run(sim, rec, lambda: not fw.out.busy, "Stow the arm for travel", max_time=6)
    x0 = sim.chassis_pose()[0].copy()
    sim.command("MOVE 2500 300")
    ok1 = run(sim, rec, idle, "25 mm ridges at 0.3 m/s (closed-loop wheel speed)", max_time=20)
    p1 = sim.chassis_pose()[0].copy()
    sim.command("MOVE 1100 200")
    ok2 = run(sim, rec, idle, "12 degree ramp", max_time=15)
    p2 = sim.chassis_pose()[0].copy()
    _, R = sim.chassis_pose()
    h0 = math.atan2(R[1, 0], R[0, 0])
    sim.command("TURN 180 60")
    t_turn = sim.d.time
    ok3 = run(sim, rec, idle, "Skid-steer turn on the spot (gyro heading)", max_time=30)
    t_turn = sim.d.time - t_turn
    _, R = sim.chassis_pose()
    h1 = math.atan2(R[1, 0], R[0, 0])
    turned = math.degrees((h1 - h0 + math.pi) % (2 * math.pi) - math.pi)
    sim.command("MOVE 900 250")
    ok4 = run(sim, rec, idle, "Back down the ramp", max_time=12)
    run(sim, rec, lambda: False, max_time=1.0)
    report["terrain"] = dict(ridges_ok=bool(ok1), ramp_ok=bool(ok2), turn_ok=bool(ok3), descent_ok=bool(ok4),
                             distance_ridges_m=round(float(p1[0] - x0[0]), 3), commanded_ridges_m=2.5,
                             climb_m=round(float(p2[2] - p1[2]), 3), distance_ramp_m=round(float(np.linalg.norm(p2[:2] - p1[:2])), 3),
                             turn_deg=round(turned, 1), turn_time_s=round(t_turn, 1), max_roll_deg=round(stats["max_roll"], 1),
                             max_pitch_deg=round(stats["max_pitch"], 1), max_speed_m_s=round(stats["max_speed"], 2))
    print("terrain:", json.dumps(report["terrain"], indent=1))
    finish(sim, rec, report, "terrain_test")
    return report


def finish(sim, rec, report, name):
    report["servo_peak_torque_Nm"] = {j: round(v, 2) for j, v in sim.max_tau.items()}
    report["servo_stall_torque_Nm"] = {j: gen_mjcf.SERVOS[j]["tau"] for j in ARM_JOINTS}
    report["servo_peak_tracking_error_deg"] = {j: round(math.degrees(v), 2) for j, v in sim.track_err.items()}
    report["sim_time_s"] = round(sim.d.time, 1)
    report["mass_kg"] = round(float(sum(sim.m.body_mass[sim.chassis:sim.chassis + 11])), 2)
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, name + "_report.json"), "w") as fh:
        json.dump(report, fh, indent=1)
    names = [sim.m.body(i).name for i in range(1, sim.m.nbody)]
    np.savez_compressed(os.path.join(OUT, name + "_traj.npz"), t=np.array(sim.t_log, dtype=np.float32),
                        poses=np.array(sim.frames_pose), bodies=np.array(names),
                        pump=np.array([l[0] for l in sim.log]), state=np.array([l[1] for l in sim.log]))
    if rec:
        rec.close()
        print(f"video: {rec.n} frames")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scenario", choices=["row", "terrain"])
    ap.add_argument("--no-video", action="store_true")
    args = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    {"row": scenario_row, "terrain": scenario_terrain}[args.scenario](not args.no_video)


if __name__ == "__main__":
    main()
