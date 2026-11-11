#!/usr/bin/env python3
"""Quick look at the CAD: render the generated MuJoCo model from a few angles.

    MUJOCO_GL=egl python tools/preview.py out_dir [model] [--pose yaw,shoulder,elbow,wrist,grip]
"""
import math, os, sys
import mujoco
import numpy as np
from PIL import Image

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    out = sys.argv[1]
    model = sys.argv[2] if len(sys.argv) > 2 and not sys.argv[2].startswith("--") else "agrobot_bare"
    pose = [0, 115, -105, -10, 20]
    for i, a in enumerate(sys.argv):
        if a == "--pose":
            pose = [float(v) for v in sys.argv[i + 1].split(",")]
    os.makedirs(out, exist_ok=True)
    m = mujoco.MjModel.from_xml_path(os.path.join(ROOT, "simulation", "model", model + ".xml"))
    d = mujoco.MjData(m)
    for name, deg in zip(("yaw", "shoulder", "elbow", "wrist", "grip"), pose):
        d.qpos[m.joint(name).qposadr[0]] = math.radians(deg)
    d.qpos[m.joint("grip_mimic").qposadr[0]] = math.radians(pose[4])
    mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, 900, 1200)
    opt = mujoco.MjvOption()
    opt.geomgroup[3] = 1 if "--collision" in sys.argv else 0
    cam = mujoco.MjvCamera()
    views = dict(iso=(135, -22, 1.25, (0.05, 0, 0.17)), front=(180, -8, 1.1, (0.1, 0, 0.17)), side=(90, -5, 1.2, (0.05, 0, 0.17)),
                 top=(90, -89, 1.2, (0.05, 0, 0.1)), rear=(35, -25, 1.2, (0.0, 0, 0.17)), arm=(120, -15, 0.6, (0.3, 0, 0.25)),
                 under=(140, 35, 1.0, (0, 0, 0.08)), grip=(100, -50, 0.35, (0.42, 0, 0.2)))
    for name, (az, el, dist, look) in views.items():
        cam.azimuth, cam.elevation, cam.distance = az, el, dist
        cam.lookat[:] = look
        r.update_scene(d, camera=cam, scene_option=opt)
        Image.fromarray(r.render()).save(os.path.join(out, f"{name}.png"))
    print("mass", sum(m.body_mass), "kg; bodies:", {m.body(i).name: round(float(m.body_mass[i]), 3) for i in range(1, m.nbody) if m.body_mass[i] > 0.01})


if __name__ == "__main__":
    main()
