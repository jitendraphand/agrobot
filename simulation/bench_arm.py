#!/usr/bin/env python3
"""Static arm bench test in MuJoCo: holding torque of every servo vs. payload.

    python simulation/bench_arm.py

The arm is held at full stretch just above horizontal (worst case for the
shoulder; exactly 0 deg would rest on the joint end stop and hide the load),
a point mass is welded to the gripper, and the steady-state servo torques are
compared with the stall torque of the servo chosen for that joint.
Writes simulation/output/arm_bench.json and prints a Markdown table.
"""
import json
import math
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gen_mjcf   # noqa: E402

JOINTS = ["yaw", "shoulder", "elbow", "wrist"]
POSES = {"full stretch, 6 deg above horizontal": (0, 6, 0, 0), "probing soil at the side": (97, 14, -88, 75),
         "home": (0, 115, -105, -10)}


def torques(pose_deg, payload_kg):
    xml = open(os.path.join(HERE, "model", "agrobot_bare.xml")).read()
    # weld a payload to the grasp centre
    xml = xml.replace('<site name="tcp_gripper"', f'<geom name="payload" type="sphere" size="0.02" mass="{max(payload_kg, 1e-6)}" '
                      f'pos="{gen_mjcf.P.L3 * 0.001} 0 0" contype="0" conaffinity="0" rgba="1 0 0 1"/>\n<site name="tcp_gripper"', 1)
    path = os.path.join(HERE, "model", "_bench.xml")
    with open(path, "w") as fh:
        fh.write(xml)
    m = mujoco.MjModel.from_xml_path(path)
    os.remove(path)
    d = mujoco.MjData(m)
    for j, q in zip(JOINTS, pose_deg):
        d.qpos[m.joint(j).qposadr[0]] = math.radians(q)
        d.ctrl[m.actuator("s_" + j).id] = math.radians(q)
    # freeze the chassis so only the arm settles
    for _ in range(1500):
        mujoco.mj_step(m, d)
    out = {}
    for j in JOINTS:
        out[j] = abs(float(d.actuator_force[m.actuator("s_" + j).id]))
    droop = math.degrees(abs(d.qpos[m.joint("shoulder").qposadr[0]] - math.radians(pose_deg[1])))
    tip = np.array(d.site_xpos[m.site("tcp_gripper").id])
    return out, droop, tip


def main():
    rows = []
    for name, pose in POSES.items():
        for payload in (0.0, 0.1, 0.15, 0.2, 0.3):
            tq, droop, tip = torques(pose, payload)
            margin = {j: gen_mjcf.SERVOS[j]["tau"] / tq[j] if tq[j] > 1e-3 else float("inf") for j in JOINTS}
            rows.append(dict(pose=name, payload_g=int(payload * 1000), torque_Nm={j: round(v, 2) for j, v in tq.items()},
                             shoulder_margin=round(margin["shoulder"], 2), elbow_margin=round(margin["elbow"], 2),
                             wrist_margin=round(margin["wrist"], 2), shoulder_droop_deg=round(droop, 2)))
    os.makedirs(os.path.join(HERE, "output"), exist_ok=True)
    with open(os.path.join(HERE, "output", "arm_bench.json"), "w") as fh:
        json.dump(dict(stall_Nm={j: gen_mjcf.SERVOS[j]["tau"] for j in JOINTS}, rows=rows), fh, indent=1)
    print("| Pose | Payload (g) | Shoulder (N m) | Elbow (N m) | Wrist (N m) | Shoulder margin | Elbow margin | Wrist margin |")
    print("|---|---|---|---|---|---|---|---|")
    for r in rows:
        t = r["torque_Nm"]
        print(f'| {r["pose"]} | {r["payload_g"]} | {t["shoulder"]:.2f} | {t["elbow"]:.2f} | {t["wrist"]:.2f} | '
              f'{r["shoulder_margin"]:.1f}x | {r["elbow_margin"]:.1f}x | {r["wrist_margin"]:.1f}x |')


if __name__ == "__main__":
    main()
