#!/usr/bin/env python3
"""Clash detection: intersect every pair of solids in the assembled robot.

    python cad/check_interference.py [--pose yaw,shoulder,elbow,wrist,grip]

Reports every pair whose common volume exceeds 1 mm^3.  Exit code 1 if any.
"""
import itertools
import os
import sys

import cadquery as cq

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from agrobot_cad import assembly as A   # noqa: E402

THRESH = 1.0   # mm^3


def main():
    pose = A.default_pose()
    for i, a in enumerate(sys.argv):
        if a == "--pose":
            v = [float(x) for x in sys.argv[i + 1].split(",")]
            pose = dict(yaw=v[0], shoulder=v[1], elbow=v[2], wrist=v[3], grip=v[4], grip_mimic=v[4])
    wl = A.link_world_locations(pose)
    items = []
    for d in A.part_defs():
        if d.name == "race_balls":
            continue
        for i, (link, loc) in enumerate(d.placements):
            s = d.shape.val().located(wl[link] * loc)
            items.append((f"{d.name}#{i}" if len(d.placements) > 1 else d.name, s, s.BoundingBox()))
    clashes = []
    for (na, sa, ba), (nb, sb, bb) in itertools.combinations(items, 2):
        if (ba.xmax < bb.xmin or bb.xmax < ba.xmin or ba.ymax < bb.ymin or bb.ymax < ba.ymin
                or ba.zmax < bb.zmin or bb.zmax < ba.zmin):
            continue
        try:
            common = sa.intersect(sb)
            vol = common.Volume() if common is not None else 0.0
        except Exception as e:      # noqa: BLE001
            vol = -1.0
        if vol > THRESH or vol < 0:
            cb = common.BoundingBox() if vol > 0 else None
            where = f"x[{cb.xmin:.0f},{cb.xmax:.0f}] y[{cb.ymin:.0f},{cb.ymax:.0f}] z[{cb.zmin:.0f},{cb.zmax:.0f}]" if cb else ""
            clashes.append((vol, na, nb, where))
    clashes.sort(reverse=True)
    for vol, na, nb, where in clashes:
        print(f"  {vol:9.1f} mm^3  {na:28s} x {nb:28s} {where}")
    print(f"{len(items)} solids, {len(clashes)} clashing pairs (pose {pose})")
    return 1 if clashes else 0


if __name__ == "__main__":
    sys.exit(main())
