#!/usr/bin/env python3
"""Slicer-readiness check for every printable STL in cad/export/stl:
one closed (watertight) body, sitting on Z = 0.

    pip install trimesh scipy
    python cad/check_meshes.py
"""
import glob
import os
import sys

import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    files = sorted(glob.glob(os.path.join(HERE, "export", "stl", "*.stl")))
    bad = 0
    for f in files:
        m = trimesh.load(f)
        ok = m.is_watertight and m.body_count == 1 and abs(m.bounds[0][2]) < 1e-3 and m.volume > 0
        if not ok:
            bad += 1
            print(f"FAIL {os.path.basename(f)}: watertight={m.is_watertight} bodies={m.body_count} zmin={m.bounds[0][2]:.3f}")
    print(f"{len(files)} STL files checked, {bad} problem(s)")
    return 1 if bad or not files else 0


if __name__ == "__main__":
    sys.exit(main())
