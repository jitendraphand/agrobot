#!/usr/bin/env python3
"""Build every AgroBot part.

Outputs (all under cad/export/):
  step/<part>.step      printable parts, already in print orientation
  stl/<part>.stl        same orientation, ready for the slicer
  vitamins/<part>.stl   bought-in components (for visualisation only)
  agrobot_assembly.step complete robot, every solid in place
  manifest.json         kinematic tree + where every mesh goes (sim / Blender)
  print_report.md       bounding boxes vs. the Neptune 4 Max build volume

Usage:  python cad/build.py [--only name1,name2] [--no-assembly]
"""
import argparse
import json
import os
import sys
import time

import cadquery as cq

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from agrobot_cad import params as P          # noqa: E402
from agrobot_cad import assembly as A        # noqa: E402

OUT = os.path.join(HERE, "export")
STL_TOL, STL_ANG = 0.06, 0.25


def loc_to_pos_quat(loc):
    """cq.Location -> ([x,y,z] mm, [w,x,y,z])."""
    t = loc.wrapped.Transformation()
    p = t.TranslationPart()
    q = t.GetRotation()
    return [p.X(), p.Y(), p.Z()], [q.W(), q.X(), q.Y(), q.Z()]


def print_transform(shape, rots):
    """Rotate into print orientation and drop onto the bed centre.
    Returns (moved shape, Location mapping modelling frame -> print frame)."""
    loc = cq.Location()
    for (axis, ang) in rots:
        loc = cq.Location(cq.Vector(0, 0, 0), cq.Vector(*axis), ang) * loc
    s = shape.val().located(loc)
    bb = s.BoundingBox()
    shift = cq.Location(cq.Vector(-(bb.xmin + bb.xmax) / 2, -(bb.ymin + bb.ymax) / 2, -bb.zmin))
    loc = shift * loc
    return shape.val().located(loc), loc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", default="")
    ap.add_argument("--no-assembly", action="store_true")
    args = ap.parse_args()
    only = set(filter(None, args.only.split(",")))

    for sub in ("step", "stl", "vitamins"):
        os.makedirs(os.path.join(OUT, sub), exist_ok=True)

    t0 = time.time()
    defs = A.part_defs()
    print(f"modelled {len(defs)} solids in {time.time() - t0:.1f}s")

    manifest = dict(units="mm", links={}, parts=[], pose_default=A.default_pose(),
                    tcp=dict(gripper=P.TCP_GRIPPER, probe=P.TCP_PROBE, nozzle=P.TCP_NOZZLE))
    for name, L in A.LINKS.items():
        e = dict(parent=L["parent"], pos=list(L["pos"]), joint=L["joint"],
                 axis=list(L["axis"]) if L["axis"] else None,
                 limits=list(L["limits"]) if L.get("limits") else None)
        if L["rot"]:
            e["quat"] = loc_to_pos_quat(cq.Location(cq.Vector(0, 0, 0), cq.Vector(*L["rot"][0]), L["rot"][1]))[1]
        else:
            e["quat"] = [1, 0, 0, 0]
        manifest["links"][name] = e

    rows = []
    usable = (P.BED_X - 2 * P.BED_MARGIN, P.BED_Y - 2 * P.BED_MARGIN, P.BED_Z - P.BED_MARGIN)
    ok_all = True
    for d in defs:
        solid = d.shape.val()
        vol = solid.Volume()
        com = solid.Center()
        if d.printed:
            rho = P.RHO_TPU if "TPU" in d.material else P.RHO_PETG
            mass = vol * rho * P.FILL_FACTOR
        else:
            mass = d.mass_g
        moved, ploc = print_transform(d.shape, d.print_rot)
        bb = moved.BoundingBox()
        dims = (bb.xlen, bb.ylen, bb.zlen)
        sub = "stl" if d.printed else "vitamins"
        stl_rel = f"{sub}/{d.name}.stl"
        if not only or d.name in only:
            cq.exporters.export(cq.Workplane("XY").newObject([moved]), os.path.join(OUT, stl_rel),
                                tolerance=STL_TOL, angularTolerance=STL_ANG)
            if d.printed:
                cq.exporters.export(cq.Workplane("XY").newObject([moved]), os.path.join(OUT, "step", d.name + ".step"))
        inv = ploc.inverse
        placements = []
        for (link, loc) in d.placements:
            pos, quat = loc_to_pos_quat(loc * inv)
            cpos = (loc * cq.Location(com)).toTuple()[0]
            placements.append(dict(link=link, pos=pos, quat=quat, com=list(cpos)))
        mbb = solid.BoundingBox()
        manifest["parts"].append(dict(
            name=d.name, mesh=stl_rel, printed=d.printed, material=d.material if d.printed else "bought",
            color=list(d.color), mass_g=round(mass, 1), qty=len(d.placements),
            bbox_model=[[mbb.xmin, mbb.ymin, mbb.zmin], [mbb.xmax, mbb.ymax, mbb.zmax]],
            print_dims=[round(v, 1) for v in dims], placements=placements, note=d.note))
        if d.printed:
            fits = (max(dims[0], dims[1]) <= max(usable[0], usable[1])
                    and min(dims[0], dims[1]) <= min(usable[0], usable[1]) and dims[2] <= usable[2])
            ok_all &= fits
            rows.append((d.name, len(d.placements), d.material, dims, mass, fits, d.note))
            print(f"  {d.name:26s} {dims[0]:6.1f} x {dims[1]:6.1f} x {dims[2]:6.1f}  {mass:6.0f} g  {'OK' if fits else 'TOO BIG'}")

    with open(os.path.join(OUT, "manifest.json"), "w") as f:
        json.dump(manifest, f, indent=1)

    # ---- print report ----
    tot_petg = sum(r[4] * r[1] for r in rows if "TPU" not in r[2])
    tot_tpu = sum(r[4] * r[1] for r in rows if "TPU" in r[2])
    with open(os.path.join(OUT, "print_report.md"), "w") as f:
        f.write(f"# Print report - {P.PRINTER}\n\n")
        f.write(f"Build volume {P.BED_X:.0f} x {P.BED_Y:.0f} x {P.BED_Z:.0f} mm; "
                f"checked against {usable[0]:.0f} x {usable[1]:.0f} x {usable[2]:.0f} mm "
                f"({P.BED_MARGIN:.0f} mm margin). Files are exported in print orientation.\n\n")
        f.write("Mass = solid volume x density x %.2f (4 perimeters + ~25 %% infill). "
                "Treat it as an estimate; your slicer has the real number.\n\n" % P.FILL_FACTOR)
        f.write("| Part | Qty | Material | X x Y x Z (mm) | Est. mass each (g) | Fits | Notes |\n|---|---|---|---|---|---|---|\n")
        for (n, q, m, dm, ms, ok, note) in rows:
            f.write(f"| `{n}` | {q} | {m} | {dm[0]:.0f} x {dm[1]:.0f} x {dm[2]:.0f} | {ms:.0f} | {'yes' if ok else '**NO**'} | {note} |\n")
        f.write(f"\n**Totals:** PETG ~{tot_petg:.0f} g, TPU ~{tot_tpu:.0f} g, "
                f"{sum(r[1] for r in rows)} printed pieces from {len(rows)} unique files.\n")
    print(f"PETG ~{tot_petg:.0f} g  TPU ~{tot_tpu:.0f} g   all fit: {ok_all}")

    # ---- assembly STEP ----
    if not args.no_assembly and not only:
        wl = A.link_world_locations(A.default_pose())
        assy = cq.Assembly(name="AgroBot_Mk1")
        for d in defs:
            if d.name == "race_balls":
                continue
            for i, (link, loc) in enumerate(d.placements):
                nm = d.name if len(d.placements) == 1 else f"{d.name}_{i + 1}"
                assy.add(d.shape.val(), name=nm, loc=wl[link] * loc, color=cq.Color(*d.color, 1.0))
        assy.save(os.path.join(OUT, "agrobot_assembly.step"))
        print("assembly STEP written")
    print(f"done in {time.time() - t0:.1f}s")
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
