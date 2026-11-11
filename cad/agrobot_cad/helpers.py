"""Small CadQuery helpers used by every part."""
from math import cos, sin, radians, pi, atan2, sqrt, tan, acos

import cadquery as cq

from . import params as P


def W(obj):
    """Wrap a raw OCC solid in a Workplane."""
    return cq.Workplane("XY").newObject([obj])


def box(x0, x1, y0, y1, z0, z1):
    """Axis-aligned box from explicit bounds."""
    return W(cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, cq.Vector(x0, y0, z0)))


def cyl(d, h, p=(0, 0, 0), n=(0, 0, 1)):
    """Cylinder of diameter d, height h, starting at p along direction n."""
    return W(cq.Solid.makeCylinder(d / 2.0, h, cq.Vector(*p), cq.Vector(*n)))


def cone(d0, d1, h, p=(0, 0, 0), n=(0, 0, 1)):
    return W(cq.Solid.makeCone(d0 / 2.0, d1 / 2.0, h, cq.Vector(*p), cq.Vector(*n)))


def rbox(x0, x1, y0, y1, z0, z1, r):
    """Box with vertical edges filleted."""
    return (cq.Workplane("XY").workplane(offset=z0)
            .center((x0 + x1) / 2, (y0 + y1) / 2)
            .rect(x1 - x0, y1 - y0).extrude(z1 - z0)
            .edges("|Z").fillet(r))


def union_all(items):
    out = items[0]
    for it in items[1:]:
        out = out.union(it)
    return out


def polar(r, ang_deg, cx=0.0, cy=0.0):
    a = radians(ang_deg)
    return (cx + r * cos(a), cy + r * sin(a))


def frame(origin, xdir, zdir):
    """A cq.Plane used to drop servo / horn features anywhere in space."""
    return cq.Plane(origin=cq.Vector(*origin), xDir=cq.Vector(*xdir), normal=cq.Vector(*zdir))


def in_frame(wp, pl):
    """Move a shape modelled in world XY coordinates into plane `pl`."""
    loc = pl.location
    return W(wp.val().located(loc)) if wp.size() == 1 else wp.newObject(
        [o.located(loc) for o in wp.vals()])


# --------------------------------------------------------------------------
# Servo features (see params.py for the servo frame convention)
# --------------------------------------------------------------------------
def servo_pocket(pl, screw_d=P.M3_SELF_TAP, screw_depth=9.0, through=40.0):
    """Cutter: rectangular pocket for the servo case plus 4 pilot holes."""
    cx = P.SERVO_AXIS_OFF
    c = box(cx - P.SERVO_POCKET_L / 2, cx + P.SERVO_POCKET_L / 2,
            -P.SERVO_POCKET_W / 2, P.SERVO_POCKET_W / 2, -through, 0.01)
    for sx in (-1, 1):
        for sy in (-1, 1):
            c = c.union(cyl(screw_d, screw_depth + 0.01,
                            (cx + sx * P.SERVO_HOLE_DX / 2, sy * P.SERVO_HOLE_DY / 2, -screw_depth)))
    return in_frame(c, pl)


def horn_holes(pl, t, centre_d=9.0, bolt_d=P.M3_CLEAR, rot=45.0):
    """Cutter: 4 x M3 on the horn PCD plus a centre access hole, through
    a plate of thickness t whose near face sits on the horn (z = 0 .. t)."""
    c = cyl(centre_d, t + 0.4, (0, 0, -0.2))
    for i in range(4):
        x, y = polar(P.HORN_PCD / 2, rot + 90 * i)
        c = c.union(cyl(bolt_d, t + 0.4, (x, y, -0.2)))
    return in_frame(c, pl)


def journal_boss(pl, length):
    """Solid: thrust shoulder + printed journal the idler plate rides on.
    Plane origin on the box wall, +Z pointing outwards along the joint axis."""
    b = cyl(18.0, 1.0).union(cyl(P.JOURNAL_D, 1.0 + length + 0.6))
    return in_frame(b, pl)


def journal_insert(pl, length):
    """Cutter: M3 heat-set insert pocket in the tip of the journal."""
    top = 1.0 + length + 0.6
    return in_frame(cyl(P.M3_INSERT, P.M3_INSERT_DEPTH + 1, (0, 0, top - P.M3_INSERT_DEPTH)), pl)


# --------------------------------------------------------------------------
# Involute spur gear (used by the gripper fingers)
# --------------------------------------------------------------------------
def involute_gear_profile(module, teeth, pressure_deg=20.0, backlash=0.15, phase_deg=0.0, n_pts=8):
    """Return a closed list of (x, y) points for an external spur gear."""
    z = teeth
    rp = module * z / 2.0
    rb = rp * cos(radians(pressure_deg))
    ra = rp + module
    rf = rp - 1.25 * module

    def inv(a):
        return tan(a) - a

    # half tooth thickness angle at the pitch circle, thinned for backlash
    half_thick = (pi * module / 2.0 - backlash) / (2.0 * rp)
    alpha_p = radians(pressure_deg)
    pts = []
    for k in range(z):
        centre = radians(phase_deg) + 2 * pi * k / z
        right, left = [], []
        r0 = max(rb, rf)
        for i in range(n_pts + 1):
            r = r0 + (ra - r0) * i / n_pts
            a = acos(min(1.0, rb / r))
            # angle from tooth centre line to the flank at radius r
            th = half_thick + inv(alpha_p) - inv(a)
            right.append((r, centre - th))
            left.append((r, centre + th))
        # root on the right side (radial line down to the root circle)
        if rf < rb:
            pts.append((rf, right[0][1]))
        pts.extend(right)
        pts.extend(reversed(left))
        if rf < rb:
            pts.append((rf, left[0][1]))
    return [(r * cos(a), r * sin(a)) for r, a in pts]


def spur_gear(module, teeth, thickness, phase_deg=0.0, backlash=0.15):
    prof = involute_gear_profile(module, teeth, phase_deg=phase_deg, backlash=backlash)
    return cq.Workplane("XY").polyline(prof).close().extrude(thickness)
