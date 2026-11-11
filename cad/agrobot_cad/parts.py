"""Every printed part of AgroBot Mk1.

Each function returns a cq.Workplane holding one solid, modelled in the frame of
the kinematic link it belongs to (see assembly.py):

  chassis    : robot frame (origin on the ground under the chassis centre)
  wheel      : origin at wheel centre, +Y = axle pointing outboard (left wheel)
  turntable  : origin on the yaw axis at the underside of the turntable disc
  upper_arm  : origin on the shoulder axis, +X towards the elbow
  forearm    : origin on the elbow axis, +X towards the wrist
  tool       : origin on the wrist axis, +X along the gripper
  finger_*   : origin on the finger pivot, +X towards the finger tip
"""
from math import cos, sin, radians, pi, sqrt

import cadquery as cq

from . import params as P
from .helpers import (W, box, cyl, cone, rbox, union_all, polar, frame, in_frame,
                      servo_pocket, horn_holes, journal_boss, journal_insert, spur_gear)

# ==========================================================================
# CHASSIS
# ==========================================================================
HL, HW = P.TRAY_L / 2, P.TRAY_W / 2
Z0 = P.TRAY_Z0
MOTOR_Z = P.AXLE_Z + P.MOTOR_SHAFT_OFF      # gearbox centre sits above the shaft


def _rounded_plate(l, w, r, z0, t):
    return (cq.Workplane("XY").workplane(offset=z0).rect(l, w).extrude(t)
            .edges("|Z").fillet(r))


def chassis_tray():
    """Main tub: carries the four gear motors, battery and electronics."""
    t = P.TRAY_WALL
    outer = _rounded_plate(P.TRAY_L, P.TRAY_W, P.TRAY_CORNER_R, Z0, P.TRAY_H)
    inner = _rounded_plate(P.TRAY_L - 2 * t, P.TRAY_W - 2 * t, P.TRAY_CORNER_R - t,
                           Z0 + P.TRAY_FLOOR, P.TRAY_H)
    body = outer.cut(inner)
    zf = Z0 + P.TRAY_FLOOR                       # inside floor level
    ztop = Z0 + P.TRAY_H

    adds, cuts = [], []

    # --- deck screw posts (heat-set inserts in the top) ---
    for (x, y) in P.POSTS:
        adds.append(cyl(13.0, P.TRAY_H - P.TRAY_FLOOR, (x, y, zf)))
        cuts.append(cyl(P.M3_INSERT, P.M3_INSERT_DEPTH + 0.01, (x, y, ztop - P.M3_INSERT_DEPTH)))

    # --- motor pads + hole patterns ---
    pad_t = P.MOTOR_PAD_T
    for sx in (-1, 1):
        for sy in (-1, 1):
            x = sx * P.AXLE_X
            y_in = sy * (HW - t)                 # inner wall face
            y0, y1 = sorted((y_in, y_in - sy * pad_t))
            adds.append(box(x - 24, x + 24, y0, y1, zf, min(ztop, MOTOR_Z + 26)))
            n = (0, sy, 0)
            y_start = sy * (HW - t - pad_t - 0.1)
            depth = t + pad_t + 0.2
            # boss / shaft hole (centred on the shaft = axle)
            cuts.append(cyl(P.MOTOR_BOSS_D + 0.8, depth, (x, y_start, P.AXLE_Z), n))
            # 6 x M3 on the gearbox pitch circle
            for k in range(6):
                dx, dz = polar(P.MOTOR_PCD / 2, 30 + 60 * k)
                cuts.append(cyl(P.M3_CLEAR, depth, (x + dx, y_start, MOTOR_Z + dz), n))

    # --- electronics plate bosses ---
    for (x, y) in P.EPLATE_HOLES:
        adds.append(cyl(10.0, P.EPLATE_BOSS_H, (x, y, zf)))
        cuts.append(cyl(P.M3_INSERT, P.M3_INSERT_DEPTH, (x, y, zf + P.EPLATE_BOSS_H - P.M3_INSERT_DEPTH + 0.01)))
    adds.append(cyl(10.0, P.EPLATE_BOSS_H, (33, 0, zf)))        # centre support, no screw

    # --- battery bay: locating rails, end stop and clamp bosses ---
    bx0, bx1 = P.BATT_X - P.BATT_L / 2 - 1, P.BATT_X + P.BATT_L / 2 + 1
    for sy in (-1, 1):
        y = sy * (P.BATT_W / 2 + 1.0)
        y0, y1 = sorted((y, y + sy * 2.4))
        adds.append(box(bx0 + 20, bx1 - 20, y0, y1, zf, zf + 10))
        adds.append(cyl(10.0, 8.0, (P.BATT_CLAMP_X, sy * 34.0, zf)))
        cuts.append(cyl(P.M3_INSERT, P.M3_INSERT_DEPTH, (P.BATT_CLAMP_X, sy * 34.0, zf + 8 - P.M3_INSERT_DEPTH + 0.01)))
    adds.append(box(bx1, bx1 + 2.4, -18, 18, zf, zf + 10))

    # --- front bumper bosses (thicken the wall for inserts) ---
    for sy in (-1, 1):
        adds.append(box(HL - t - 4, HL - t, sy * 90 - 7, sy * 90 + 7, Z0 + 30, Z0 + 50))
        cuts.append(cyl(P.M3_INSERT, 7.5, (HL + 0.01, sy * 90, Z0 + 40), (-1, 0, 0)))
    # HC-SR04 cable slot in the front wall
    cuts.append(box(HL - t - 0.1, HL + 0.1, -8, 8, Z0 + 46, Z0 + 54))

    # --- rear panel: e-stop, power switch, charge jack ---
    cuts.append(cyl(16.2, t + 0.2, (-HL - 0.1, -60, Z0 + 50), (1, 0, 0)))
    cuts.append(cyl(12.2, t + 0.2, (-HL - 0.1, 0, Z0 + 50), (1, 0, 0)))
    cuts.append(cyl(12.2, t + 0.2, (-HL - 0.1, 60, Z0 + 50), (1, 0, 0)))

    # --- drain holes ---
    for (x, y) in [(165, 95), (165, -95), (-165, 95), (-165, -95)]:
        cuts.append(cyl(4.0, P.TRAY_FLOOR + 0.2, (x, y, Z0 - 0.1)))

    body = body.union(union_all(adds)).cut(union_all(cuts))
    # keep everything inside the outer envelope (posts / pads near the corners)
    return body.intersect(outer)


def top_deck():
    """Lid of the tray; carries the arm, basket, mast, bottle and pump."""
    deck = _rounded_plate(P.TRAY_L, P.TRAY_W, P.TRAY_CORNER_R, P.DECK_Z0, P.DECK_T)
    z, t = P.DECK_Z0 - 0.1, P.DECK_T + 0.2
    cuts = []
    for (x, y) in P.POSTS:
        cuts.append(cyl(P.M3_CLEAR, t, (x, y, z)))
    # yaw servo drops through the deck, tabs on top
    cuts.append(servo_pocket(frame((P.YAW_X, 0, P.DECK_TOP), (-1, 0, 0), (0, 0, 1)),
                             screw_d=P.M3_CLEAR, screw_depth=P.DECK_T + 0.2))
    # race ring bolts
    for k in range(6):
        x, y = polar(P.RING_BOLT_R, 30 + 60 * k, P.YAW_X, 0)
        cuts.append(cyl(P.M3_CLEAR, t, (x, y, z)))
    # arm cable grommet
    cuts.append(cyl(P.CABLE_HOLE[2], t, (P.CABLE_HOLE[0], P.CABLE_HOLE[1], z)))
    # accessories
    for (x, y) in basket_holes() + mast_holes() + bottle_holes() + pump_holes():
        cuts.append(cyl(P.M3_CLEAR, t, (x, y, z)))
    # pump hose / wire pass-through and camera cable hole
    cuts.append(cyl(10.0, t, (P.PUMP_X + 32, P.PUMP_Y, z)))
    cuts.append(cyl(10.0, t, (P.MAST_X + 2, P.MAST_Y + 34, z)))
    return deck.cut(union_all(cuts))


def electronics_plate():
    """Perforated board (10 mm M3 grid) for the ESP32, drivers and regulators."""
    x0, x1, hw = P.EPLATE_X0, P.EPLATE_X1, P.EPLATE_W / 2
    z = Z0 + P.TRAY_FLOOR + P.EPLATE_BOSS_H
    plate = rbox(x0, x1, -hw, hw, z, z + P.EPLATE_T, 5.0)
    cx = (x0 + x1) / 2
    pts = []
    nx = int((x1 - x0 - 12) // P.GRID_PITCH)
    ny = int((2 * hw - 12) // P.GRID_PITCH)
    mount = set(P.EPLATE_HOLES)
    for i in range(nx + 1):
        for j in range(ny + 1):
            x = cx + (i - nx / 2) * P.GRID_PITCH
            y = (j - ny / 2) * P.GRID_PITCH
            if any(abs(x - mx) < 7 and abs(y - my) < 7 for (mx, my) in mount):
                continue
            pts.append((x, y))
    plate = (plate.faces(">Z").workplane(origin=(0, 0, 0)).pushPoints(pts).hole(P.M3_CLEAR))
    cuts = [cyl(P.M3_CLEAR, P.EPLATE_T + 0.2, (mx, my, z - 0.1)) for (mx, my) in P.EPLATE_HOLES]
    # cable slots along both long edges
    for sy in (-1, 1):
        cuts.append(rbox(cx - 30, cx + 30, sy * (hw - 9) - 3.5, sy * (hw - 9) + 3.5, z - 0.1, z + P.EPLATE_T + 0.1, 3.0))
    return plate.cut(union_all(cuts))


def battery_clamp():
    """Bridge that straps the battery pack to the tray floor."""
    zf = Z0 + P.TRAY_FLOOR
    x0, x1 = P.BATT_CLAMP_X - 8, P.BATT_CLAMP_X + 8
    top = zf + P.BATT_H + 2.0 + 4.0
    body = box(x0, x1, -41, 41, zf + 8, top)
    # tunnel for the pack (plus foam pad)
    body = body.cut(box(x0 - 1, x1 + 1, -P.BATT_W / 2 - 1.5, P.BATT_W / 2 + 1.5, zf - 1, zf + P.BATT_H + 2.0))
    # end stop lip in front of the pack
    body = body.union(box(x1, x1 + 3, -20, 20, zf + 12, top))
    for sy in (-1, 1):
        body = body.cut(cyl(P.M3_CLEAR, 60, (P.BATT_CLAMP_X, sy * 34.0, zf)))
        body = body.cut(cyl(P.M3_HEAD_D, 30, (P.BATT_CLAMP_X, sy * 34.0, zf + 8 + 6)))
    return body


def front_bumper():
    """Bumper bar that also houses the HC-SR04 ultrasonic ranger."""
    x0, x1 = HL, HL + 14.0
    zc = Z0 + 40
    body = rbox(x0, x1, -112, 112, Z0 + 20, Z0 + 60, 5.0)
    # chamfer the leading edges so it rides over stalks
    body = body.faces(">X").edges("|Y").chamfer(4.0)
    cuts = []
    for sy in (-1, 1):
        cuts.append(cyl(16.4, 20, (x0 - 1, sy * 13.0, zc), (1, 0, 0)))          # transducers
        cuts.append(cyl(P.M3_CLEAR, 20, (x0 - 1, sy * 90, zc), (1, 0, 0)))
        cuts.append(cyl(P.M3_HEAD_D, 12, (x0 + 4, sy * 90, zc), (1, 0, 0)))     # counterbore
    cuts.append(box(x0 - 0.1, x0 + 2.2, -23, 23, zc - 10.5, zc + 10.5))        # PCB pocket
    cuts.append(box(x0 - 0.1, x0 + 6, -8, 8, zc + 5, zc + 14.5))                # header / wires
    # lightening pockets from the back
    for sy in (-1, 1):
        cuts.append(rbox(x0 - 0.1, x0 + 9, sy * 55 - 20, sy * 55 + 20, zc - 14, zc + 14, 3.0))
    return body.cut(union_all(cuts))


# ==========================================================================
# WHEELS   (wheel frame: +Y outboard, origin at the wheel centre)
# ==========================================================================
def wheel_rim():
    """PETG rim; a 12 mm hex coupler on the motor shaft drives it."""
    hw = P.WHEEL_W / 2
    # barrel
    rim = cyl(P.RIM_D, P.WHEEL_W, (0, -hw, 0), (0, 1, 0))
    rim = rim.cut(cyl(P.RIM_D - 8, P.WHEEL_W - 5, (0, -hw - 0.01, 0), (0, 1, 0)))
    # bead flanges (inboard one is chamfered so it prints without support)
    rim = rim.union(cyl(P.RIM_FLANGE_D, P.RIM_FLANGE_W, (0, hw - P.RIM_FLANGE_W, 0), (0, 1, 0)))
    rim = rim.union(cone(P.RIM_FLANGE_D, P.RIM_D, 2.0, (0, -hw + P.RIM_FLANGE_W, 0), (0, 1, 0)))
    rim = rim.union(cyl(P.RIM_FLANGE_D, P.RIM_FLANGE_W, (0, -hw, 0), (0, 1, 0)))
    # hub boss reaching inboard to the coupler
    boss_in = -hw - 4.0
    rim = rim.union(cyl(30.0, hw - boss_in, (0, boss_in, 0), (0, 1, 0)))
    # spokes: 5 ribs from hub to barrel
    for k in range(5):
        rib = box(10, P.RIM_D / 2 - 2, -hw + 4, hw - 5, -2.5, 2.5).rotate((0, 0, 0), (0, 1, 0), 72 * k)
        rim = rim.union(rib)
    cuts = []
    # hex socket for the coupler
    hexs = (cq.Workplane("XZ").workplane(offset=-boss_in)
            .polygon(6, (P.COUPLER_AF + 0.3) / cos(pi / 6)).extrude(-17.5))
    cuts.append(hexs)
    cuts.append(cyl(P.M4_CLEAR, P.WHEEL_W + 10, (0, boss_in, 0), (0, 1, 0)))
    cuts.append(cyl(9.5, 6.0, (0, hw - 6.0 + 0.01, 0), (0, 1, 0)))             # M4 head + washer
    # lightening holes in the outboard web
    for k in range(5):
        x, z = polar(38.0, 36 + 72 * k)
        cuts.append(cyl(24.0, 6, (x, hw - 5.5, z), (0, 1, 0)))
    return rim.cut(union_all(cuts))


def wheel_tyre():
    """TPU tyre with chevron lugs for loose soil."""
    w = P.WHEEL_W - 2 * P.RIM_FLANGE_W - 0.4
    tyre = cyl(P.TYRE_BASE_D, w, (0, -w / 2, 0), (0, 1, 0))
    tyre = tyre.cut(cyl(P.RIM_D + 0.4, w + 1, (0, -w / 2 - 0.5, 0), (0, 1, 0)))
    tyre = tyre.cut(cone(P.RIM_FLANGE_D + 0.6, P.RIM_D + 0.4, 2.2, (0, -w / 2 - 0.2, 0), (0, 1, 0)))
    lug_h = (P.WHEEL_D - P.TYRE_BASE_D) / 2 + 1.5
    r_mid = P.TYRE_BASE_D / 2 - 1.5 + lug_h / 2
    lugs = []
    for k in range(P.TYRE_LUGS):
        for side in (-1, 1):
            lug = (cq.Workplane("XY").box(9.0, w / 2 + 3.0, lug_h)
                   .rotate((0, 0, 0), (0, 0, 1), side * 28.0)
                   .translate((side * 0.0, side * (w / 4 - 0.5), r_mid)))
            ang = 360.0 / P.TYRE_LUGS * (k + (0.25 if side > 0 else -0.25) * 0.0)
            lugs.append(lug.rotate((0, 0, 0), (0, 1, 0), ang))
    tyre = tyre.union(union_all(lugs))
    # trim lugs to the tyre width and the outer diameter
    env = cyl(P.WHEEL_D, w, (0, -w / 2, 0), (0, 1, 0))
    return tyre.intersect(env)


# ==========================================================================
# ARM BASE
# ==========================================================================
def race_ring():
    """Lower ball race bolted to the deck around the yaw servo."""
    z = P.DECK_TOP
    ring = cyl(2 * P.RING_RO, P.RING_H, (P.YAW_X, 0, z))
    ring = ring.cut(cyl(2 * P.RING_RI, P.RING_H + 1, (P.YAW_X, 0, z - 0.5)))
    groove = (cq.Workplane("XZ")
              .polyline([(P.RACE_R - P.GROOVE_DEPTH, P.RING_H + 0.01),
                         (P.RACE_R, P.RING_H - P.GROOVE_DEPTH),
                         (P.RACE_R + P.GROOVE_DEPTH, P.RING_H + 0.01)]).close()
              .revolve(360, (0, 0, 0), (0, 1, 0))
              .translate((P.YAW_X, 0, z)))
    ring = ring.cut(groove)
    cuts = []
    for k in range(6):
        x, y = polar(P.RING_BOLT_R, 30 + 60 * k, P.YAW_X, 0)
        cuts.append(cyl(P.M3_CLEAR, P.RING_H + 1, (x, y, z - 0.5)))
    # servo lead exit
    cuts.append(box(P.YAW_X - P.RING_RO - 1, P.YAW_X - P.RING_RI + 1, -5, 5, z - 0.1, z + 6))
    return ring.cut(union_all(cuts))


def turntable_clip():
    """Hook that stops the turntable lifting off its balls (3 used).
    Modelled at the +X position on the ring; instanced at 30, 150, 270 deg."""
    z0 = P.DECK_TOP + P.RING_H
    lip_z = P.TT_TOP + 0.6
    top = lip_z + 3.0
    a = 13.0
    prof_body = cyl(2 * P.RING_RO, top - z0, (0, 0, z0)).cut(cyl(2 * (P.TT_R + 1.0), top - z0 + 1, (0, 0, z0 - 0.5)))
    lip = cyl(2 * P.RING_RO, 3.0, (0, 0, lip_z)).cut(cyl(2 * (P.TT_R - 5.0), 4, (0, 0, lip_z - 0.5)))
    wedge = (cq.Workplane("XY").polyline([(0, 0), polar(100, -a), polar(100, a)]).close()
             .extrude(60).translate((0, 0, z0 - 1)))
    clip = prof_body.union(lip).intersect(wedge)
    clip = clip.cut(cyl(P.M3_CLEAR, 40, (P.RING_BOLT_R, 0, z0 - 1)))
    return clip.translate((P.YAW_X, 0, 0))


def turntable():
    """Rotating disc with upper ball race and the shoulder-servo tower.
    Frame: origin on the yaw axis at the disc underside."""
    disc = cyl(2 * P.TT_R, P.TT_T)
    groove = (cq.Workplane("XZ")
              .polyline([(P.RACE_R - P.GROOVE_DEPTH, -0.01),
                         (P.RACE_R, P.GROOVE_DEPTH),
                         (P.RACE_R + P.GROOVE_DEPTH, -0.01)]).close()
              .revolve(360, (0, 0, 0), (0, 1, 0)))
    disc = disc.cut(groove)
    disc = disc.cut(horn_holes(frame((0, 0, 0), (1, 0, 0), (0, 0, 1)), P.TT_T, rot=45.0))
    # counterbores for the horn screw heads
    for i in range(4):
        x, y = polar(P.HORN_PCD / 2, 45 + 90 * i)
        disc = disc.cut(cyl(P.M3_HEAD_D, 3.0, (x, y, P.TT_T - 2.4)))

    sx = P.SHOULDER_X
    zs = P.TT_T + P.SHOULDER_H                       # shoulder axis height in this frame
    body_c = zs - P.SERVO_AXIS_OFF                   # servo case centre
    z_top = body_c + P.SERVO_TAB_L / 2 + 3.0
    hx = P.BOX_HALF_H
    tower = rbox(sx - hx, sx + hx, P.BOX_Y0, P.BOX_Y1, P.TT_T - 0.01, z_top, 2.0)
    # gussets tying the tower to the disc
    for s in (-1, 1):
        g = (cq.Workplane("XZ").polyline([(0, 0), (8, 0), (0, 22)]).close().extrude(4.0)
             .translate((0, 2.0, 0)))
        if s < 0:
            g = g.mirror("YZ")
        tower = tower.union(g.translate((sx + s * hx, -8.5, P.TT_T)))
    part = disc.union(tower)
    # shoulder servo: output along +Y, case hanging down from the axis
    spl = frame((sx, P.BOX_Y1, zs), (0, 0, -1), (0, 1, 0))
    part = part.cut(servo_pocket(spl, through=P.SERVO_BELOW + 0.6))
    # hollow for the lead at the bottom of the case + exit slot towards the yaw axis
    part = part.cut(box(sx - hx - 1, sx + 4, P.BOX_Y0 + 4.5, P.BOX_Y0 + 14.0, P.TT_T + 2.0, body_c - P.SERVO_BODY_L / 2 + 6))
    # idler journal on the -Y face
    jpl = frame((sx, P.BOX_Y0, zs), (1, 0, 0), (0, -1, 0))
    part = part.union(journal_boss(jpl, P.PLATE_T)).cut(journal_insert(jpl, P.PLATE_T))
    return part


# ==========================================================================
# UPPER ARM (two flat plates + brace).  Frame: shoulder axis, +X to elbow
# ==========================================================================
def _stadium(length, width, y0, y1):
    wp = (cq.Workplane("XZ").workplane(offset=-y1)
          .center(length / 2, 0).slot2D(length + width, width).extrude(y1 - y0))
    return wp


def _arm_plate_windows(y0, y1):
    cuts = []
    for (xa, xb) in ((34, 68), (132, 166)):
        cuts.append((cq.Workplane("XZ").workplane(offset=-y1 - 0.1)
                     .center((xa + xb) / 2, 0).slot2D(xb - xa, 16).extrude(y1 - y0 + 0.2)))
    for dx in (-12, 12):
        cuts.append(cyl(P.M3_CLEAR, y1 - y0 + 0.2, (P.L1 / 2 + dx, y0 - 0.1, 0), (0, 1, 0)))
    return cuts


def upper_arm_drive_plate():
    """+Y plate: bolted to the shoulder horn and to the elbow horn."""
    y0, y1 = P.PLATE_IN, P.PLATE_OUT
    plate = _stadium(P.L1, P.ARM_PLATE_W, y0, y1)
    cuts = _arm_plate_windows(y0, y1)
    for x in (0, P.L1):
        cuts.append(horn_holes(frame((x, y0, 0), (1, 0, 0), (0, 1, 0)), P.PLATE_T))
    return plate.cut(union_all(cuts))


def upper_arm_idler_plate():
    """-Y plate: rides on the printed journals of the tower and the forearm."""
    y0, y1 = -P.PLATE_OUT, -P.PLATE_IN
    plate = _stadium(P.L1, P.ARM_PLATE_W, y0, y1)
    cuts = _arm_plate_windows(y0, y1)
    for x in (0, P.L1):
        cuts.append(cyl(P.JOURNAL_D + P.JOURNAL_CLEAR, P.PLATE_T + 0.2, (x, y0 - 0.1, 0), (0, 1, 0)))
    return plate.cut(union_all(cuts))


def upper_arm_brace():
    """Box spacer between the two plates; doubles as a cable duct."""
    xc = P.L1 / 2
    body = box(xc - 20, xc + 20, -P.PLATE_IN, P.PLATE_IN, -13, 13)
    body = body.cut(box(xc - 21, xc + 21, -P.PLATE_IN + 7.5, P.PLATE_IN - 7.5, -10, 10))
    for sy in (-1, 1):
        for dx in (-12, 12):
            body = body.cut(cyl(P.M3_INSERT, P.M3_INSERT_DEPTH, (xc + dx, sy * P.PLATE_IN, 0), (0, -sy, 0)))
    return body


# ==========================================================================
# FOREARM.  Frame: elbow axis, +X to wrist
# ==========================================================================
FA_X0 = -(P.SERVO_TAB_L / 2 - P.SERVO_AXIS_OFF) - 4.5         # -21.5
FA_X1 = P.L2 + (P.SERVO_TAB_L / 2 - P.SERVO_AXIS_OFF) + 4.5   # 201.5


def forearm():
    """Box beam that carries the elbow servo (near end) and wrist servo (far end)."""
    h = P.BOX_HALF_H
    body = rbox(FA_X0, FA_X1, P.BOX_Y0, P.BOX_Y1, -h, h, 2.0)
    # hollow core between / under the two servos
    body = body.cut(box(FA_X0 + 5, FA_X1 - 5, P.BOX_Y0 + 4.5, P.BOX_Y1 - P.SERVO_WALL, -h + 5, h - 5))
    # elbow servo: case runs towards the wrist;  wrist servo: case runs back to the elbow
    body = body.cut(servo_pocket(frame((0, P.BOX_Y1, 0), (1, 0, 0), (0, 1, 0)), through=P.SERVO_BELOW + 0.6))
    body = body.cut(servo_pocket(frame((P.L2, P.BOX_Y1, 0), (-1, 0, 0), (0, 1, 0)), through=P.SERVO_BELOW + 0.6))
    # cable slot in the top wall
    yc = (P.BOX_Y0 + P.BOX_Y1) / 2
    body = body.cut(rbox(48, P.L2 - 48, yc - 6, yc + 6, h - 6, h + 1, 5.0))
    # weight relief in the -Y wall between the journals
    body = body.cut((cq.Workplane("XZ").workplane(offset=-P.BOX_Y0 - 5)
                     .center(P.L2 / 2, 0).slot2D(70, 14).extrude(6)))
    for x in (0, P.L2):
        jpl = frame((x, P.BOX_Y0, 0), (1, 0, 0), (0, -1, 0))
        body = body.union(journal_boss(jpl, P.PLATE_T)).cut(journal_insert(jpl, P.PLATE_T))
    return body


# ==========================================================================
# WRIST + GRIPPER.  Tool frame: wrist axis, +X along gripper
# ==========================================================================
WEB_HOLES = [(sy * 22.0, sz * 12.0) for sy in (-1, 1) for sz in (-1, 1)]
CLIP_HOLES = [(22.0, 13.0), (22.0, -13.0)]          # (x, z) on the fork arms


def _x_window(x0, x1, width=18.0, height=12.0):
    """Cutter: rounded cable window through a wall that faces +X."""
    return (cq.Workplane("YZ").workplane(offset=x0).slot2D(width, height).extrude(x1 - x0))


def wrist_fork():
    """U bracket: +Y arm bolts to the wrist horn, -Y arm rides on the journal."""
    hw = P.WRIST_ARM_W / 2
    x1 = P.WRIST_WEB_X + P.PLATE_T
    parts = []
    for (y0, y1) in ((P.PLATE_IN, P.PLATE_OUT), (-P.PLATE_OUT, -P.PLATE_IN)):
        arm = box(0, x1, y0, y1, -hw, hw).union(cyl(2 * hw, y1 - y0, (0, y0, 0), (0, 1, 0)))
        parts.append(arm)
    parts.append(box(P.WRIST_WEB_X, x1, -P.PLATE_OUT, P.PLATE_OUT, -hw, hw))
    fork = union_all(parts)
    cuts = [horn_holes(frame((0, P.PLATE_IN, 0), (1, 0, 0), (0, 1, 0)), P.PLATE_T),
            cyl(P.JOURNAL_D + P.JOURNAL_CLEAR, P.PLATE_T + 0.2, (0, -P.PLATE_OUT - 0.1, 0), (0, 1, 0))]
    for (y, z) in WEB_HOLES:
        cuts.append(cyl(P.M3_CLEAR, P.PLATE_T + 0.2, (P.WRIST_WEB_X - 0.1, y, z), (1, 0, 0)))
    # heat-set inserts for the tool clips (a through-bolt would foul the forearm)
    for (x, z) in CLIP_HOLES:
        cuts.append(cyl(P.M3_INSERT, 2 * P.PLATE_OUT + 1, (x, -P.PLATE_OUT - 0.5, z), (0, 1, 0)))
    # cable window in the web
    cuts.append(_x_window(P.WRIST_WEB_X - 0.1, x1 + 0.1))
    return fork.cut(union_all(cuts))


PALM_X0 = P.WRIST_WEB_X + P.PLATE_T          # 36
PALM_X1 = P.GRIP_X + 24.0                    # 104
PALM_T = 5.0


def gripper_palm():
    """Carries the gripper servo and the idler-finger pivot; bolts to the fork web."""
    zt = P.PALM_TOP_Z                         # top of the horizontal plate (-22)
    flange = box(PALM_X0, PALM_X0 + 5, -P.PLATE_OUT, P.PLATE_OUT, zt - PALM_T, P.WRIST_ARM_W / 2)
    plate = rbox(PALM_X0, PALM_X1, -30, 30, zt - PALM_T, zt, 4.0)
    part = flange.union(plate)
    # gussets
    for y in (-30, 26):
        g = (cq.Workplane("XZ").polyline([(0, 0), (30, 0), (0, 30)]).close().extrude(-4.0)
             .translate((PALM_X0 + 5, y, zt)))
        part = part.union(g)
    # idler pivot pillar + pin
    px, py = P.GRIP_X, -P.GRIP_DY
    part = part.union(cyl(15.0, (-P.FINGER_T / 2 - 0.5) - zt + 0.01, (px, py, zt - 0.01)))
    part = part.union(cyl(8.0, P.FINGER_T + 1.0 + 0.5, (px, py, -P.FINGER_T / 2 - 1.0)))
    cuts = [cyl(P.M3_INSERT, P.M3_INSERT_DEPTH, (px, py, P.FINGER_T / 2 + 0.5 - P.M3_INSERT_DEPTH + 0.01))]
    # gripper servo, output up (+Z), case running back towards the wrist
    cuts.append(servo_pocket(frame((P.GRIP_X, P.GRIP_DY, zt), (-1, 0, 0), (0, 0, 1)),
                             screw_d=P.M3_SELF_TAP, screw_depth=PALM_T + 0.2, through=PALM_T + 1))
    for (y, z) in WEB_HOLES:
        cuts.append(cyl(P.M3_CLEAR, 6, (PALM_X0 - 0.5, y, z), (1, 0, 0)))
    cuts.append(_x_window(PALM_X0 - 0.1, PALM_X0 + 5.1))
    return part.cut(union_all(cuts))


def _finger_blade(sign):
    """Finger outline in the closed pose; sign=+1 for the pivot at +Y."""
    yi = -(P.PAD_FACE - P.PAD_T)          # blade face the pad sits on
    pts = [(-4, 11), (24, 10.5), (44, 6.0), (70, 1.5), (70, yi),
           (31, yi), (15, -5.5), (0, -6)]
    if sign < 0:
        pts = [(x, -y) for (x, y) in reversed(pts)]
    blade = (cq.Workplane("XY").workplane(offset=-P.FINGER_T / 2)
             .polyline(pts).close().extrude(P.FINGER_T))
    return blade


def _finger(sign, phase):
    gear = spur_gear(P.GRIP_GEAR_MODULE, P.GRIP_GEAR_TEETH, P.FINGER_T, phase_deg=phase).translate((0, 0, -P.FINGER_T / 2))
    return gear.union(_finger_blade(sign))


def finger_pad():
    """Soft TPU pad that clips over the finger blade.  The shallow spherical
    dish cradles round produce so it cannot creep out of the grip.
    Modelled on the drive finger (pivot at +Y); the idle finger uses the same
    part turned over."""
    yf = -P.PAD_FACE                       # working face
    yb = yf + P.PAD_T                      # blade face
    hz = P.PAD_H / 2
    pad = box(P.PAD_X0, P.PAD_X1, yf, yb + 6.0, -hz, hz)
    pad = pad.edges("|X").fillet(1.5)
    # channel that grips the 8 mm blade
    pad = pad.cut(box(P.PAD_X0 - 1, P.PAD_X1 + 1, yb, yb + 7.0, -P.FINGER_T / 2 - 0.1, P.FINGER_T / 2 + 0.1))
    xc = (P.PAD_X0 + P.PAD_X1) / 2
    dish = W(cq.Solid.makeSphere(P.PAD_DISH_R, cq.Vector(xc, yf - P.PAD_DISH_R + P.PAD_DISH_DEPTH, 0)))
    return pad.cut(dish)


def finger_drive():
    """Finger bolted to the gripper-servo horn (pivot at +Y)."""
    pitch = 360.0 / P.GRIP_GEAR_TEETH
    f = _finger(+1, phase=-90.0)              # a tooth points at the other gear
    f = f.cut(horn_holes(frame((0, 0, -P.FINGER_T / 2), (1, 0, 0), (0, 0, 1)), P.FINGER_T, centre_d=8.5))
    return f


def finger_idle():
    """Mirror finger; free on the palm pin, driven through the gear mesh."""
    pitch = 360.0 / P.GRIP_GEAR_TEETH
    f = _finger(-1, phase=90.0 + pitch / 2)   # a gap points at the other gear
    f = f.cut(cyl(8.35, P.FINGER_T + 1, (0, 0, -P.FINGER_T / 2 - 0.5)))
    return f


def probe_clip():
    """Holds the capacitive soil-moisture blade on the -Y fork arm."""
    yo = -P.PLATE_OUT
    back = rbox(8, 36, yo - 4, yo, -52, 19, 0.01)
    block = box(8, 36, yo - 10.5, yo, -52, -22)
    part = back.union(block)
    # slot for the 23 mm wide, 1.6 mm thick PCB (clamped by a zip tie / M3 screw)
    part = part.cut(box(10.3, 33.7, yo - 7.2, yo - 5.0, -53, -21))
    part = part.cut(box(14, 30, yo - 11, yo - 5.0, -53, -21))                # open face, 2 mm lips
    part = part.cut(box(7, 37, yo - 11, yo - 9.0, -39, -35))                 # zip-tie groove
    part = part.cut(box(7, 9.5, yo - 11, yo + 1, -39, -35)).cut(box(34.5, 37, yo - 11, yo + 1, -39, -35))
    for (x, z) in CLIP_HOLES:
        part = part.cut(cyl(P.M3_CLEAR, 12, (x, yo - 11, z), (0, 1, 0)))
        part = part.cut(cyl(P.M3_HEAD_D, 9, (x, yo - 12.5, z), (0, 1, 0)))
    return part


def nozzle_clip():
    """Holds the misting nozzle (10 mm body) on the +Y fork arm."""
    yo = P.PLATE_OUT
    back = box(8, 36, yo, yo + 4, -40, 19)
    ring_c = (22.0, yo + 12.0)
    ring = cyl(18.0, 14.0, (ring_c[0], ring_c[1], -40))
    web = box(15, 29, yo, ring_c[1], -40, -26)
    part = back.union(ring).union(web)
    part = part.cut(cyl(10.3, 20, (ring_c[0], ring_c[1], -43)))
    part = part.cut(box(ring_c[0] - 1.0, ring_c[0] + 1.0, ring_c[1], ring_c[1] + 12, -43, -20))   # clamp slit
    for (x, z) in CLIP_HOLES:
        part = part.cut(cyl(P.M3_CLEAR, 12, (x, yo - 1, z), (0, 1, 0)))
    return part


# ==========================================================================
# DECK ACCESSORIES (chassis frame)
# ==========================================================================
def basket_holes():
    x, y = P.BASKET_X, P.BASKET_Y
    return [(x + sx * (P.BASKET_L / 2 - 14), y + sy * (P.BASKET_W / 2 - 14)) for sx in (-1, 1) for sy in (-1, 1)]


def harvest_basket():
    """Open crate on the rear deck for picked produce."""
    z = P.DECK_TOP
    x0, x1 = P.BASKET_X - P.BASKET_L / 2, P.BASKET_X + P.BASKET_L / 2
    y0, y1 = P.BASKET_Y - P.BASKET_W / 2, P.BASKET_Y + P.BASKET_W / 2
    outer = rbox(x0, x1, y0, y1, z, z + P.BASKET_H, 10.0)
    inner = rbox(x0 + 2.4, x1 - 2.4, y0 + 2.4, y1 - 2.4, z + 3.0, z + P.BASKET_H + 1, 7.6)
    part = outer.cut(inner)
    cuts = []
    for (hx, hy) in basket_holes():
        cuts.append(cyl(P.M3_CLEAR, 5, (hx, hy, z - 1)))
    # drain / dirt slots in the floor
    for i in range(-2, 3):
        cuts.append(rbox(P.BASKET_X - 30, P.BASKET_X + 30, P.BASKET_Y + i * 22 - 3, P.BASKET_Y + i * 22 + 3, z - 1, z + 4, 2.9))
    # ventilation slots in the long walls
    for sx in (-1, 1):
        for i in range(-2, 3):
            xs = x0 - 1 if sx < 0 else x1 - 4
            cuts.append(box(xs, xs + 5, P.BASKET_Y + i * 26 - 4, P.BASKET_Y + i * 26 + 4, z + 14, z + P.BASKET_H - 12))
    return part.cut(union_all(cuts))


CAM_TILT = 28.0        # degrees nose-down
CAM_HINGE_DX = 12.0    # hinge axis ahead of the mast centre line
CAM_BOARD_DX = 7.0     # board cradle stands off the hinge by this much


def mast_holes():
    x, y = P.MAST_X, P.MAST_Y
    return [(x + sx * 15, y + sy * 15) for sx in (-1, 1) for sy in (-1, 1)]


def camera_mast():
    """Post for the ESP32-CAM overlooking the arm's workspace."""
    z = P.DECK_TOP
    x, y = P.MAST_X, P.MAST_Y
    flange = rbox(x - 22, x + 22, y - 22, y + 22, z, z + 5, 5.0)
    post = box(x - 22, x - 6, y - 10, y + 10, z, z + P.MAST_H)
    rib = (cq.Workplane("XZ").polyline([(0, 0), (26, 0), (0, 40)]).close().extrude(-5.0)
           .translate((x - 6, y - 2.5, z + 5)))
    part = flange.union(post).union(rib)
    # cable channel up the front of the post
    part = part.cut(box(x - 11, x - 5.9, y - 4, y + 4, z + 50, z + P.MAST_H - 24))
    # hinge ears at the top (hinge axis CAM_HINGE_DX ahead of the mast centre)
    top = z + P.MAST_H
    hx0 = x + CAM_HINGE_DX
    for sy in (-1, 1):
        ya, yb = sorted((y + sy * 6.2, y + sy * 10.0))
        ear = box(x - 22, hx0, ya, yb, top - 20, top).union(cyl(20.0, yb - ya, (hx0, ya, top - 10), (0, 1, 0)))
        part = part.union(ear)
    part = part.cut(cyl(P.M3_CLEAR, 30, (hx0, y - 15, top - 10), (0, 1, 0)))
    for (hx, hy) in mast_holes():
        part = part.cut(cyl(P.M3_CLEAR, 7, (hx, hy, z - 1)))
    return part




def camera_holder():
    """Tilting cradle for an ESP32-CAM (27 x 40.5 mm board).
    Modelled around the hinge axis (origin), camera looking along +X, then
    placed on the mast by the assembly."""
    # back plate in the YZ plane, board sits on its +X face
    plate = box(6, 9, -16, 16, -30, 16)
    lips = union_all([
        box(9, 12.5, -16, -13.7, -30, 16), box(9, 12.5, 13.7, 16, -30, 16),      # side rails
        box(11.0, 12.5, -16, -12.6, -30, 16), box(11.0, 12.5, 12.6, 16, -30, 16),  # retaining lips
        box(9, 12.5, -16, 16, -30, -27.5),                                         # bottom stop
    ])
    cradle = plate.union(lips)
    # windows for the header pins on the back of the board
    for sy in (-1, 1):
        cradle = cradle.cut(box(5, 10, sy * 11.4 - 1.6, sy * 11.4 + 1.6, -26, 13))
    cradle = cradle.translate((CAM_BOARD_DX, 0, 0))
    ear = box(-6, 6 + CAM_BOARD_DX + 0.5, -6, 6, -8, 8).union(cyl(16.0, 12.0, (0, -6, 0), (0, 1, 0)))
    part = cradle.union(ear)
    part = part.cut(cyl(P.M3_CLEAR, 14, (0, -7, 0), (0, 1, 0)))
    return part


def bottle_holes():
    x, y = P.BOTTLE_X, P.BOTTLE_Y
    return [(x, y + 20), (x, y - 20)]


def bottle_holder():
    """Cup + ring for a 500 ml PET bottle used as the spray tank."""
    z = P.DECK_TOP
    x, y, d = P.BOTTLE_X, P.BOTTLE_Y, P.BOTTLE_D
    od = d + 6.0
    cup = cyl(od, 45.0, (x, y, z)).cut(cyl(d + 1.0, 45.0, (x, y, z + 3.0)))
    ring = cyl(od, 12.0, (x, y, z + 108)).cut(cyl(d + 1.0, 14.0, (x, y, z + 107)))
    part = cup.union(ring)
    for a in (0, 120, 240):
        sx, sy = polar(od / 2 - 1.5, a + 90, x, y)
        strut = box(-5, 5, -2.0, 2.0, z + 40, z + 110).rotate((0, 0, 0), (0, 0, 1), a).translate((sx, sy, 0))
        part = part.union(strut)
    part = part.cut(cyl(d + 1.0, 200, (x, y, z + 3.0)))
    for (hx, hy) in bottle_holes():
        part = part.cut(cyl(P.M3_CLEAR, 5, (hx, hy, z - 1)))
        part = part.cut(cyl(P.M3_HEAD_D + 0.5, 3, (hx, hy, z + 1.6)))
    # drain
    part = part.cut(cyl(5.0, 5, (x + 26, y, z - 1)))
    return part


def pump_holes():
    x, y = P.PUMP_X, P.PUMP_Y
    return [(x - 22, y), (x + 22, y)]


def pump_saddle():
    """Cradle + strap slots for an R385 diaphragm pump (28 mm motor can)."""
    z = P.DECK_TOP
    x, y = P.PUMP_X, P.PUMP_Y
    base = rbox(x - 29, x + 29, y - 30, y + 30, z, z + 4, 4.0)
    cradle = box(x - 16, x + 16, y - 30, y + 18, z, z + 14)
    cradle = cradle.cut(cyl(28.6, 70, (x, y - 35, z + 4 + 14.3), (0, 1, 0)))
    part = base.union(cradle)
    for (hx, hy) in pump_holes():
        part = part.cut(cyl(P.M3_CLEAR, 6, (hx, hy, z - 1)))
    for sy in (-1, 1):       # zip-tie slots
        part = part.cut(box(x - 20, x - 16.5, y + sy * 16 - 2.5, y + sy * 16 + 2.5, z - 1, z + 5))
        part = part.cut(box(x + 16.5, x + 20, y + sy * 16 - 2.5, y + sy * 16 + 2.5, z - 1, z + 5))
    return part
