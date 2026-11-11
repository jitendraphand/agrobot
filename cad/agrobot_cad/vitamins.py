"""Bought-in components ("vitamins"), modelled as simple envelopes so that the
assembly STEP, the simulation and the Blender scenes show the complete robot.
None of these are printed."""
from math import cos, sin, radians, pi

import cadquery as cq

from . import params as P
from .helpers import W, box, cyl, cone, rbox, union_all, polar


def servo():
    """Standard-size servo with a 25T metal disc horn, in the servo frame."""
    cx = P.SERVO_AXIS_OFF
    L, Wd = P.SERVO_BODY_L, P.SERVO_BODY_W
    case = box(cx - L / 2, cx + L / 2, -Wd / 2, Wd / 2, -P.SERVO_BELOW, P.SERVO_TAB_T + P.SERVO_ABOVE)
    tabs = box(cx - P.SERVO_TAB_L / 2, cx + P.SERVO_TAB_L / 2, -Wd / 2, Wd / 2, 0, P.SERVO_TAB_T)
    top = P.SERVO_TAB_T + P.SERVO_ABOVE
    turret = cyl(13.0, 2.0, (0, 0, top))
    horn = cyl(P.HORN_D, P.HORN_T, (0, 0, P.SERVO_HORN_FACE - P.HORN_T)).union(
        cyl(9.0, P.SERVO_HORN_FACE - top - 2.0, (0, 0, top + 2.0)))
    return case.union(tabs).union(turret).union(horn)


def gearmotor():
    """JGB37-520 with encoder.  Frame: origin on the shaft axis at the mounting
    face, +Z along the shaft (outboard), +X from the shaft to the gearbox centre."""
    o = P.MOTOR_SHAFT_OFF
    gb = cyl(P.MOTOR_D, P.MOTOR_GEARBOX_L, (o, 0, -P.MOTOR_GEARBOX_L))
    can = cyl(P.MOTOR_CAN_D, P.MOTOR_TOTAL_L - P.MOTOR_GEARBOX_L - 8, (o, 0, -P.MOTOR_TOTAL_L + 8))
    enc = cyl(30.0, 8.0, (o, 0, -P.MOTOR_TOTAL_L))
    boss = cyl(P.MOTOR_BOSS_D, P.MOTOR_BOSS_H)
    shaft = cyl(P.MOTOR_SHAFT_D, P.MOTOR_SHAFT_L)
    return union_all([gb, can, enc, boss, shaft])


def hex_coupler():
    """12 mm hex wheel coupler.  Frame: wheel frame (+Y outboard)."""
    y0 = -P.WHEEL_W / 2 - 4.0 - 1.0
    c = (cq.Workplane("XZ").workplane(offset=-y0).polygon(6, P.COUPLER_AF / cos(pi / 6)).extrude(-P.COUPLER_L))
    return c.cut(cyl(P.MOTOR_SHAFT_D + 0.1, P.COUPLER_L + 30, (0, y0 - 15, 0), (0, 1, 0)))


def battery():
    x0, x1 = P.BATT_X - P.BATT_L / 2, P.BATT_X + P.BATT_L / 2
    z = P.TRAY_Z0 + P.TRAY_FLOOR + 1.0
    return rbox(x0, x1, -P.BATT_W / 2, P.BATT_W / 2, z, z + P.BATT_H, 4.0)


def bottle():
    z = P.DECK_TOP + 3.0
    x, y, d = P.BOTTLE_X, P.BOTTLE_Y, P.BOTTLE_D
    body = cyl(d, 135, (x, y, z)).union(cone(d, 28, 40, (x, y, z + 135))).union(cyl(28, 18, (x, y, z + 175)))
    return body


def bottle_cap():
    z = P.DECK_TOP + 3.0 + 191
    cap = cyl(31, 16, (P.BOTTLE_X, P.BOTTLE_Y, z - 12))
    return cap.cut(cyl(28.4, 15, (P.BOTTLE_X, P.BOTTLE_Y, z - 13)))


def pump():
    z = P.DECK_TOP + 4 + 14.3
    x, y = P.PUMP_X, P.PUMP_Y
    motor = cyl(28.0, 50, (x, y - 30, z), (0, 1, 0))
    head = box(x - 20, x + 20, y + 20, y + 45, z - 14, z + 20)
    ports = cyl(8.0, 12, (x - 10, y + 45, z + 8), (0, 1, 0)).union(cyl(8.0, 12, (x + 10, y + 45, z + 8), (0, 1, 0)))
    return motor.union(head).union(ports)


def hcsr04():
    x0 = P.TRAY_L / 2
    zc = P.TRAY_Z0 + 40
    pcb = box(x0, x0 + 1.6, -22.5, 22.5, zc - 10, zc + 10)
    cans = cyl(16.0, 12.4, (x0 + 1.6, 13, zc), (1, 0, 0)).union(cyl(16.0, 12.4, (x0 + 1.6, -13, zc), (1, 0, 0)))
    return pcb.union(cans)


def esp32cam():
    """In the camera-holder frame (board on the +X face of the back plate)."""
    pcb = box(9.2, 10.8, -13.5, 13.5, -27.3, 13.2)
    cam = box(10.8, 16.0, -4, 4, -2, 6).union(cyl(8.0, 6.0, (16.0, 0, 2), (1, 0, 0)))
    from .parts import CAM_BOARD_DX
    return pcb.union(cam).translate((CAM_BOARD_DX, 0, 0))


def soil_probe():
    """Capacitive soil moisture sensor v1.2 (98 x 23 mm).  Tool frame."""
    tx, ty, tz = P.TCP_PROBE
    blade = box(tx - 11.5, tx + 11.5, ty - 0.8, ty + 0.8, tz + 8, tz + 98)
    tip = (cq.Workplane("XZ").workplane(offset=-(ty + 0.8))
           .polyline([(tx - 11.5, tz + 8), (tx, tz), (tx + 11.5, tz + 8)]).close().extrude(1.6))
    return blade.union(tip)


def nozzle():
    tx, ty, tz = P.TCP_NOZZLE
    body = cyl(10.0, 26, (tx, ty, tz + 4)).union(cone(6.0, 10.0, 4, (tx, ty, tz)))
    barb = cyl(6.0, 12, (tx, ty, tz + 30))
    return body.union(barb)


def balls():
    r = P.BALL_D / 2
    zc = P.DECK_TOP + P.RING_H + P.race_ball_gap() / 2
    out = []
    for k in range(P.RACE_N_BALLS):
        x, y = polar(P.RACE_R, 360.0 / P.RACE_N_BALLS * k, P.YAW_X, 0)
        out.append(W(cq.Solid.makeSphere(r, cq.Vector(x, y, zc))))
    return W(cq.Compound.makeCompound([o.val() for o in out]))


def _board(x, y, l, w, h, z):
    return box(x - l / 2, x + l / 2, y - w / 2, y + w / 2, z, z + h)


def board_layout():
    """(name, x, y, length_x, width_y, height) of each module on the plate."""
    return [
        ("esp32_devkit", 60, 0, 52, 28, 14),
        ("pca9685", 70, 60, 26, 62, 12),
        ("bts7960_left", -8, 70, 50, 50, 43),
        ("bts7960_right", -8, -70, 50, 50, 43),
        ("buck_5v_lm2596", 32, -55, 21, 43, 14),
        ("buck_servo_xl4016", 65, -62, 37, 60, 22),
        ("mosfet_pump", 35, 50, 17, 34, 12),
        ("mpu6050", 10, 0, 21, 16, 5),
        ("fuse_holder", -22, 0, 20, 40, 18),
    ]


def boards():
    z = P.TRAY_Z0 + P.TRAY_FLOOR + P.EPLATE_BOSS_H + P.EPLATE_T + 6.0
    out = {}
    for (name, x, y, l, w, h) in board_layout():
        out[name] = _board(x, y, l, w, h, z)
    return out
