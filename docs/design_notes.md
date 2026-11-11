# Design notes

Why the robot is the way it is, with the numbers behind each choice. All
dimensions live in `cad/agrobot_cad/params.py`; CAD, simulation, Blender
scenes and the firmware geometry header are generated from that one file.

## Layout

| | |
|---|---|
| Footprint | 410 mm long x 366 mm wide (over the tyres) |
| Height | 350 mm to the bottle cap; arm reaches about 0.7 m above ground |
| Ground clearance | 55 mm under the tray |
| Wheels | 4 x 150 mm, 260 mm wheelbase, 316 mm track, skid steer |
| Mass | about 5.2 kg ready to work (simulation model) |
| Arm | 4 axes + gripper: yaw, shoulder, elbow, wrist pitch |
| Arm links | 200 mm + 180 mm + 130 mm to the grasp centre |
| Tools | gripper with soft dished pads; soil-moisture probe and spray nozzle fixed to the wrist at 90 degrees to the gripper |

The tray is a single 380 x 250 mm print because the Neptune 4 Max can do it:
one stiff tub with no seams to leak or creep, instead of a chassis bolted
together from A4-sized pieces.

### Tools at right angles

The probe and the nozzle point along the wrist's -Z axis, the gripper along
+X. With the gripper horizontal the probe is vertical, so one wrist servo
selects between "probe / spray downwards" and "grip forwards" with no tool
changer. The cost is that the probe sticks out behind the wrist when the
gripper points down.

### Reaching the basket

The basket is behind the arm and a 270 degree yaw servo only gives +-135
degrees, so the arm reaches it backwards over its own shoulder. That is why
the elbow uses a 270 degree servo (range -150 to +120 degrees) and why the
inverse kinematics tries four configurations (forward / over-the-shoulder x
elbow up / down) and keeps the one inside the joint limits.

## Arm torque budget

Holding torque from the MuJoCo bench (`simulation/bench_arm.py`) with the
arm at full stretch 6 degrees above horizontal, the worst case for the
shoulder. Stall torques are datasheet values at about 6.8 V.

| Payload | Shoulder | Margin (3.3 N m DS3235) | Elbow | Margin (2.0 N m DS3218) | Wrist | Margin (1.0 N m MG996R) |
|---|---|---|---|---|---|---|
| 0 g | 1.34 N m | 2.5x | 0.54 N m | 3.7x | 0.06 N m | 16x |
| 100 g | 1.84 N m | 1.8x | 0.89 N m | 2.2x | 0.22 N m | 4.6x |
| 150 g | 2.08 N m | 1.6x | 1.04 N m | 1.9x | 0.30 N m | 3.3x |
| 200 g | 2.31 N m | 1.4x | 1.18 N m | 1.7x | 0.39 N m | 2.6x |
| 300 g | 2.82 N m | 1.2x | 1.42 N m | 1.4x | 0.49 N m | 2.1x |

A hand calculation from part masses gave 2.26 N m for the 200 g case, which
agrees with the simulated 2.31 N m.

**Rated payload: 150 g at full reach, about 250 g when working within
350 mm of the shoulder.** Hobby servos overheat if held near stall, so a
1.5x margin is the floor, not a comfortable number. If you need more:

* run a rubber band or a light extension spring from the tower to the upper
  arm brace (a counterbalance removes most of the arm's own 1.3 N m);
* or fit a 60 kg.cm servo at the shoulder (same case size).

Typical tomatoes are 80-150 g and chillies, okra, cherry tomatoes and
strawberries are well under 50 g, so the rating fits small produce. This is
not an arm for brinjal or capsicum crates.

## Drive

| | |
|---|---|
| Motor | JGB37-520, 12 V, about 66 rpm no-load, about 2.4 N m stall (seller data; varies by batch) |
| No-load speed | 66 rpm x pi x 0.15 m = 0.52 m/s |
| Firmware speed limit | 0.40 m/s |
| Stall traction force | 4 x 2.4 N m / 0.075 m = 128 N, 2.5 times the robot's weight |
| Steady climb, 12 degree ramp | simulated, see simulation.md |

The 66 rpm ratio trades speed for torque: a field robot spends its time at
walking pace, and skid-steer turns on soil need several times the
straight-line torque.

**Skid steer** was chosen over Ackermann because it needs no steering
linkage, turns in its own length between rows, and all four motor mounts
are identical. It scrubs the soil when turning, and odometry heading from
wheel counts is poor, which is why the firmware takes heading from the gyro
and only distance from the encoders.

Only the rear encoders are read. On a 4WD skid steer the two wheels of a
side are forced to the same speed by the ground, so the front encoders add
wiring but little information.

## Turntable bearing

The base uses 24 loose 1/4 inch bicycle balls in printed V-grooves at a
100 mm pitch diameter, with three printed hooks that stop the turntable
lifting. A proper thin-section bearing (6810 or a lazy-susan ring) would be
smoother, but 1/4 inch balls are in every cycle shop in India for a few
rupees. The yaw servo then carries torque only; tipping loads go through
the balls and the hooks.

## Joints

Each pitch joint has the servo's output on one side and a printed 12 mm
journal on the other, so the servo shaft is not cantilevered. The upper arm
is two flat 6 mm plates with a box spacer: flat plates print in their
strongest orientation with no supports and are easy to re-print after a
crash.

## Finger pads

The first simulated pick held the fruit through the lift and lost it during
the overhead swing: with flat fingers a round fruit creeps out sideways.
The fix was a clip-on TPU pad with a shallow spherical dish (24 mm radius,
2.5 mm deep) on each finger. The same lesson applies to the real gripper,
so the pads are part of the print list, not an optional extra.

## Known limitations and open questions

* **Nothing has been built.** No part has been printed and no board has
  been wired. Dimensions of bought parts come from datasheets and common
  listings; clones vary. The parameters most likely to need a change are
  listed in [printing.md](printing.md).
* **Probe insertion force is unknown.** The simulation does not model soil.
  In the probing pose the shoulder has about 2.4 N m of spare torque, which
  at a 0.3 m radius is only about 8 N of downward push. That is enough for
  tilled or irrigated beds and almost certainly not for dry, compacted
  soil. A sharpened stainless guide spike beside the blade, or using the
  robot's weight by driving the probe with a separate linear actuator,
  would be the next design step.
* **No perception.** The pick command needs a fruit position from
  somewhere. The ESP32-CAM only streams video to the operator. A Raspberry
  Pi with a depth camera talking the serial protocol is the intended
  upgrade path.
* **Weather.** The tray is a tub with drain holes and a lid, not a sealed
  enclosure. The electronics are open boards. Treat it as a dry-weather,
  polyhouse or kitchen-garden machine.
* **Backlash.** Hobby servos have about a degree of play, which is 5-8 mm
  at the fingertips. Good enough for probing and for fruit on a truss; not
  for precision.
* **Software e-stop only**, unless you add the hard cut-off described in
  the wiring document.
