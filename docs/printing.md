# Printing guide (Elegoo Neptune 4 Max)

Every file in `cad/export/stl/` and `cad/export/step/` is already rotated
into its print orientation and sits on Z = 0. The build script checks each
part against the 420 x 420 x 480 mm build volume with a 10 mm margin and
fails if one does not fit; the result is
[`cad/export/print_report.md`](../cad/export/print_report.md). The largest
parts are the tray and the deck at 380 x 250 mm.

These settings are a starting point from general PETG / TPU practice. The
parts have **not** been test-printed, so expect to tune fits (see
"Tolerances" below).

## Materials

| Material | Parts | Why |
|---|---|---|
| PETG (or ASA for outdoor life) | everything structural | Tough, does not creep in the sun like PLA. A PLA robot left in a field in May will sag. |
| TPU 95A | `wheel_tyre` x4, `finger_pad` x2 | Grip on soil; soft contact on produce. |

## Slicer settings

| Setting | PETG structural | TPU |
|---|---|---|
| Nozzle / layer | 0.4 mm / 0.20 mm (0.12-0.16 mm for `race_ring`, `turntable`, fingers) | 0.4 mm / 0.20 mm |
| Perimeters | 4 (6 for arm plates and fingers) | 3 |
| Top / bottom layers | 5 / 5 | 4 / 4 |
| Infill | 25 % gyroid (40 % `wheel_rim`; 100 % fingers, clips, `turntable_clip`) | 15 % gyroid for tyres, 100 % for pads |
| Nozzle / bed | 240 C / 80 C | 225 C / 50 C |
| Speed | The Neptune 4 Max will happily do 250 mm/s; keep outer walls at 80-120 mm/s on the tray and deck for layer adhesion | 30-40 mm/s, retraction 1 mm or off |
| Cooling | 30-50 % | 80-100 % |
| Brim | 8 mm on `chassis_tray`, `top_deck`, arm plates | none |

The tray is the long print: roughly 600 g and 15-25 hours depending on
speed. Use a textured PEI sheet with glue stick so the PETG releases, and
keep the room draught-free: a 380 mm part will lift at the corners
otherwise.

## Supports

Parts were designed to print without supports, with three exceptions that
need support **from the build plate only**:

| Part | Where |
|---|---|
| `turntable` | under the round idler journal on the side of the tower |
| `forearm` | under the two idler journals |
| `camera_holder` | under the hinge ear |

The servo windows in the side walls bridge 41 mm; with PETG and the fan at
50 % this prints cleanly enough because the servo tabs seat on the outer
face, not on the bridge.

## Print list

| Group | Files | Qty |
|---|---|---|
| Chassis | `chassis_tray`, `top_deck`, `electronics_plate`, `battery_clamp`, `front_bumper` | 1 each |
| Wheels | `wheel_rim` (PETG), `wheel_tyre` (TPU) | 4 each |
| Arm base | `race_ring`, `turntable` | 1 each |
| | `turntable_clip` | 3 |
| Arm | `upper_arm_drive_plate`, `upper_arm_idler_plate`, `upper_arm_brace`, `forearm`, `wrist_fork` | 1 each |
| Gripper | `gripper_palm`, `finger_drive`, `finger_idle` | 1 each |
| | `finger_pad` (TPU) | 2 |
| Tools | `probe_clip`, `nozzle_clip` | 1 each |
| Deck | `harvest_basket`, `camera_mast`, `camera_holder`, `bottle_holder`, `pump_saddle` | 1 each |

26 unique files, 35 printed pieces, about 2.3 kg of PETG and 0.5 kg of TPU.

## Tolerances you will probably need to tune

All of these are parameters in `cad/agrobot_cad/params.py`; change the
number and run `python cad/build.py`.

| Fit | Parameter | Modelled | If too tight / loose |
|---|---|---|---|
| Servo pocket | `SERVO_POCKET_L`, `SERVO_POCKET_W` | 41.0 x 20.6 mm | servos vary by about 0.5 mm between brands |
| Servo horn bolt circle | `HORN_PCD` | 14 mm | measure your horn discs first |
| Horn face height | `SERVO_HORN_FACE` | 18 mm above the tab seat | sets the gap of every fork; shim with M3 washers if your servo is shorter |
| Idler journal | `JOURNAL_D`, `JOURNAL_CLEAR` | 12.0 mm pin in a 12.3 mm hole | ream the hole if it binds |
| Heat-set insert hole | `M3_INSERT` | 4.0 mm | for 4.6 mm OD inserts |
| Tyre on rim | `RIM_D` + 0.4 mm | TPU tyre stretches over the 128 mm flange | warm the tyre in hot water |
| Wheel hex socket | `COUPLER_AF` + 0.3 mm | 12.3 mm across flats | |
| Ball race gap | `GROOVE_DEPTH` | 3.0 mm V-groove for 6.35 mm balls gives a 2.98 mm gap | deepen the groove if the turntable rocks |

Print `turntable_clip` and one `finger_pad` first: they are ten-minute
parts that tell you whether your printer's holes come out tight.

## Regenerating the files

```bash
pip install cadquery            # Python 3.10 - 3.12
python cad/build.py             # STEP + STL + manifest + print report
python cad/check_interference.py   # clash check of the assembled robot
```
