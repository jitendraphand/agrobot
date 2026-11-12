# Simulation

The simulation is firmware-in-the-loop: `simulation/run_sim.py` loads the
compiled control core (`firmware/lib/agro_core`, the same C++ that goes on
the ESP32) and lets it drive a MuJoCo model built from the CAD. The Python
side only plays the hardware: it turns PWM duty into motor torque, servo
commands into joint torques, and physics state back into encoder ticks,
gyro rate, battery voltage and soil readings. Commands go in through the
same text protocol as the serial port (`ROW 3 610 600 335`).

```bash
pip install mujoco numpy pillow            # plus ffmpeg on the PATH
python simulation/gen_mjcf.py              # CAD manifest -> simulation/model/*.xml
MUJOCO_GL=egl python simulation/run_sim.py row       # crop-row mission
MUJOCO_GL=egl python simulation/run_sim.py terrain   # rough-ground drive
python simulation/bench_arm.py             # payload vs. servo torque table
```

Add `--no-video` for physics and a JSON report only (a few seconds).

## What is modelled

| Item | Model |
|---|---|
| Geometry | The exported STL of every part, placed by `cad/export/manifest.json` |
| Mass / inertia | Printed parts: CAD volume x density x 0.55 fill. Bought parts: catalogue masses. Total 5.2 kg with a two-thirds-full bottle |
| Drive motors | DC motor torque-speed line: torque = 2.4 N m x duty - 0.35 N m s x wheel speed (66 rpm no-load), plus gearbox friction and reflected rotor inertia |
| Servos | Position servo with gain, damping and a hard torque limit at the datasheet stall torque |
| Encoders | 3960 counts per wheel revolution, rear wheels only |
| IMU | Gyro Z with noise; roll / pitch from the true attitude |
| Soil probe | Reads "air" until the blade tip is under the surface, then the moisture assigned to the nearest plant, scaled by insertion depth |
| Contacts | Primitive collision shapes: wheels are cylinders, links are boxes, finger pads are a flat face plus four bumps standing in for the dish |
| Ground | Flat plane (row scene) or a height field with 25 mm ridges and a 12 degree ramp (terrain scene) |

## What is not modelled

* **Soil.** The probe blade passes through the ground with no resistance,
  and wheels do not sink. Real insertion force in dry soil can exceed what
  the arm can push; see the design notes.
* **Vision.** The `PICK` command is given the fruit position by the
  simulator. There is no perception in this project.
* **Spray.** The droplets in the video are drawn by the renderer when the
  pump output is on; there is no fluid simulation.
* Servo backlash, cable drag, battery sag, wheel slip on loose soil.
* Plants are scenery without collision, except the small cup the fruit
  rests on.

So the simulation checks kinematics, sequencing, control loops, reach,
torque margins and stability. It does not prove field performance.

## Results

Numbers below are from `simulation/output/*_report.json`, produced by the
commands above.

### Crop-row mission (`run_sim.py row`, 98 s of simulated time)

| Measure | Result |
|---|---|
| Row mission completed without fault | yes, in 62 s for three plants |
| Soil readings logged by the firmware | 24 / 66 / 27 % (moisture set in the simulator: 24 / 66 / 27 %) |
| Plants sprayed | 2 of 3: the two below the 35 % threshold, 2.0 s each; the wet one was skipped |
| Probe depth reached | 43 mm (commanded 40 mm) |
| Position after 1.81 m in three moves | 1.803 m true, 1.804 m by odometry (7 mm short) |
| Lateral drift / heading drift | 1 mm / 0.08 degrees |
| Pick and place | fruit lifted to 0.71 m on the way over the shoulder and released into the basket: yes |
| Fruit still in the basket after driving on | yes |
| Peak servo torque / stall torque | shoulder 1.09 / 3.3 N m, elbow 0.84 / 2.0 N m, wrist 0.54 / 1.0 N m, yaw 0.29 / 2.0 N m |
| Peak servo following error while moving | 4.6 degrees (wrist) |

The 7 mm position shortfall is the stop tolerance of the distance loop
(2.5 mm per move) adding up; it is not slip.

### Rough ground (`run_sim.py terrain`, 42 s)

| Measure | Result |
|---|---|
| 2.5 m across 25 mm ridges at 0.3 m/s | completed; 2.46 m true distance for 2.50 m of wheel travel |
| 12 degree ramp | climbed 131 mm over 1.06 m |
| 180 degree turn on the spot, on the slope | 179.7 degrees, but it took 16 s with one side's motors at full duty |
| Largest pitch / roll | 15 / 14 degrees (firmware tilt limit is 30) |
| Top speed reached | 0.35 m/s |

Two things to take from this run. Encoder distance over-reads by about 2 %
on ridged ground (the wheels follow the bumps, and slip a little). And the
skid-steer turn on a slope is at the limit of these motors: it finishes,
slowly. For comparison, on the flat hard floor of the bare scene the same
`TURN 180 60` takes 7.2 s at a peak duty of 0.87, and the robot's centre
wanders about 60 mm while it scrubs round. That is still half the commanded
rate, because tyre scrub is higher than `TRACK_SLIP_FACTOR` assumes. On a
real slope with loose soil, plan to turn at the row ends where it is level.

### Arm bench (`bench_arm.py`)

See the torque table in [design_notes.md](design_notes.md#arm-torque-budget).
