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

RESULTS_PLACEHOLDER
