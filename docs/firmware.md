# Firmware

```
firmware/
  platformio.ini          ESP32 DevKit V1, Arduino framework, no external libraries
  src/
    main.cpp              100 Hz control tick, serial console, servo calibration commands
    hal_esp32.cpp/.h      encoders, BTS7960 PWM, PCA9685, MPU6050, AHT20, HC-SR04, ADCs
    web_server.cpp/.h     WiFi access point + HTTP API
    web_ui.h              the control panel page (served from flash, works offline)
    pins.h                pin map
  lib/agro_core/          everything that decides what the robot does (portable C++17)
    robot_geometry.h      GENERATED from the CAD parameters
    agro_config.h         gains, speeds, safety limits, agronomy thresholds
    kinematics.*          arm FK / IK (4 configurations, joint limits)
    drive.*               wheel-speed PI, odometry, distance / turn moves
    arm_controller.*      smooth joint moves and straight-line tool moves
    mission.*             row mission, plant care, pick-and-place programs
    robot.*               state machine, interlocks, telemetry
    protocol.*            text command parser
    c_api.cpp             C binding used by the simulator (not built for the ESP32)
  test_host/              unit tests + stub headers for checking the sketch on a PC
  esp32cam/               MJPEG streaming sketch for the camera
```

## What has and has not been verified

| Check | Status |
|---|---|
| Control core unit tests on a PC (`make -C firmware/test_host test`) | **pass**, 127,470 assertions |
| Control core driving the physics simulation (same C++, via ctypes) | **pass**, see [simulation.md](simulation.md) |
| `src/*.cpp` type-checked against stub Arduino headers (`make sketch-check`) | **pass** |
| Compiled with the real ESP32 toolchain (`pio run`) | **not done**: the build machine could not reach the PlatformIO registry |
| Run on an ESP32 with real motors, servos and sensors | **not done** |
| `esp32cam/esp32cam.ino` | **not compiled, not run** |

So the logic is tested; the hardware layer is written carefully against the
documented Arduino-ESP32 APIs but is unproven. Expect to fix small things
on the first `pio run` and on first power-up (pin polarity, servo
direction, a register value). The code supports both Arduino-ESP32 core 2.x
and 3.x LEDC APIs.

## Build and flash

```bash
pip install platformio
cd firmware
pio run -t upload            # USB cable to the ESP32
pio device monitor           # 115200 baud
```

On boot the robot prints its WiFi name and address:

```
AgroBot Mk1
PCA9685 ok, MPU6050 ok
WiFi AP 'AgroBot-3F7A'  password 'agrobot123'  ->  http://192.168.4.1/
```

Join that network with a phone and open `http://192.168.4.1/`. Change
`AP_PASSWORD` in `src/web_server.cpp` before using it anywhere public: the
control page has no login of its own.

## Calibration (do this once, in this order)

Type these in the serial monitor.

1. **Servo zero points.** `CAL SHOW` prints the table. For each joint, send
   the joint to a known angle and adjust until the arm is really there:

   | Joint | Send | The arm should be | Adjust with |
   |---|---|---|---|
   | 1 shoulder | `ARM J 0 90 0 0` | upper arm vertical (use a set-square) | `CAL 1 <zero_us> 11.11` |
   | 2 elbow | same | forearm in line with the upper arm | `CAL 2 <zero_us> 7.41` |
   | 3 wrist | same | gripper in line with the forearm | `CAL 3 <zero_us> 11.11` |
   | 0 yaw | `ARM J 0 90 0 0` | arm plane straight ahead | `CAL 0 <zero_us> 7.41` |
   | 4 gripper | `GRIP 0` | pads 3 mm apart | `CAL 4 <zero_us> 11.11` |

   `zero_us` is the pulse width at joint angle 0; `us_per_deg` is 11.11 for
   a 180 degree servo and 7.41 for a 270 degree servo. **Make it negative
   if the joint moves the wrong way.** Finish with `CAL SAVE`.

2. **Drive direction and encoders**: see "Motor polarity" in the wiring
   document. Then `MOVE 1000` and measure. If the robot is consistently
   short or long, correct `WHEEL_D` in `params.py` (tyres squash) and
   regenerate the header.

3. **Turning**: `TURN 360`. Heading comes from the gyro, so this should be
   close already. `TRACK_SLIP_FACTOR` in `agro_config.h` only matters if
   the IMU is missing.

4. **Soil probe**: note the `soil` value in `STATUS` with the blade in dry
   air and in a glass of water, and put the raw ADC counts into
   `SOIL_RAW_DRY` / `SOIL_RAW_WET`. Pick `SOIL_DRY_PCT` for your crop and
   soil; the default of 35 % is a placeholder, not agronomy advice.

5. **Battery**: compare `batt` in `STATUS` with a multimeter and trim
   `BATT_DIVIDER` in `pins.h`.

## Command protocol

One line per command over USB serial, or `GET /cmd?c=<url-encoded line>`
over WiFi. Replies are `OK ...` or `ERR <reason>`. Units are millimetres and
degrees. Positions are in the robot frame: origin on the ground under the
middle of the chassis, +X forward, +Y left, +Z up.

| Command | Meaning |
|---|---|
| `PING` | `OK PONG` |
| `STATUS` | one line of JSON (also `GET /status`) |
| `VEL <mm/s> <deg/s>` | drive; must be repeated at least every 0.6 s or the robot stops |
| `MOVE <mm> [mm/s]` | straight move with gyro heading hold |
| `TURN <deg> [deg/s]` | turn on the spot, positive = left |
| `ARM J <yaw> <shoulder> <elbow> <wrist>` | joint move |
| `ARM P <x> <y> <z> <pitch> [G\|P\|N]` | move the gripper (G), probe tip (P) or nozzle (N) to a point |
| `ARM L <x> <y> <z> <pitch> [G\|P\|N] [mm/s]` | same, in a straight line |
| `ARM HOME`, `ARM STOW` | named poses |
| `GRIP <deg>`, `GRIP OPEN`, `GRIP CLOSE` | each finger's opening angle, 0-50 |
| `PUMP <seconds>`, `PUMP ON`, `PUMP OFF` | sprayer |
| `PROBE <rowY> [spray height]` | probe the soil `rowY` mm to the side (+ left), spray if dry |
| `ROW <n> <first> <spacing> <rowY> [spray height]` | drive `first` mm, treat a plant, then `spacing` mm between the next `n-1` |
| `PICK <x> <y> <z> <diameter>` | pick a round fruit at that point and drop it in the basket |
| `STOP` | stop motion and cancel the mission |
| `ESTOP`, `RESET` | latch / release the software emergency stop |
| `CAL ...` | servo calibration (serial only) |

`pitch` is the elevation of the gripper axis: 0 = horizontal pointing away
from the arm base, -90 = straight down. With pitch 0 the soil probe and the
nozzle point straight down, which is how the missions use them.

## Safety behaviour built into the core

* Teleop velocity commands expire after 0.6 s (lost WiFi stops the robot).
* E-stop input is fail-safe (open circuit = stop) and latches until `RESET`.
* Forward motion is refused while the ultrasonic range is under 22 cm.
* Roll or pitch beyond 30 degrees, or battery under 9.6 V while driving,
  puts the robot in `FAULT` with motors off.
* Every arm target is checked against joint limits and reach before the arm
  moves; a mission whose points are out of reach is rejected before it
  starts.
* Teleop is locked out while a mission runs.

These are software interlocks on a hobby controller. They are not a
substitute for keeping hands, children and animals away from a moving arm.

## Tests

```bash
make -C firmware/test_host test          # unit tests of the control core
make -C firmware/test_host sketch-check  # type-check src/ against stub headers
make -C firmware/test_host lib           # libagrocore.so for the simulator
```
