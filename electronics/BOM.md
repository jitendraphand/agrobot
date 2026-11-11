# Bill of materials

Everything below is a stock hobby-robotics part sold by Indian online stores
(Robu.in, Probots, Zbotic, Robocraze, ThinkRobotics, FlyRobo, Hubtronics,
ElectronicsComp, Amazon.in) and by the component markets (SP Road Bengaluru,
Lamington Road Mumbai, Nehru Place / Lajpat Rai Market Delhi, Ritchie Street
Chennai).

**Prices are indicative street prices in INR including GST, collected in
November 2026. They move a lot between sellers** (the same BTS7960 board was
listed anywhere from about Rs 250 to Rs 760 on the day I looked), so treat the
totals as a budget, not a quote. Nothing here has been ordered or
fit-checked by me; check the "verify" column before buying.

## 1. Drive train

| # | Part | Qty | Approx. each | Verify before buying |
|---|---|---|---|---|
| D1 | JGB37-520 12 V gear motor, **66 rpm**, 6 mm D-shaft, **with hall encoder** | 2 | Rs 900-1400 | Gearbox dia 37 mm, six M3 holes on a 31 mm circle, shaft 7 mm off-centre. Encoder version has a 6-pin JST lead. |
| D2 | JGB37-520 12 V gear motor, 66 rpm, no encoder (front axle) | 2 | Rs 450-700 | Same gearbox as D1. You may also buy four of D1. |
| D3 | BTS7960 43 A H-bridge module | 2 | Rs 250-700 | Single-channel board with the tall heat sink. |
| D4 | 12 mm hex wheel coupler, 6 mm bore, M4 end thread | 4 | Rs 60-120 | Length 18 mm. |
| D5 | M3 x 10 pan-head screws (motor faces) | 24 | - | From the fastener kit. They pass through 6 mm of wall; do not use longer ones, the gearbox threads are shallow. |

Only the two **rear** motors need encoders: the firmware reads one encoder
per side. Plain JGB37-520 motors are much easier to find than the encoder
version; if a seller only has a different ratio, pick 60-110 rpm and update
`MOTOR_GEAR_RATIO` / `MOTOR_NOLOAD_RPM` in `cad/agrobot_cad/params.py`.

## 2. Arm

| # | Part | Qty | Approx. each | Verify before buying |
|---|---|---|---|---|
| A1 | DS3235 35 kg.cm digital servo, **180 deg** (shoulder) | 1 | Rs 1600-2400 | 25T spline, standard size 40 x 20 x 40.5 mm. |
| A2 | DS3218 20 kg.cm digital servo, **270 deg** (base yaw, elbow) | 2 | Rs 1100-2800 | Must be the 270 deg version; many listings are 180 deg. |
| A3 | MG996R metal-gear servo, 180 deg (wrist, gripper) | 2 | Rs 250-450 | 25T spline. |
| A4 | 25T aluminium **disc** servo horn, 4 x M3 | 5 | Rs 60-150 | Printed parts are drilled for a **14 mm bolt circle**. Measure yours; if different, change `HORN_PCD` and re-export. |
| A5 | PCA9685 16-channel PWM servo driver | 1 | Rs 200-400 | I2C address 0x40. |
| A6 | 1/4 inch (6.35 mm) steel balls ("cycle chharra") | 24 | Rs 30-60 / set | Any bicycle shop. |
| A7 | Servo extension leads, 30 cm and 50 cm | 3 + 2 | Rs 30-60 | |

## 3. Brain, sensors, sprayer

| # | Part | Qty | Approx. each | Verify before buying |
|---|---|---|---|---|
| E1 | ESP32 DevKit V1 (ESP-WROOM-32, 30 pin) | 1 | Rs 350-550 | GPIO 16/17 must be free: use a WROOM, not a WROVER, board. |
| E2 | ESP32 30-pin screw-terminal expansion board | 1 | Rs 150-300 | Optional, makes field wiring vibration-proof. |
| E3 | MPU6050 (GY-521) IMU | 1 | Rs 120-220 | |
| E4 | AHT20 (or AHT10) temperature / humidity, I2C | 1 | Rs 100-220 | Optional. |
| E5 | HC-SR04 ultrasonic ranger | 1 | Rs 60-110 | Optional. Needs a 1k/2k divider on ECHO. |
| E6 | Capacitive soil moisture sensor v1.2 / v2.0 (analog) | 1 | Rs 80-180 | Conformal-coat the top edge; see docs/assembly.md. |
| E7 | ESP32-CAM (AI-Thinker) + ESP32-CAM-MB USB board | 1 | Rs 500-800 | Optional. |
| E8 | R385 12 V diaphragm pump | 1 | Rs 200-380 | 6 mm barbs. |
| E9 | MOSFET switch module, logic-level (dual AOD4184 "15 A 400 W") | 1 | Rs 60-140 | Not the IRF520 board: it does not switch fully from 3.3 V. |
| E10 | Misting nozzle with 6 mm push-fit, 2 m of 6 mm silicone tube | 1 | Rs 150-250 | Nozzle body about 10 mm dia. |
| E11 | 500 ml PET bottle | 1 | - | 66 mm diameter; drill the cap for the tube. |
| E12 | Active buzzer 3.3-5 V | 1 | Rs 20-40 | Optional. |

## 4. Power

| # | Part | Qty | Approx. each | Verify before buying |
|---|---|---|---|---|
| P1 | 3S Li-ion pack 11.1 V, 5200 mAh or more, **with BMS** (3S2P 18650), XT60 | 1 | Rs 1600-2800 | Must fit 140 x 50 x 28 mm. A 3S 5200 mAh LiPo also fits but needs a balance charger and a fire-safe bag. |
| P2 | 12.6 V 2 A Li-ion charger | 1 | Rs 350-600 | |
| P3 | XL4016 8 A adjustable buck converter (servo rail, set to 6.8 V) | 1 | Rs 250-480 | |
| P4 | LM2596 buck converter (logic rail, set to 5.0 V) | 1 | Rs 60-120 | |
| P5 | Inline blade fuse holder + 15 A fuse | 1 | Rs 60-120 | |
| P6 | Toggle / rocker switch, 15 A, 12 mm panel hole | 1 | Rs 50-120 | |
| P7 | 16 mm latching emergency-stop push button, NC contact | 1 | Rs 150-350 | |
| P8 | 12 mm panel DC jack or XT60 panel mount (charge port) | 1 | Rs 40-150 | |
| P9 | Electrolytic capacitor 1000 uF 16 V (servo rail) | 2 | Rs 10-20 | |
| P10 | Resistors 33k, 10k (battery divider), 1k, 2k (echo divider) | 1 each | - | |
| P11 | Silicone wire 16 AWG red/black 2 m each, 22 AWG hook-up wire, XT60 pair, JST-XH / Dupont leads, heat-shrink | - | Rs 500-700 | |

## 5. Hardware

| # | Part | Qty | Notes |
|---|---|---|---|
| H1 | M3 brass heat-set inserts (M3 x 5 mm, OD 4.6 mm) | 25 used | Pack of 100 is about Rs 250-400. |
| H2 | M3 socket-head screws: 8, 10, 12, 16, 25 mm, plus 3 x M3 x 40 | assortment | About Rs 300-450 for a 400-piece box; the three 40 mm screws hold the turntable hooks. |
| H3 | M3 nyloc nuts + washers | 30 | |
| H4 | M4 x 12 screws + washers (wheels to couplers) | 4 | |
| H5 | M3 x 10 self-tapping or machine screws for servo tabs | 20 | |
| H6 | Zip ties 2.5 mm, double-sided foam tape, M3 nylon standoffs 6-10 mm | - | Electronics mounting. |
| H7 | Lithium or silicone grease | small tube | Ball race and printed journals. |

## 6. Filament

| Material | Quantity | Used for |
|---|---|---|
| PETG | about 2.3 kg printed, buy 3 kg (Rs 900-1300 / kg) | Everything structural. ASA is a better choice for a robot that lives outdoors. |
| TPU 95A | about 0.5 kg (Rs 1500-2200 / kg) | Tyres and finger pads. |

Print quantities and per-part masses are in
[`cad/export/print_report.md`](../cad/export/print_report.md).

## Budget

| Block | Approx. |
|---|---|
| Drive train | Rs 3,400 - 6,100 |
| Arm | Rs 5,000 - 10,400 |
| Brain, sensors, sprayer | Rs 1,800 - 3,200 |
| Power | Rs 3,100 - 5,500 |
| Hardware | Rs 1,000 - 1,600 |
| Filament (3 kg PETG + 1 kg TPU) | Rs 4,200 - 6,100 |
| **Total** | **about Rs 18,500 - 33,000** |

The low end assumes the cheapest listing of everything and skipping
nothing; the optional items (camera, ultrasonic, air sensor, buzzer,
expansion board) add up to about Rs 900 - 1,500 of it.

## Where the prices came from

Listings seen while compiling this table (stock and price change often):

- JGB37-520 gear motors: [Robu.in category](https://robu.in/product-category/dc-jgb37-520-geared-motors/), [Probots 66 rpm](https://probots.co.in/jgb37-520-dc12v-66rpm-miniature-speed-reduction-gear-dc-motor.html), [MakerBazar](https://makerbazar.in/products/jgb37-520-12v-300rpm-metal-gear-dc-motor)
- Servos: [Probots DS3235 180 deg](https://probots.co.in/servo-ds3235-35kg-180-degrees-high-torque-high-speed-digital-steering-gear.html), [Probots RDS3218 270 deg](https://probots.co.in/rds3218-digital-servo-20kg-dual-shaft-270-degree-with-metal-bracket-water-resistant.html), [Zbotic 35 kg](https://zbotic.in/product/35kg-metal-gear-digital-servo-with-servo-arm-default-270-degrees/), [ThinkRobotics DS3218](https://thinkrobotics.com/products/ds3218-servo)
- BTS7960: [Thingbits](https://thingbits.in/products/bts7960-43a-h-bridge-motor-driver), [FlyRobo](https://flyrobo.in/bts7960-43a-h-bridge-high-power-stepper-motor-driver-module), [Hubtronics](https://hubtronics.in/bts7960-motor-driver-module?tag=driver), [Probots](https://probots.co.in/bts7960-43a-stepper-motor-driver-h-bridge-high-power-module-motor-control.html), [Robocraze](https://robocraze.com/products/smartelex-bts7960-high-power-dual-channel-43a-dc-motor-driver-module)

Rows without a link above are priced from general knowledge of these stores,
not from a listing I opened; they are the least certain numbers in the table.
