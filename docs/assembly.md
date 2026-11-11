# Assembly guide

Read [printing.md](printing.md) and [wiring.md](../electronics/wiring.md)
first. This sequence comes from the CAD model, not from a physical build:
the order is the one that keeps every screw reachable in the model, but
nobody has put this robot together yet. Please open an issue with whatever
you find.

Reference files: `cad/export/agrobot_assembly.step` (open in FreeCAD,
Fusion 360 or any STEP viewer) and
`blender/renders/agrobot_exploded.png`.

Tools: soldering iron with an insert tip, 2 / 2.5 / 3 mm hex keys,
multimeter, flush cutters, 3 mm and 4 mm drill bits for cleaning holes.

## 0. Before any screws

1. **Centre every servo.** Connect each servo to the PCA9685, flash the
   firmware and leave the arm at its power-on pose, or drive each one to
   1500 us with a servo tester. Fit the horn discs afterwards, never
   before.
2. **Set the bucks**: XL4016 to 6.8 V, LM2596 to 5.0 V.
3. **Heat-set inserts** (iron at 230 C, push slowly, let the plastic
   re-freeze before loading):

   | Part | Inserts |
   |---|---|
   | `chassis_tray` | 7 deck posts, 4 electronics-plate bosses, 2 battery-clamp bosses, 2 bumper bosses in the front wall |
   | `turntable` | 1 in the tip of the shoulder journal |
   | `forearm` | 2, in the tips of the elbow and wrist journals |
   | `upper_arm_brace` | 4 (two per side) |
   | `gripper_palm` | 1 in the top of the idler pin |
   | `wrist_fork` | 4, two in each arm for the tool clips (push them in from the outside) |

## 1. Drive train

1. Bolt each gear motor to the inside of its wall pad with six M3 x 10
   screws from outside. The shaft goes through the lower hole, so the
   gearbox sits 7 mm **above** the axle; encoder motors go at the rear.
2. Slide a hex coupler on each shaft, grub screw on the flat, leaving about
   2 mm to the wall.
3. Stretch a TPU tyre over each rim (hot water helps). The chevrons should
   point forward at the top of the wheel on both sides.
4. Push each wheel onto its coupler and fix it with an M4 x 12 screw and
   washer through the centre.

## 2. Tray internals

1. Drop the battery between the floor rails against the rear wall, add a
   strip of foam, and screw `battery_clamp` over its front end (2 x M3 x
   12).
2. Fit the e-stop (16 mm hole), main switch and charge port (12 mm holes)
   in the rear wall.
3. Mount the modules on `electronics_plate` with nylon standoffs following
   the layout sketch in the wiring document, wire them on the bench, then
   screw the plate to its four bosses (M3 x 8).
4. Put the HC-SR04 in the pocket at the back of `front_bumper`, feed its
   four wires through the slot in the front wall and screw the bumper on
   (2 x M3 x 10).

## 3. Deck and arm base

1. Drop the yaw servo (DS3218, 270 deg) through the deck from above, cable
   end towards the rear, and fix its tabs with M3 screws and nyloc nuts.
2. Place `race_ring` around it and bolt it through the deck with three
   M3 x 25 screws in alternate holes (nuts underneath). The slot in the
   ring faces the rear for the servo lead.
3. Grease the groove and lay in the 24 balls.
4. Screw a horn disc to the underside of `turntable` (4 x M3 from above,
   heads in the counterbores), then press the turntable onto the centred
   yaw servo so the tower is at the **front** and fix the horn screw
   through the centre hole.
5. Fit the three `turntable_clip` hooks with M3 x 40 screws through the
   remaining ring holes (clip, ring and deck, nut underneath). The turntable must turn freely with no rocking;
   if it binds, add a washer under each clip.

## 4. Arm

1. **Shoulder servo** (DS3235) into the tower from the +Y side (the left,
   looking forward), output shaft at the top, tabs screwed to the wall.
2. **Elbow servo** (DS3218 270 deg) and **wrist servo** (MG996R) into the
   two windows of `forearm`, both from the same side as the shoulder
   servo. The elbow servo's shaft is at the end with the shorter overhang.
   Route both leads out through the slot in the top wall.
3. Fit horn discs to the shoulder, elbow and wrist servos (centred!).
4. Bolt `upper_arm_drive_plate` to the shoulder horn and to the elbow horn
   (4 x M3 x 8 each). With the shoulder servo centred the upper arm should
   point straight up, and with the elbow servo centred the forearm should
   be folded about 15 degrees forward of the upper arm's line. Get both
   within one spline tooth; the firmware calibration removes the rest.
5. Grease the journals. Slide `upper_arm_idler_plate` over the two
   journals on the other side and retain each with an M3 x 8 screw and a
   large washer; do not clamp, the plate must turn.
6. Screw `upper_arm_brace` between the plates (4 x M3 x 12). Pass the
   elbow and wrist leads through the brace.
7. `wrist_fork`: drive side onto the wrist horn (4 x M3 x 8), idler side
   over the journal with screw and washer.

## 5. Gripper and tools

1. Gripper servo (MG996R) into `gripper_palm` from above, shaft at the
   front, on the same side as the horn discs of the arm.
2. Bolt `gripper_palm` to the fork web (4 x M3 x 16 with nuts).
3. Horn disc on the gripper servo; bolt `finger_drive` to it so that the
   finger points straight forward when the servo is at its closed end.
4. Drop `finger_idle` over the pin with its teeth meshed so that both
   fingers are symmetric, retain with an M3 x 8 screw and washer.
5. Press a `finger_pad` on each finger, dish facing inwards, with a drop
   of cyanoacrylate.
6. `probe_clip` on the right-hand fork arm (the idler side), `nozzle_clip`
   on the left, each with 2 x M3 x 8 into the inserts in the fork arms.
   Do not use longer screws: anything that pokes through the arm rubs on
   the forearm. Slide the
   soil sensor into the clip and zip-tie it. **Seal the electronics at the
   top of the sensor** with conformal coating, nail varnish or hot glue:
   only the blade below the white line is meant to get wet.

## 6. Deck accessories and closing up

1. Screw `harvest_basket`, `camera_mast`, `bottle_holder` and
   `pump_saddle` to the deck (M3 x 10 with nuts underneath).
2. Camera: ESP32-CAM into `camera_holder`, holder between the mast ears
   with an M3 x 25 screw and nyloc nut as a friction hinge.
3. Pump on its saddle with two zip ties; suction tube into the bottle
   through a hole in the cap, delivery tube along the arm (leave slack at
   every joint) to the nozzle.
4. Bring the arm leads down through the 20 mm grommet hole behind the
   turntable with a loop long enough for 135 degrees of yaw either way.
5. Plug the leads into the PCA9685 (table in the wiring document), check
   polarity once more, and fix the deck with seven M3 x 12 screws.

## 7. First power-up

1. Wheels off the ground, arm clear, e-stop released.
2. Power on with the robot standing still (the gyro calibrates in the
   first second).
3. Follow "Calibration" in [firmware.md](firmware.md) before running any
   mission: until the servo zero points are set the arm does not know where
   it is.
