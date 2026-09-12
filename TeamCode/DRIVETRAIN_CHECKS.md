# Mecanum bring-up

The code assumes four goBILDA 435 RPM Yellow Jacket motors (13.7:1 internal
gearboxes, approximately 384.5 encoder ticks per output revolution) and 1:1
external bevel gearing. Roller diagonals must form an X viewed from above.
Front/back and left/right are from the robot's perspective.

## 1. Verify hardware before closed-loop driving

Power off before changing wiring. In the active robot configuration, verify:

| Physical wheel | Configuration name | Intended Control Hub motor AND encoder port |
| --- | --- | --- |
| Front left | `leftfront` | 0 |
| Front right | `rightfront` | 1 |
| Back left | `leftback` | 2 |
| Back right | `rightback` | 3 |

Names, not port numbers in comments, determine the code's mappings. Use the
matching goBILDA motor profile on all four ports. The TeleOp overrides the
drive motors' ticks/revolution and rated RPM for the 435 RPM gearbox, but does
not tune or replace the configured hub velocity PIDF coefficients.
The intake remains `intake` on Expansion Hub motor/encoder port 0, with its
existing encoder mode and full-speed trigger controls unchanged.

Securely support the chassis with every wheel off the ground. Keep hands,
clothing, and cables clear; have the Driver Station Stop control ready.
Select **Mecanum Wheel Test**, initialize, and start it.

1. Hold right bumper plus exactly one button: X = front left, Y = front right,
   A = back left, B = back right. Only that wheel should move at 15% open-loop
   power. Release either control to stop. Holding multiple face buttons stops
   all wheels. Left bumper reverses the test direction.
2. Verify the physical wheel matches its name. Fix configuration/power wiring
   if not. Watch all four encoder positions: only the selected wheel's channel
   should change substantially. Another channel changing indicates crossed
   encoder cables. Zero or erratic counts indicate an encoder/cable problem.
3. With positive test power (no left bumper), encoder position must increase.
   If it decreases, investigate motor-lead polarity/encoder wiring. Changing
   SDK `Direction` reverses both output and reported encoder sign, so it does
   NOT repair a motor/encoder polarity mismatch.
4. Positive power must rotate the wheel so its top travels toward the robot's
   front (the bottom travels toward the rear). If the encoder sign is correct
   but wheel rotation is backward, change that wheel's entry in
   `MecanumTeleOp.DRIVE_DIRECTIONS` from `FORWARD` to `REVERSE`, rebuild/deploy,
   and repeat. Entries are FL, FR, BL, BR and are shared by both OpModes.

Rear right is currently REVERSE; the other three direction entries are FORWARD.
These settings are NOT a verified Strayfer direction map.
The mounting description and 1:1 ratio alone do not establish the required
signs for the assembled bevel gears and wiring. Do not run normal driving
until every wheel passes these checks.

## 2. Check mixing and velocity

Select **Mecanum TeleOp**. First test with the robot supported, then on a clear
floor. Hold left bumper for 25% driving speed. Use one stick axis at a time:

| Command | FL | FR | BL | BR |
| --- | --- | --- | --- | --- |
| Forward (left stick up) | + | + | + | + |
| Strafe right (left stick right) | + | - | - | + |
| Rotate clockwise (right stick right) | + | - | + | - |

Signs mean calibrated wheel-forward/backward, not clockwise as viewed from
opposite sides of the robot. Each pure-axis command should have equal speed
magnitudes; diagonal motion and simultaneous turning intentionally do not.
The previous rotation input was inverted; the new code makes right stick
right request clockwise rotation. A small rescaled deadband rejects stick
drift. The original 1.1 lateral-input multiplier is retained.

Telemetry shows each wheel's target and actual **ticks/second**, plus encoder
position. Compare speeds after acceleration has settled. Full-scale target
is about 2,091 ticks/s (326 wheel RPM), 75% of nominal no-load speed to leave
regulation headroom. Slow mode reduces that to about 523 ticks/s. Lower the
common maximum if the slowest wheel cannot reach it under normal load or at
lower battery voltage; 75% is a starting point, not a guaranteed attainable
speed. RPM = ticks/second * 60 / 384.5.

The original `RUN_USING_ENCODER` with `setPower` already enabled hub velocity
regulation. The new `DcMotorEx.setVelocity` commands explicit per-wheel
targets with one common speed scale and exposes feedback for diagnosis. It
uses the hub's built-in velocity loop, not an extra competing software PID.

**A velocity limit is not a power limit.** Missing/mispaired encoder feedback
can make the hub demand full motor power even in slow mode. Use the open-loop
wheel test first and stop immediately if feedback is wrong or a wheel runs
away. Do not try to solve this by increasing PID gains.

## 3. If motion still looks wrong

- Equal forward targets but different actual speeds: check encoder pairing,
  battery/connectors, bevel mesh/alignment, bearing preload, and wheel rubbing.
  If hardware is sound, compare velocity PIDF settings and tune with measured
  step responses. Do not assign arbitrary per-wheel power multipliers.
- Actual speeds track targets but the chassis drifts or strafes poorly: check
  wheel handedness, free rollers, loose hubs/gears, wheel diameters, and uneven
  weight or traction. Motor encoders do not detect a slipping wheel or loose
  wheel hub. Correct encoder speeds do not guarantee straight chassis motion;
  heading hold would require additional feedback such as an IMU.
- All actual speeds fall short: reduce the common maximum and test with a
  charged battery. Controllers cannot produce unavailable voltage or torque.
- Intake operation changes drivetrain behavior: check battery voltage sag,
  connection quality, and intake binding/current draw.

## Build verification

From the repository root:

```powershell
.\gradlew.bat :TeamCode:assembleDebug :TeamCode:lintDebug
```

Compilation cannot validate the physical mapping, encoder wiring, motor
direction signs, or speed-loop tuning. Complete the tests above on the robot.
