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
floor. Hold the gamepad 1 left stick button for 25% driving speed. Use one stick axis at a time:

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

## 4. Expansion Hub fly motors

Configure the new motors in the active Robot Controller configuration:

| Configuration name | Expansion Hub motor AND encoder port |
| --- | --- |
| `flymotor1` | 1 |
| `flymotor2` | 2 |

Use the matching goBILDA motor profile. The code assumes 6000 RPM Yellow Jacket
motors with 1:1 internal gearboxes and 28 encoder ticks per motor revolution.
It overrides the profile's ticks/revolution, rated RPM, and fly motor velocity
PIDF coefficients. Verify the exact motor specification before
running; the drivetrain's 384.5 ticks/revolution does not apply here.

The external 8-tooth motor gear drives a 14-tooth output gear:

```text
output RPM = motor RPM * 8 / 14
6000 motor RPM = 3428.57 output RPM (nominal no-load)
encoder ticks/second = output RPM * 14 / 8 * 28 / 60
100 output RPM = 175 motor RPM = 81.67 encoder ticks/second
```

In **Mecanum TeleOp**, both motors start stopped and share one target:

- Gamepad 1 right bumper: increase by 100 output RPM per press.
- Gamepad 1 left bumper: decrease by 100 output RPM per press.
- Holding a bumper does not repeat; pressing both makes no change.
- B: zero the target and remove power from both speed motors.
- Target range: 0-3400 output RPM, preserving whole 100-RPM steps without reversal.
- Left stick button: hold for 25% driving speed, replacing the old left bumper control.
- Driver Station Stop removes power from all motors.

Each speed motor uses its own encoder and the hub's built-in PIDF velocity loop,
not a second software PID competing with the hub. Telemetry shows battery
voltage, applied PIDF gains, the shared output-RPM target, and each motor's
measured output RPM, RPM error, and raw ticks/second. Output RPM is inferred
from its encoder and the gear ratio; it cannot detect gear slip.
Zero target and shutdown let the speed motors coast; B is not a mechanical brake.
Both fly motor directions are REVERSE. **Power off and restore both motors'
power leads to their original polarity before running this version.** Software
reversal then preserves the counterclockwise rotation previously achieved by
flipping the power leads, while keeping positive encoder feedback for positive
velocity targets. Verify rotation from the same viewing side used previously,
especially if the motors are mechanically coupled.

Reversing only the power leads makes motor output oppose encoder feedback.
Changing SDK Direction reverses both output and reported encoder sign; it cannot
fix that mismatch while the leads remain flipped. Do not mask negative RPM with
an absolute value: the hub would still receive incorrect velocity feedback and
could demand full power. With restored leads and the software reversal, each
motor must report positive RPM for a positive target.

Secure the mechanism and keep people clear of rotating parts. Before closed-loop
operation, verify each motor/encoder pairing and encoder sign independently at
low open-loop power using a suitable motor test; **Mecanum Wheel Test only tests
the drivetrain**, not these motors. Incorrect feedback can cause full power
even at a low RPM target. Stop immediately if feedback is absent or incorrect.
Then start at 100 output RPM and compare target versus actual as you step up.
The 3400 RPM cap is close to nominal no-load speed, not a guaranteed regulated
speed under load. Reduce the target if either motor cannot track it; check
wiring, load, and battery before tuning PIDF.

### Voltage compensation and tuning

`FLY_VELOCITY_P/I/D/F` in `MecanumTeleOp` are explicit starting gains for the
fly motors only. The nominal 12 V feedforward is `32767 / 2800 = 11.7025`,
using the hub's full-scale output and the motor's nominal ticks/second.
Initial P is `0.1 * F`, I is `0.1 * P`, and D is zero to avoid amplifying
encoder velocity noise. These are starting estimates, not hardware-tuned gains.

Every 250 ms the code samples the lowest positive finite hub voltage and
adjusts F by `12 / measuredVoltage`. It only rewrites gains when F changes by
more than 0.05. Missing/invalid readings use nominal feedforward and show a
warning. P corrects instantaneous velocity error and I corrects persistent
error under load; the hub runs this feedback loop independently for each motor.
Voltage compensation reduces battery-related variation only while sufficient
voltage and torque remain available. It cannot boost the battery voltage.

Start with low targets, then compare settled RPM at attainable targets such as
1000, 1500, and 2000 with charged and partially discharged batteries. Allow the
flywheel to accelerate before judging error. If speed oscillates, reduce P/I;
if there is persistent error below the physical limit, tune F then P/I in small
steps using measured responses. Test speed reductions and B/Stop as well as
increases. Do not tune gains to force an unreachable target.

### Diagnosing 3400 requested versus 2400 measured

With the assumed gearing, 3400 output RPM requires 5950 motor RPM (2777 ticks/s).
2400 output RPM corresponds to 4200 motor RPM (1960 ticks/s). Confirm that the
reported 2400 is **output RPM**, not raw ticks/second or motor-shaft RPM. Verify
the actual motor model, encoder resolution, and gear tooth counts; do not change
the conversion just to make telemetry match the target.

3400 is 99.2% of the nominal no-load output speed. Friction, flywheel aerodynamic
drag, wiring losses, and battery sag all reduce attainable speed. After verifying
encoder sign/pairing at low open-loop power, a secured, guarded open-loop sweep
in a suitable test OpMode can distinguish tuning from a physical limit. Increase
power gradually only within the mechanism's safe speed limits; release-to-stop
controls are required. Do not use the drivetrain-only test for this.

If safe full-power operation also settles near 2400 RPM, PID cannot reach 3400:
check battery voltage under load, connectors, gear mesh/bearings, and motor specs.
Choose a regulated target below the measured loaded maximum at the lowest
operating battery voltage, or change the gearing/motor/load for more headroom.
If open-loop operation comfortably exceeds 2400 but PIDF control does not, tune
the gains against those measurements. `getPower()` in encoder mode is not actual
PWM duty, so it cannot establish that the hub has saturated at full power.

## Build verification

From the repository root:

```powershell
.\gradlew.bat :TeamCode:assembleDebug :TeamCode:lintDebug
```

Compilation cannot validate the physical mapping, encoder wiring, motor
direction signs, or speed-loop tuning. Complete the tests above on the robot.
