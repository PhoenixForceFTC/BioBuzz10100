# Intake, index and shooter bench test

**Test status: complete October 26, 2026.** The operator confirmed all transfer and intake tests complete, and turret rotation, shooting/angle and gate tests good. See [mechanism test sign-off](HardwareReviewImplementationStatus.md#mechanism-tests--complete-october-26-2026). Numerical calibration remains separate.

Select **Test: Intake + Index + Shooter** in the Driver Station's Test group.
It initializes the intake, both transfer motors, full sensor, gate, turret servos,
turret encoder and flywheel. It does not initialize the drivetrain, Pinpoint,
Limelight or depollinator. Use Gamepad 1.

The turret homes during INIT to robot-right (90 degrees), with the hood at its
highest launch elevation (+15 degrees). Clear its travel before INIT. After START,
release B, both bumpers and both triggers once to enable motor controls. The
flywheel and transfer do not start automatically.

| Control | Action |
| --- | --- |
| Hold RB | Intake inward, stage 1 loading with stage 2 off; full confirmation reverses intake and stops both transfers until feeding |
| Hold LB | Reverse intake only; transfer stops and gate closes |
| Both bumpers | Stop intake and loading; inhibit feeding |
| Hold RT | Spin flywheel at selected RPM; release stops powering it |
| Hold RT + LT | Qualify readiness, then repeat timed feed/recovery cycles; release either trigger to stop feeding and close the gate |
| Tap Y / A | Increase / decrease flywheel output RPM by 100, clamped to 0..3700 RPM; starts at 2800 RPM |
| D-pad up / down | Lower / raise launch elevation by 0.5 degrees per press |
| Right stick X | Rotate turret; release holds its target |
| X | Rehome yaw and hood when bumpers and triggers are released |
| B | Stop intake, transfer and flywheel commands and close gate; release B, bumpers and triggers to resume |

Feeding pauses loading and requires a homed, settled turret and hood, plus measured
flywheel speed within 100 output RPM of the selected target. Bench yaw settling accepts
up to 4 degrees of error (the retained bench-specific tolerance) and 3 degrees/s
of measured motion; the competition auto-aim's tighter 3-degree requirement does
not apply to this manual test. Release the yaw stick before starting feed. Initial
readiness must hold for the configured 0.15 seconds. Feed then waits for the gate's
0.20-second command-to-feed delay. Both stages feed forward for 0.50 seconds with the
gate held open (about 0.70 second open per uninterrupted attempt), followed by 0.35 seconds
of recovery with the gate closed and both transfer motors stopped. Holding RT + LT
repeats the cycle after readiness requalifies and the gate reopens. Loss of readiness
closes the gate and enters recovery. Releasing either trigger, pressing B or LB,
losing home, or setting RPM to zero stops feeding. Software counts timed attempts,
not launched balls.

The flywheel starts with the FTC SDK's encoder velocity PIDF controller in
`RUN_USING_ENCODER` mode. **Tap GP2 B** to switch between PIDF and bang-bang.
Switching stops the motors and closes the gate; release GP1 B, bumpers and triggers
to re-arm, then hold RT again. A held GP2 B toggles only once. The selected RPM and
PIDF tuning are retained, while comparison measurements reset.

Bang-bang uses `RUN_WITHOUT_ENCODER` (encoder readings still work), full forward
power whenever measured RPM is below target, and zero power/coast at or above
target. It runs every main loop, including recovery, using one cached velocity
sample per loop. There is no deadband delaying recovery and no reverse braking;
the motor stays in FLOAT. PIDF remains P=40, I=0, D=0 with voltage-compensated F
and GP2 live tuning. These are starting values, requiring loaded verification.
PID tuning controls retain settings in bang-bang mode but only affect PIDF.

Actual/selected RPM, active controller, bang-bang commanded power, lowest RPM in
the last/current feed, and elapsed/last return to RPM tolerance (seconds) are
displayed. Return time also includes spin-up, and ends at first entry into the
RPM tolerance, before the separate stable-readiness check. Compare controllers
at the same target, battery condition and ball supply. Feed/recovery timing and
RPM interlocks are identical for both controllers. Full power cannot compensate
for insufficient motor power under sustained load.

The installed hood uses increasing PWM to lower away from its upper stop: home
is 0.511268 and full downward travel is approximately 0.680282. D-pad Up increases
travel; Down returns toward home and does nothing if already at home. Both angle
commands and raw servo commands are bounded to the permitted side of home. Verify
with one Up press and then one Down press before attempting the full range.

Flywheel RPM is output-shaft RPM after the external 2:3 speed reduction: 5550 motor
RPM produces approximately 3700 flywheel RPM. The encoder remains on the motor (28 counts/rev);
software converts both commands and readings. The 2800 RPM starting target retains
the previous 4200 motor RPM operating point. Calibration displays flywheel output RPM. It does not save session files.

Turret PWM direction is reversed for the reported mounting so the right stick
moves the turret in the corresponding direction. The external encoder correction
is OFF by default to prevent an extra software trim after stopping. The Axon
servos still hold position using their internal controllers, and the Through-Bore
encoder still reports angle/rate and qualifies initial feed readiness. Check that
physical clockwise motion increases the displayed robot heading. If it does not,
verify encoder sign before enabling external correction. Only `Test: Turret` uses
B to toggle optional correction; B in this combined test remains the motor stop.

If RT spins the flywheel but RT + LT does not feed, read **FEED STATUS** at the top
of telemetry. It identifies missing trigger input, a held LB, required homing,
flywheel RPM error, turret settling or hood travel. Actual/selected RPM and yaw
error/rate appear directly below it. A spinning flywheel alone does not establish
RPM readiness: its encoder cable must be connected to encoder port 0. Release LB
while firing. If home is required, release the motor controls, press X, and wait
for homing to finish before trying again. During an active feed, FEED STATUS shows
TIMED FEED during the feed window and recovery/readiness reasons between attempts.

Start empty: check directions and clearance, then spin the flywheel and exercise
gate/feed into a clear catch area. Next hold RB to collect one ball, release RB,
hold RT until speed is steady, and add LT to feed. Release LT after the ball exits.
Add more balls only after that works. Stage 2 stays off during loading; both transfers feed forward during qualified feeding.
The full sensor must remain HIGH for two seconds to stop loading; check its
placement and that this delay does not cause a jam. Full does not prevent shooting.

B removes motor commands; the flywheel coasts and a ball already in the shooter
can still exit. Stop the OpMode and wait for motion to stop before clearing jams.
STOP or a loop exception attempts to stop every motor and close the gate.

See [shared motor baseline, PIDF tuning and hood reference](ShooterVelocityTuning.md) for the 3700 RPM target cap, GP2 live tuning controls, and calibration compatibility.
