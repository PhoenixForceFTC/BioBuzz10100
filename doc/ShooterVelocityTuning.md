# Shooter speed and loaded tuning

`BareMotorSpecifications` is the shared configuration for nominal 6000 RPM bare
motors on the drivetrain and shooter. The team-selected free-speed baseline is
5550 RPM at 12 V; encoder resolution remains 28 counts/revolution. This baseline
is not a guarantee of speed under load or a manufacturer specification for every
motor. The current [5203 page](https://www.gobilda.com/5203-series-yellow-jacket-motor-1-1-ratio-24mm-length-8mm-rex-shaft-6000-rpm-3-3-5v-encoder/)
lists 6000 RPM no-load, while the older [5202 page](https://www.gobilda.com/5202-series-yellow-jacket-motor-1-1-ratio-24mm-length-6mm-d-shaft-6000-rpm-3-3-5v-encoder/)
lists 5400 RPM tested. Measure the installed motors rather than treating a
catalog or team baseline as an exact reachable speed.

With the external 2:3 reduction, the configured flywheel free-speed estimate is
3700 RPM. Shooter commands, profiles and calibration allow the full configured maximum of
3700 flywheel RPM, with no software headroom reserve. Default speed
remains 2800 flywheel RPM (4200 motor RPM) in the combined bench test; competition TeleOp starts at 3500 flywheel RPM. A low battery or continuous ball load
can reduce the achievable speed. Do not widen readiness tolerance to hide that.

All turret-based OpModes default to hub velocity PIDF in
`RUN_USING_ENCODER`. Starting gains are P=40, I=0, D=0. P was doubled from 20 for
stronger response to shot-induced speed error. Integral remains off initially to
avoid windup near maximum speed, but can be tuned during the bench test. Feedforward at 12 V is
`32767 / (5550 / 60 * 28)`, approximately 12.651. Every 0.2 seconds its F coefficient
is multiplied by `12 / batteryVolts`, using the lowest valid hub voltage reading.
Invalid or unavailable voltage uses the 12 V reference. These are initial gains,
not gains validated on this flywheel. There is no second software velocity PID
loop. [FTC PIDF documentation](https://ftc-docs.firstinspires.org/en/latest/programming_resources/shared/pidf_coefficients/pidf-coefficients.html)
describes the per-port controller settings.

In **Test: Intake + Index + Shooter**, Gamepad 1 retains the shooting controls.
Gamepad 2 can adjust gains while testing:

| GP2 control | Change |
| --- | --- |
| B | Toggle PIDF / bang-bang in this bench test; stops motors and requires GP1 controls released to re-arm |
| D-pad Up / Down | P +1 / -1, bounded 0..100 |
| D-pad Right / Left | Feedforward multiplier +0.05 / -0.05, bounded 0.5..1.5 |
| Y / A | I +0.05 / -0.05, bounded 0..5 |
| X | Reset P, I, D and feedforward multiplier to the initial settings |

Bench settings are shared with FTC Dashboard and remain in memory across bench restarts
until the Robot Controller app restarts. Record validated values in
`ShooterVelocityTuning` to apply them to every turret-based OpMode. Current P/I/D/F,
feedforward multiplier, actual/target RPM, battery voltage, motor current, and the
lowest RPM observed during the last/current feed are displayed. The speed-headroom
percentage is an estimate from voltage and free speed, not measured motor duty.

## Live FTC Dashboard tuning

FTC Dashboard 0.6.0 is installed in TeamCode, with its transitive FTC SDK dependencies
excluded so the project retains SDK 12.0.0. See the official
[setup](https://acmerobotics.github.io/ftc-dashboard/gettingstarted) and
[configuration/telemetry reference](https://acmerobotics.github.io/ftc-dashboard/features).

1. Build and deploy the Robot Controller app once. Connect the laptop to the Control
   Hub's Wi-Fi and open [Dashboard](http://192.168.43.1:8080/dash).
2. Select **Test: Intake + Index + Shooter** on the Driver Station. In Dashboard's
   Config view, expand **IntakeShooterTestOpMode**. Editable fields are `P`, `I`, `D`,
   `F_SCALE` and `TARGET_RPM` (flywheel output RPM, not motor RPM).
3. Start, then release the bumpers/triggers to arm. Hold GP1 RT to spin. GP1 Y/A
   adjusts the same target as Dashboard. Hold RT + LT for the existing timed,
   readiness-controlled feed/recovery cycle; GP1 B stops motors and closes the gate.
4. Apply one field change at a time in Config. Gains are read each active loop and
   sent to the hub only when changed. Gamepad gain edits modify those same fields;
   refresh Config to see gamepad edits. GP2 X resets gains, including D, but not RPM.
5. Add `Target RPM` and `Measured RPM` to a graph. Add `Error RPM` and `Feeding (0/1)`
   to observe sag and recovery. Packets are sampled at up to 20 Hz; electrical readings
   retain their 5 Hz sample rate. `Selected RPM` is the operator setting even while
   stopped; `Target RPM` is the actual command and becomes zero when spin is released.

Applied P/D are bounded to 0..100, I to 0..5, F scale to 0.5..1.5 and target to
0..3700 flywheel RPM. Non-finite gains retain the prior valid setting; a non-finite
RPM selection commands zero. Graph/Driver Station gains show the bounded values
actually selected; Config can still display an out-of-range requested value.
The hub F coefficient remains `BASE_F * 12 / batteryVolts * F_SCALE`: edit the
multiplier rather than replacing voltage compensation. PIDF gains do not affect
bang-bang mode. Other OpModes keep their compiled defaults; no Dashboard values
automatically transfer into competition. Save winning gains in Java source and
rebuild. Save P/I/D in `INITIAL_P`/`INITIAL_I`/`INITIAL_D` and the F multiplier in
`INITIAL_F_SCALE` in `ShooterVelocityTuning`.

Keep I/D zero initially, tune steady-state F then loaded P, and compare repeated
shots at consistent target, feed pattern and battery voltage. Minimum-feed RPM
only samples while the transfer is commanded on; a later ball impulse may be missed.
Return-to-tolerance time includes spin-up and does not prove sustained stability.
Use the RPM graph to inspect overshoot and dwell; network telemetry may miss short dips.

Simulation decision for this integration: use existing Mechanisms/Competition
regressions for bench interlocks and unchanged competition defaults, plus ordinary
fake-hardware tests for live hub coefficients, bounds, voltage compensation and
unchanged-setting writes. SimLoop has no Android Dashboard/WebSocket service or
calibrated shot-load model; browser editing, graphs and physical recovery require
Control Hub testing. No new OpMode or simulation scenario is introduced.

Verification on November 2, 2026: `tools/verify-teleop.ps1 -Sim -Scenario
Mechanisms,Competition` passed 187 ordinary tests, debug APK packaging, lint
(zero errors, 14 existing warnings), and 12 selected simulation tests. This includes
the actual bench load/feed/STOP scenarios and both competition wrappers. The new
`ShooterLiveTuningTest` checks live coefficient updates without restarting the RPM
command, unchanged-setting write suppression, invalid/bounded gains and retained
voltage compensation. Dashboard/browser operation and loaded physical tuning were
not run on a Control Hub.

## Controller comparison and loaded trials

Bang-bang is available only through the combined bench test's controller toggle.
It switches the motor to RUN_WITHOUT_ENCODER and commands full power below the
selected RPM, coasting at or above it with FLOAT. It uses each main loop's cached
encoder sample; the hub PIDF loop is not active in that mode. No I/D gains or
deadband delay its maximum-power recovery. GP2 gain adjustments are retained for
switching back to PIDF but do not affect bang-bang. Other OpModes still start in
PIDF. Switching resets minimum-feed-RPM and return-time measurements. Compare
both modes with identical target, battery and loading; last return-to-tolerance
time includes spin-up and is not a measured ball-to-ball recovery time. These
settings cannot be called optimized until loaded trials confirm sag, overshoot,
recovery time and shot distance.

First verify unloaded speed is steady at the intended target. During repeated
loaded trials, adjust P in small steps and compare RPM sag and overshoot. Reduce P
if oscillation increases. Adjust feedforward scale in small steps when the
steady-state speed remains consistently biased. Change one setting at a time.
If speed remains below target under sustained feed despite reasonable P/F, add I
in small steps; reduce it again if recovery overshoots or oscillates. Battery
sampling remains at 5 Hz, while the hub's velocity feedback runs continuously.
Unchanged gains are not rewritten on every battery sample; F is updated when its
computed value changes by at least 0.02 or the operator explicitly adjusts gains.
If the voltage-adjusted speed estimate is near 100%, lowering the target or
reducing the ball feed rate may be necessary; PID gains cannot supply additional
motor power. RT + LT now repeats the competition 0.50-s feed / 0.35-s recovery cycle, with bench readiness required between attempts.

Hood home is shifted 2 hood degrees down from the original servo position 0.5,
to approximately 0.511268 (2:1 linkage, 355-degree servo travel). This is the old
13-degree command position, now designated the team's +15-degree home reference.
The full 30-degree downward range ends near 0.680282. D-pad Up still increases
downward travel; Down returns toward home. This is a commanded angle reference,
not an independent measurement of projectile launch angle. Calibration displays averages without saving files. The unused historical export parser retains version checks for earlier references.
