# TeleOp verification matrix

Updated November 2, 2026. See [commands and human review](../simloop/README.md),
[model provenance](SimLoopRobotModel.md), [audit and remaining boundaries](SimLoopIntegrationAudit.md).
Simulation remains optional for major changes; minor edits do not require it. Local checks first, CI deferred.

Conflict-resolution verification (November 2, 2026): retained Dashboard live gains,
bench jam recovery, automatic intake/loading, selectable shooting modes and timing,
and turret feedback on transfer motor 1. Simulation was selected to check their
combined behavior: 202 ordinary tests, debug APK packaging and lint passed; after
reconciling the isolated turret scenario with the already-configured 8-degree hood
home offset, all 13 Mechanisms/Competition/runtime simulations and manifest entries
passed. Jam-recovery tests now distinguish automatic loading from stopped/inhibited
requests and verify that releasing RB preserves recovery when direction is unchanged.
No production calibration or simulation assertions were relaxed. Full simulation and
physical robot testing were not run; ball contact and loaded clearance remain unmodeled.

Sensor recovery and bumper overrides (November 1, 2026): this supersedes the earlier
full-latch/intake-stop policy below. Continuous HIGH for 0.5 s reverses automatic
intake; stage 1 runs on for 0.5 s and then stops while HIGH persists. LOW immediately
restores forward intake and loading. RB forces intake forward, LB backward, both
stop, and releasing bumpers restores automatic control. Bench automatic loading
starts after its release-to-arm interlock; stop and shooting pauses remain effective.
Simulation was selected for this shared behavior change:
`verify-teleop.ps1 -Sim -Scenario Mechanisms,Competition` passed 191 unit tests,
13 simulations, debug packaging and lint. Existing bench and actual red/blue scenarios
assert sensor recovery and overrides alongside isolated transfer timing; no new
scenario or OpMode was added. These are command/timing checks, not physical ball,
electrical delivery, or sensor-placement validation.

Bench-only shooting sequence (November 1, 2026): simulation was selected for the
0.10 s stage-2 gate delay, 0.40 s stage-2 feed window, stage-1 gate-open midpoint
start and immediate continuous feed. `verify-teleop.ps1 -Sim -Scenario
Mechanisms,Competition` passed 191 unit tests, 13 simulations, debug APK packaging
and lint (0 errors, 14 existing warnings). The actual bench cycle exercises the
stage ordering; the added `benchContinuousFeedsImmediatelyUntilTriggerReleaseOrStop`
scenario exercises GP2 selection, immediate outputs before flywheel qualification,
held feed, either-trigger release and B/re-arm. Both actual competition wrappers
retain their defaults. Timers and commanded outputs do not prove one-ball
separation, physical gate clearance or loaded recovery.

Stage-1 run-on verification (November 1, 2026): simulation was selected for this shared
sequence change. `verify-teleop.ps1 -Sim -Scenario Mechanisms,Competition` passed
189 unit tests and 12 simulations. Existing scenarios assert bench intake reversal
with stage 1 still forward and stage 2 off, the later stage-1 stop, isolated transfer
timing, and both actual competition wrappers retaining their intake-stop policy.
Controller regressions cover exact 0.5-second boundaries, LOW reset, latched chatter,
and pause cancellation. No new scenario or OpMode was added. These checks validate
commands and timing, not loaded ball travel, sensor placement, or electrical delivery.

## Current automated coverage

All rows retain their existing pure-controller JUnit 4 regressions. A lifecycle smoke test only proves
initialization/loop/STOP under its stated fixture; it does not certify every control or fault branch.

| Capability | Actual OpMode simulation | Assertions / remaining boundary |
| --- | --- | --- |
| Drivetrain | DrivetrainTestOpMode, both competition wrappers | Forward/reverse/strafe/yaw, normalization, wheel selection, neutral/STOP, alias/missing hardware, 10/20ms, repeat decoded traces; geometry/dynamics synthetic |
| Gate | GateTestOpMode, TransferTestOpMode, both wrappers | Logical/physical direction, PWM, delayed feed, held/reopened gate, trigger priority, shutdown, missing/PWM-incapable device |
| Intake | IntakeTestOpMode, bench, both wrappers | Forward/reverse/both/neutral, confirmed-HIGH reversal, LOW recovery, bumper overrides, independent shaft encoder, shutdown; bench-only 0.5-second jam recovery has controller tests, no ball-contact simulation |
| Transfer | TransferTestOpMode, bench, both wrappers | 0.5-second HIGH dwell, 0.5-second stage-1 run-on, stage1-only loading, timed gate/both-stage feed, inhibit priority, STOP |
| Depollinator | DepollinatorTestOpMode, both wrappers | Deploy/home edges, home priority, PWM range/endpoints; holds commanded position at STOP |
| Turret / hood / flywheel | TurretTestOpMode, bench, both wrappers | Independent finite-rate yaw on transfer motor 1's encoder input, SDK REVERSE sign normalization, paired servo commands, hood held-button edge, shaft response, startup and STOP; loaded response unmeasured |
| Pinpoint | PinpointTestOpMode smoke, full competition fixture | SDK fake calibration/pod configuration and chassis coupling; NOT_READY fault blocks synthetic automatic shots; no physical pod calibration certified |
| Limelight / goal state | LimelightTagTestOpMode, GoalStateTestOpMode smoke | Geometric camera and explicit raw-versus-normalized target basis; physical convention unresolved |
| Localization | Both actual competition wrappers with normalized camera | Mapped-tag acquisition from unknown start and retention after camera loss; raw convention rejection retained; production automatic feed remains blocked |
| Automatic shot success | Test-only SyntheticRedTeleOp / SyntheticBlueTeleOp inheriting competition loop | Artificial calibrated rows exercise feed then Pinpoint-fault blocking; no production measured flags altered, no ball scoring |
| Competition integration | RedMecanumTeleOp / BlueMecanumTeleOp | Manual drive before reference, queue/feed/inhibit, early START, STOP before START, partial-init cleanup, 180 simulated seconds each |
| Bench | IntakeShooterTestOpMode | Actual intake/stage-1 loading, LB reverse and both-bumper stop, confirmed-HIGH reversal and immediate LOW recovery; staggered stage-2 feed after 0.10 s gate delay with stage 1 starting at the gate-open midpoint; GP2 continuous selection, immediate both-stage full-power feed, either-trigger release and B/re-arm; RB reload/STOP. Scenarios: `benchLoadsHoldsFullFeedsAndReloadsThroughActualMotorOutputs`, `benchContinuousFeedsImmediatelyUntilTriggerReleaseOrStop`. Modeled feedback, no physical clearance or ball-spacing claim |
| Aiming / calibration diagnostics | AutoAimTestOpMode, ShotCalibrationOpMode | Real lifecycle smoke; detailed planner/calibration logic remains covered by ordinary tests; calibration does not create real measured rows in simulation |
| Pedro diagnostic | PedroDriveTestOpMode | Idle lifecycle/STOP, Pedro package instrumented for simulated time; no autonomous path-accuracy claim |
| Raw disconnected-servo programmer | Not simulated | Upstream FakeServo has no ServoController metadata; use actual disconnected-servo procedure |
| Pollen / webcam | Not simulated | VisionPortal/USB/webcam processing unsupported by this harness; existing controller tests and hardware evidence only |
| Zenith UI-authored autonomous tree | Ordinary native-runtime regressions, outside `simTeleOpTest` | Actual saved UI JSON exercises curve/speed/marker, parameterized actions, bounded condition wait and stop through native AutoBuilder/Ivy. See [UI record](ZenithUiAcceptance.md); this is not autonomous chassis physics or camera capture coverage. |

## Physical evidence retained

November 2 encoder-wiring verification: `verify-teleop.ps1 -Sim -Scenario
Mechanisms,Sensors,Competition` passed 195 ordinary tests and 15 existing simulations,
debug APK packaging and lint. Fixtures now put turret feedback on transfer motor 1
and intake feedback on its own shaft. Jam recovery remains bench-only; its timing
and cancellation have controller tests, with physical jam clearing unverified.

The [hardware-review record](HardwareReviewImplementationStatus.md) records intake/transfer completion
October 26, turret rotation/hood/shooting checks October 24/26 and gate mechanism good October 26. This audit
does not revoke unchanged sign-offs or turn them into numerical calibration. New mechanics, wiring,
direction, limits or timing changes invalidate the corresponding evidence.

Still required on the robot: camera target-axis sample and known-pose acquisition; measured shot table
and release timing; loaded gate clearance/feed timing; drive/turret/flywheel dynamic measurements;
Control Hub loop-period p50/p95/p99/max under simultaneous load. Fixed sim ticks cannot benchmark I2C,
USB, Lynx bulk caching, scheduling, voltage sag or thermal behavior.

## Evidence and review policy

JUnit XML/HTML records assertions. Scenario TSV records inputs, commands, physical model states and
telemetry; runner-outcome.txt is not a behavioral verdict. Human recordings include environment events,
profile and STOP tick and can be replayed with exact decoded-output comparison. Record source revision
and dirty changes alongside reviewer notes. The baseline manifest prevents silent test-discovery loss.

Competition and bench policies intentionally differ: competition starts at 3500 flywheel output RPM,
uses GP1 LT / GP2 RT manual feed and GP2 LT inhibit; bench uses its dedicated readiness/release controls.
Keep those policies distinct when promoting a subsystem change. Production shot rows remain examples;
synthetic success tests do not authorize automatic shots on the robot.

## November 1, 2026 flywheel hardware update

The reported shooter is a goBILDA 5203 Yellow Jacket 6000 RPM motor with 1:1
motor-to-flywheel drive. This replaces the previous 2:3 conversion and 3700 RPM
ceiling. All shooter commands may request 6000 RPM; no lower measured-speed cap
or recovery reserve is imposed. Existing bench/competition output-RPM presets
and readiness tolerances are retained. Actual-versus-target RPM telemetry is the
basis for the team's upcoming physical maximum-speed measurement.

Simulation decision: run the existing Mechanisms and Competition scenarios via
`tools/verify-teleop.ps1 -Sim -Scenario Mechanisms,Competition`, including the
isolated TurretTestOpMode, combined bench test and both actual alliance wrappers.
The independent fake flywheel now uses the catalog 6000 RPM / 28 ticks-per-rev
plant; the drivetrain remains at its separate 5550 RPM baseline. Ordinary tests
check 1:1 encoder conversion, full 6000 RPM command/profile/calibration acceptance,
out-of-range clipping and feedforward. No new scenarios or OpModes were added.
These checks do not measure loaded recovery, reachable RPM, ball flight or a new
shot table. Production calibration flags remain unchanged.

Verification result: 185 ordinary tests and 12 selected simulations passed with
zero failures, errors or skips; debug packaging and lint passed. Evidence:
`build/flywheel-1to1-verification.log` and JUnit reports under
`TeamCode/build/test-results/`. Existing Java/source compatibility and Gradle
deprecation warnings remain. `git diff --check` and matching autonomous registry
copies were also checked. Physical maximum RPM is pending team telemetry data.

Follow-up physical evidence: team reports 4700 RPM maximum at a 6000 RPM command,
plus 100–200 RPM underspeed at operating targets, blocking the existing 100 RPM
readiness check. Controller mode, battery voltage and loading are pending. This
is an observation, not a voltage-normalized free-speed calibration. Added explicit
error/tolerance/readiness telemetry; command limits and gains remain unchanged.
Simulation omitted for observation/telemetry-only changes; regulation changes await
the missing measurement conditions. See ShooterVelocityTuning.md.

Follow-up verification: 185 unit tests, debug packaging and lint passed. Evidence: build/flywheel-speed-observation-verification.log. Simulation was not run for these telemetry-only edits.

## November 1 measured-speed feedforward follow-up (superseded observation)

The team clarified the observation: 4700 RPM maximum at 14 V in both PIDF and
bang-bang modes. Updated PIDF feedforward and speed-usage telemetry to reference
that installed-speed observation, with linear voltage scaling; command ceiling
remains 6000 RPM and readiness remains +/-100 RPM. P/I/D and bang-bang law are
unchanged. The selected lower-target bang-bang trace and instantaneous power are
still needed to distinguish saturation from sampled cycling.

Simulation decision: reproduce with the existing bench transfer-cycle scenario,
then run Mechanisms and Competition (isolated turret, bench, actual Red/Blue).
The independent fixture now uses the reported 4700 RPM at 14 V rather than the
catalog 6000 RPM estimate. Response/drag/ball-load dynamics remain synthetic.
Before changing feedforward, the bench scenario failed its existing feed assertion
at MechanismsSimTest.java:105: physical feedback settled around 2550 RPM for a
2800 RPM command and feeding stayed blocked. Evidence retained in
`build/flywheel-measured-baseline.log`, `build/flywheel-measured-baseline-test.xml`
and `TeamCode/build/sim/teleop/bench-transfer-cycle-8605208065857636247/`.
No assertions, readiness tolerances or production calibration flags were relaxed.

After correction: 185 unit tests and 12 selected simulations passed with zero
failures/errors/skips; debug packaging and lint passed. The unchanged bench feed
assertion now passes. Final evidence: build/flywheel-measured-tuning-verification.log,
JUnit XML/HTML, and TeamCode/build/sim/teleop/bench-transfer-cycle-8177026944439584903/.
A first verification run caught the headroom telemetry formula still using the
catalog reference; corrected it to the observed reference without changing the
expected 4700 RPM at 14 V assertion. Physical tracking/overshoot and the bang-bang
underspeed report still require a robot trace at the selected operating target.

## November 1 maximum-speed correction: 5400 RPM

The team corrected the maximum to **5400 RPM**, superseding the earlier 4700 RPM
report. Retain the previously supplied 14 V condition. Earlier entries above are
historical test evidence, not current calibration. Updated the shared observation,
PIDF feedforward, speed-usage estimate, independent flywheel fixture, tests and
current tuning/model documentation. Command ceiling stays 6000 RPM, motor-to-
flywheel drive stays 1:1, and readiness tolerance stays +/-100 RPM. Feedforward is
approximately 13.003 at 14 V (15.170 normalized to 12 V), still a provisional
linear voltage/speed fit given the reported intermittent startup/encoder behavior.

Simulation decision: rerun existing Mechanisms and Competition scenarios against
the corrected independent 5400 RPM at 14 V fixture, including isolated turret,
bench feeding and actual Red/Blue competition wrappers. No new scenario or OpMode,
no assertion relaxation, and no production calibration flags changed. No claim
that this parameter correction fixes the intermittent physical startup issue.

5400 RPM correction verification: 185 unit tests and 12 Mechanisms/Competition simulations passed, zero failures/errors/skips; debug packaging and lint passed. Evidence: build/flywheel-5400-verification.log and JUnit reports. Existing Java/Gradle warnings remain. Physical speed regulation and startup repeatability remain unverified.
