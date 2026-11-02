# TeleOp verification matrix

Updated November 2, 2026. See [commands and human review](../simloop/README.md),
[model provenance](SimLoopRobotModel.md), [audit and remaining boundaries](SimLoopIntegrationAudit.md).
Simulation remains optional for major changes; minor edits do not require it. Local checks first, CI deferred.

## Current automated coverage

All rows retain their existing pure-controller JUnit 4 regressions. A lifecycle smoke test only proves
initialization/loop/STOP under its stated fixture; it does not certify every control or fault branch.

| Capability | Actual OpMode simulation | Assertions / remaining boundary |
| --- | --- | --- |
| Drivetrain | DrivetrainTestOpMode, both competition wrappers | Forward/reverse/strafe/yaw, normalization, wheel selection, neutral/STOP, alias/missing hardware, 10/20ms, repeat decoded traces; geometry/dynamics synthetic |
| Gate | GateTestOpMode, TransferTestOpMode, both wrappers | Logical/physical direction, PWM, delayed feed, held/reopened gate, trigger priority, shutdown, missing/PWM-incapable device |
| Intake | IntakeTestOpMode, bench, both wrappers | Forward/reverse/both/neutral, full queue stop, independent shaft encoder, shutdown; bench-only 0.5-second jam recovery has controller tests, no ball-contact simulation |
| Transfer | TransferTestOpMode, bench, both wrappers | Two-second HIGH dwell, stage1-only loading, timed gate/both-stage feed, inhibit priority, STOP |
| Depollinator | DepollinatorTestOpMode, both wrappers | Deploy/home edges, home priority, PWM range/endpoints; holds commanded position at STOP |
| Turret / hood / flywheel | TurretTestOpMode, bench, both wrappers | Independent finite-rate yaw on transfer motor 1's encoder input, SDK REVERSE sign normalization, paired servo commands, hood held-button edge, shaft response, startup and STOP; loaded response unmeasured |
| Pinpoint | PinpointTestOpMode smoke, full competition fixture | SDK fake calibration/pod configuration and chassis coupling; NOT_READY fault blocks synthetic automatic shots; no physical pod calibration certified |
| Limelight / goal state | LimelightTagTestOpMode, GoalStateTestOpMode smoke | Geometric camera and explicit raw-versus-normalized target basis; physical convention unresolved |
| Localization | Both actual competition wrappers with normalized camera | Mapped-tag acquisition from unknown start and retention after camera loss; raw convention rejection retained; production automatic feed remains blocked |
| Automatic shot success | Test-only SyntheticRedTeleOp / SyntheticBlueTeleOp inheriting competition loop | Artificial calibrated rows exercise feed then Pinpoint-fault blocking; no production measured flags altered, no ball scoring |
| Competition integration | RedMecanumTeleOp / BlueMecanumTeleOp | Manual drive before reference, queue/feed/inhibit, early START, STOP before START, partial-init cleanup, 180 simulated seconds each |
| Bench | IntakeShooterTestOpMode | Actual intake/stage-1 loading, LB reverse and both-bumper stop, full dwell/hold after sensor clears, qualified RT+LT two-stage feed after gate delay, release and RB reload/STOP; modeled feedback, no physical readiness claim |
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
