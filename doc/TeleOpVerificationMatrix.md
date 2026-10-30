# TeleOp verification and evidence

October 28, 2026. First milestone of [the implementation plan](SimLoopTeleOpImplementationPlan.md).
This is a software coverage record. Existing operator evidence remains intact; simulation does not supply
new calibration or physical sign-offs. The rest of the plan is progressive work, chosen around major
changes or reported inconsistencies, with CI deferred.

## Baseline and scope

Baseline revision: `d020024b94b14b5e756861d6fc297319705688ad` on `V1`, including the existing uncommitted
working tree. The initial tracked diff had 23 paths, 405 insertions and 744 deletions, plus untracked
Pedro/Zenith adapters, autonomous assets/docs/scripts and localization tests. The initial status included
changes to `MecanumTeleOp`, `PoseHistory`, `ShotCoordinator`, `TeleOpFieldReference`, drivetrain and
autonomous code. These were preserved. No commit, deployment or robot movement is part of this work.

The current `CompetitionTeleOp.md` already specifies arbitrary autonomous-finish position/heading,
acquisition from either alliance's mapped tags and selected-alliance firing targets. Its former fixed
blue-corner assumption was not restored. The simulation foundation does not change any competition
controls, shot measurements, automatic calibration gates or autonomous behavior.

Fresh pre-integration checks: **154 JUnit 4 tests in 23 classes**, zero failures/errors/skips; debug APK
packaged; lint **0 errors, 14 warnings** (12 native-library alignment warnings and 2 unused resources).
The named cases are recorded in [the discovery manifest](../tools/teleop-test-baseline.json), including
autonomous/Pedro/pollen regressions. Historical 153/159-test documentation is not this working-tree baseline.
Gradle 9.1.0, AGP 8.13.2 and FTC 12.0.0 resolved successfully. This host selected Java 25; Java 8
source/target obsolescence and existing Gradle deprecation notices remain.

## Coverage matrix

| Function | Production owner | Actual diagnostic OpMode | Desktop evidence | Hardware evidence/date | Remaining physical measurements/checks | Competition integration |
| --- | --- | --- | --- | --- | --- | --- |
| Robot-centric drive | `MecanumDrivetrain` | `DrivetrainTestOpMode` | SimLoop INIT/START/ACTIVE/STOP, forward/reverse/strafe/CW, normalization, wheel selection, neutral, aliases, missing device, STOP before START, 10/20 ms, repeat decoded traces | No new drive sign-off recorded here | Loaded signs/response, actual dimensions and dynamics, loop periods | Existing shared red/blue drive; actual competition simulation deferred |
| Gate PWM/delay | `GateSubsystem` | `GateTestOpMode` | SimLoop logical .5/.2, physical .5/.8 and 1500/2100 us with REVERSE and 500-2500 us range; held-open delay, close/reopen reset, trigger threshold, STOP, missing/PWM-incapable servo, 10/20 ms | Operator gate test good, Oct 26 | Verify unchanged installed direction/endpoints; loaded clearance and 0.20-s feed delay remain unmeasured | Same SDK interface path as competition; combined transfer integration still controller-tested |
| Odometry | `PinpointOdometry` | `PinpointTestOpMode` | Existing pure pose/reference tests; no Pinpoint SimLoop model yet | No numerical odometry sign-off supplied in this record | Pod signs/resolution/offsets, units and actual loop refresh | Existing competition local sample; arbitrary-start field acquisition remains intact |
| Intake | `IntakeSubsystem`, `IntakeQueueController` | `IntakeTestOpMode` | Existing queue/controller regressions; no actual-OpMode simulation yet | All intake tests complete, Oct 26 | No supplied new inconsistency; future wiring/tuning changes invalidate affected evidence | Existing competition full-hold and bumper overrides |
| Transfer | `TransferSubsystem`, `TransferController`, `FiringCoordinator` | `TransferTestOpMode` | Existing chatter/full/latch/feed/inhibit/recovery and configuration regressions; actual hardware sequencing simulation deferred | All transfer tests complete, Oct 26 | Loaded gate/feed/recovery timing; no invented jam thresholds/policy | Existing stage-1 loading/stage-2-off policy retained |
| Depollinator | `DepollinatorSubsystem` | `DepollinatorTestOpMode` | Existing geometry/source checks only; no actual-OpMode simulation | No new evidence recorded | Deployed clearance, positional STOP policy, loaded travel | Existing stick-click controls retained |
| Turret/encoder | `TurretSubsystem`, `TurretControl`, `TurretHomingController` | `TurretTestOpMode` | Existing homing, bounds, release/correction, alliance tests; coupled servo/encoder simulation deferred | Assembly, alignment, encoder and mechanism checks Oct 24; rotation good Oct 26 | Loaded speed, settling/overshoot, stopping distance, encoder accuracy; no absolute home sensor | Existing manual/aim release behavior retained |
| Hood/flywheel | `TurretSubsystem`, `ShooterSpeed`, `ShooterVelocityTuning` | `TurretTestOpMode`, `IntakeShooterTestOpMode` | Existing bounds, conversion and speed/controller tests; no response/load/voltage model yet | Hood/shooting mechanism good Oct 24/26 | Measured shot rows, RPM/hood settings, loaded recovery and timing | Production profile remains example; measured flags remain false |
| Limelight/goal diagnostics | `LimelightTagSubsystem`, `GoalStateObserver` | `LimelightTagTestOpMode`, `GoalStateTestOpMode` | Real JVM JSON parsing verified; existing geometry/goal tests. No fake-camera scenario yet | No new camera sign-off | Pipeline/tag size/3D settings, mount, latency, transforms and raised-reference acquisition on field | Diagnostic goal-state policy remains distinct from competition |
| Field aim/localization | `ShotCoordinator`, `PoseHistory`, `TeleOpFieldReference` | `AutoAimTestOpMode` | Existing all-tag acquisition, delayed-history, wrap/reference/velocity and camera-loss controller tests; no actual-OpMode SimLoop integration yet | No new automatic-aim sign-off | Field map/CAD geometry, capture timing, known-pose/tag/turret-angle acquisition and dropout checks | Both existing alliance wrappers preserved; actual red/blue simulation NOT RUN |
| Calibration/raw programmer | `ShotCalibration`, `ServoZeroCalibrationOpMode` | `ShotCalibrationOpMode`, `ServoZeroCalibrationOpMode` | Existing calibration/legacy export-helper tests; raw controller metadata simulation deferred | Mechanism tests do not provide shot rows | Actual session measurements; mechanically disconnected raw-servo procedure | Current calibration only displays averages; no new file-save or profile activation |
| Combined bench | `IntakeShooterTestController` | `IntakeShooterTestOpMode` | Existing bench controller tests, no actual-OpMode simulation | Intake/transfer/shooting/gate evidence above | Preserve intentional bench readiness and loaded timing checks | Bench controls must not replace competition controls |
| Loop/shutdown | `RobotLoop`, `RobotShutdown` | Owning OpModes | Empty Lynx inventory works; drivetrain/gate each flush once per active tick; existing stop-all failure regression | No Control Hub distribution measured here | Cache ownership on actual hubs; p50/p95/p99/max period at simultaneous load | 50-100+ Hz is a robot target, not a simulated benchmark |

## Intentional diagnostic differences

| Policy | Competition red/blue | Intake/index/shooter bench |
| --- | --- | --- |
| Flywheel | 3500 output RPM; Y/A steps 50 RPM; B unassigned | RT held; 2800 initially; Y/A steps 100 RPM; B stops motors |
| Full queue intake | Stop automatically; bumpers override | Reverse intake while full during loading |
| Manual feed | GP1 LT or GP2 RT overrides shot readiness; GP2 LT inhibits | RT + LT also requires bench readiness; timed feeding |
| Yaw readiness | 3-degree angle check; no motion/stationary gates | 4 degrees and 3 degrees/s |
| Release/correction | Capture measured yaw on release; bounded correction enabled | Diagnostic commanded-target hold; correction disabled by default |

Both policies retain stage 2 off while loading and until the gate's 0.20-s command delay elapses.
Production automatic feeding remains inhibited without measured shot profile/release timing. Synthetic
fixtures have not enabled those flags or introduced camera-loss/motion-based firing gates.

## Implementation evidence

The first suite has **11 Jupiter tests** (6 drivetrain, 4 gate, 1 real-JSON runtime), with multiple named
scenario runs inside those tests. It uses the published `v0.2.0-alpha.2` artifact, test-only PsiKit,
Vintage/Jupiter 5.11.3, Platform launcher 1.11.3 and real JSON 20250517. No desktop UI artifact is needed.
Production runtime dependency inspection excludes SimLoop, PsiKit, JUnit and JSON's JVM-only artifact.
Ordinary unit tests exclude only the designated simulation package and retain the 154 baseline cases.

Final full verification passed **154 ordinary tests and 11 simulation tests**, zero failures/errors/skips,
debug APK packaging and lint **0 errors, 14 existing warnings**. The full suite passed repeatedly.
Selected `-Sim -Scenario Gate` execution passed 5 tests (gate plus runtime), with drivetrain simulations
excluded. Invalid simultaneous `-Sim -FullSim` options return exit code 1. Ordinary mode does not run sim.

Critical assertions were mutation-checked: removing forward-Y negation failed with expected +1 versus
actual -1 wheel power; resetting the gate timer every open loop failed with expected ready versus false.
Both temporary changes were restored before the final suite. Gate telemetry now reports feed eligibility;
its endpoints, direction, pulse range, timer and command-to-feed delay are preserved. Gate hardware is
retrieved as `Servo` with an explicit `PwmControl` capability requirement, rather than concrete `ServoImplEx`.

Generated evidence: Gradle XML/HTML under `TeamCode/build/test-results/` and `TeamCode/build/reports/`,
unique decoded scenario traces under `TeamCode/build/sim/teleop/`, and local verification/dependency/mutation
logs under `build/teleop-verification/`. Inputs, phase/tick, model parameters, physical wheel response,
gate PWM and telemetry are retained; assertion reports provide expected/actual results and trace paths.

## Tuning and inconsistency backlog

No additional robot defect was reported in this request. These are known measurement/coverage boundaries,
not presumed failures in working mechanisms.

| Symptom / reproduction | Expected behavior | Mechanism | Available evidence | Smallest useful next check |
| --- | --- | --- | --- | --- |
| Automatic calibrated feed unavailable with GP1 RT under example configuration | Continue to inhibit until actual shot table/timing are supplied | Hood/flywheel/feed | Existing controller tests and unchanged false measurement flags | Collect distance/average output RPM/labeled launch angle rows and loaded release timing |
| Arbitrary-start acquisition not physically validated | Manual drive/feed remain available while waiting; accepted raised tags establish field reference | Camera/Pinpoint/aim | Existing all-tag desktop geometry/localization tests | Known field poses/headings and turret angles, then cover/move/uncover camera |
| Gate timing verified only as command elapsed time | No feed before command delay; adequate actual clearance at feed start | Gate/transfer | Actual gate OpMode PWM/timer simulation plus prior operator mechanism check | Measure loaded opening/feed/recovery timing; do not treat PWM command as position feedback |
| No real loop distribution available | Responsive drive during simultaneous mechanism work | Loop/hubs | Serial sim checks command response, not bus/work latency | Record Control Hub p50/p95/p99/max with representative load |
| Full-robot harness not implemented | Add coverage for affected major changes when useful | Combined red/blue | Existing pure integration/controller regressions; first milestone only | Add independent turret feedback, Pinpoint and camera models before claiming competition success |

Later phases remain explicitly unimplemented: full robot named-device fixtures, turret/PWM capability
cleanup and feedback model, flywheel load/voltage response, Pinpoint/camera models, other diagnostic
lifecycle scenarios, calibrated synthetic success seam, red/blue actual competition suite and simulated
180-s endurance. Webcam/pollen, autonomous behavior changes, real hardware performance and CI are outside
this milestone. Full-robot simulation is not a prerequisite for routine tuning or minor fixes.
