# Autonomous: Pedro Pathing v3, Ivy, and Zenith

Every registered autonomous now extends Zenith's Ivy `AutoFromFile`. `BioBuzzRobot` connects the runtime to the existing hardware subsystems and `ShotCoordinator`. `WaypointFollower`, `CompetitionAutonomousController`, and `AutonomousStrategy` have been removed. The pollen navigator is an experiment in the **Test** group, separate from the real match routines.

The remaining command coverage, UI project organization, generation policy and verification work are specified in [the complete Zenith implementation plan](ZenithAutonomousImplementationPlan.md).

## Installed versions

| Component | Version / repository |
| --- | --- |
| Pedro core | `3.0.0-20260828.185437-17`, Sonatype snapshots |
| Ivy Pedro adapter and core | `1.1.1`, Maven Central |
| Zenith robot runtime | `v0.1.2`, JitPack |
| Zenith CLI | `0.1.2`, npm, locked in package-lock.json |

These are the versions documented and tested together by [Zenith](https://github.com/Horizon-36596/zenith/blob/v0.1.2/robot/README.md). This Pedro v3 API uses `Paths`, `Pose`, and `Follower.follow`; v1/v2 examples using `PathChain` do not apply. Keep the exact Pedro version in TeamCode/build.gradle and autos/robot.json identical. The npm override pins patched Zod 4.6.6 for the CLI; it does not enter the APK.

## Before robot motion

**Pedro tuning has not been measured on this robot.** `PedroConfiguration.TUNING_VERIFIED` is false. All autonomous routines refuse to start until dynamics, braking and gains are measured and verified. The initial values in Java and robot.json are labeled examples, not robot measurements. Camera and shooting calibration gates also remain active.

Use **Test: Pedro Drive**, which initializes only the drive subsystem (four motors and Pinpoint). Keep the chassis stationary through calibration. Sticks command at most 0.2 power, A requests 12 inches forward from the current pose, B requests a 90-degree CCW hold, and X cancels. Stick movement cancels a running test. A/B use Ivy commands, have a five-second timeout, and stop on completion. This diagnostic deliberately allows testing before TUNING_VERIFIED is set.

Verify forward, left and CCW motion and the matching odometry signs. Measure maximum forward/strafe velocities under the configured wheel-power cap, natural deceleration, linear/quadratic braking, rotational braking, and feedback gains using [Pedro's tuning procedures](https://pedropathing.com/docs/pathing/tuning). Update `PedroConfiguration.create()` and the matching kinematics/follower values in autos/robot.json. The current wheel cap is 0.35. Adjust the diagnostic's manual limit deliberately when measuring at that cap. Measure robot footprint and height too; the editor currently has explicit 18-inch example dimensions. Set TUNING_VERIFIED only after low-speed path, hold, stop, and turn checks pass.

The pinned Pedro snapshot normalizes a zero displacement inside ForesightV3's hold calculation. `PedroForesight` guards only that numerical case using a 0.0001-inch offset, below Pedro's translation deadband, and delegates the control calculations to Pedro. Regression tests exercise exact-position holds and in-place turns.

## Author and deploy match routines

Open the repository's zenith.json in Zenith. It selects Ivy and `org.firstinspires.ftc.teamcode.BioBuzzRobot`. The project includes Zenith's BIOBUZZ field model, attributed below. Verify field geometry and robot dimensions against the actual setup before planning paths.

No competition route has been supplied. **Auto: Red Match (Route Required)** reads `red-match.auto.json`; **Auto: Blue Match (Route Required)** reads `blue-match.auto.json`. Until those assets exist, Zenith fails during INIT with the missing filename, before initializing robot hardware. No replacement match coordinates were invented.

1. Create routes in Zenith under autos/. The file's name must match its stem, such as `red-match`. Use explicit, measured start poses and waypoints in inches, field center origin, +x audience right, +y away from the audience, CCW radians.
2. Compose paths with the named commands below. Zenith's runtime preserves supported curves, heading modes, distance markers, sequences, parallel all/race/deadline groups, branches, waits, and current-pose starts. It validates names and file versions during INIT.
3. Install CLI dependencies once with `npm ci --ignore-scripts` (Node 22+). Run `npm run zenith:validate` and `npm run zenith:deploy -- --dry-run`, then `npm run zenith:deploy`. Deploy writes local APK assets and generated Ivy OpMode stubs; it does not install anything on the Control Hub.
4. Run `.\\gradlew.bat :TeamCode:testDebugUnitTest :TeamCode:assembleDebug`. Install the resulting APK through the team's normal robot workflow.

Existing match entry points can load updated files without Java edits. New routines get generated OpModes in the Generated group. The generated robot uses the file's alliance; the explicit red/blue entry points use their selected alliance. Opposite-alliance files mirror position and heading through field-center point symmetry.

Each path's speed fraction is applied through `ForesightConfig.setPathSpeed`, in addition to the independent wheel-power cap. Estimates remain provisional until Java and robot.json contain measured, matching values. Put timeouts/end conditions on path and command steps. Parking and fallback decisions belong in the authored routine; the old controller's automatic park fallback was removed with it. The runtime does not provide live obstacle avoidance. A global 29-second deadline, localization loss, routine completion, Driver Station STOP, and exceptions stop outputs and close the gate. Explicitly canceling the Ivy routine ensures running commands receive their end hooks before scheduler reset. Camera shutdown follows motor shutdown.

## Named command contract

| Name | Arguments | Behavior / requirements |
| --- | --- | --- |
| intakeOn / intakeOff | none | Enable/stop intake; confirmed full stops intake. Requires intake. |
| loadOn / loadOff | none | Enable/pause stage 1 loading. Requires transfer. Stage 2 stays off until gated shooting. |
| aim | side: AUDIENCE (default) or FAR | Select alliance hive side; continuously update autonomous odometry aiming. Requires shooting. |
| shoot | seconds: >0, <=28 | Request readiness-gated firing for the duration. Requires shooting, gate, transfer. No manual feed bypass and no exact ball count. |
| collectPollen | seconds: >0, <=28 (default 28) | Calibrated vision experiment using Pedro paths and holds. Requires drive, intake, transfer, gate and available C270. |
| stopAll | none | Stop drive, intake, transfer and turret/flywheel; close gate. Use at the end of a routine. Requires all outputs. |

Conditions: `queueFull`, `pollenVisible`, `localizationReady`, `turretHomed`. Registry declarations in autos/robot.json match Java registrations, checked by Zenith's Java scanner during validation. aim/shoot fail during INIT if requested by the isolated pollen test, which does not initialize turret hardware. Gate opening stays inside the shared firing coordinator.

Aim can remain active while a path runs. Only shoot requests feeding. Existing homing, mount/profile/timing calibration, pose, hood, yaw and RPM readiness still apply. Transfer loading requires loadOn; intake requires intakeOn, or the collection command. Timed attempts are not inferred piece counts.

## Architecture and verification

`PedroDrive` owns the existing mecanum and Pinpoint subsystems. It selects open-loop motor power for Pedro during INIT, preserving existing aliases, cloned bare-motor specs, directions and BRAKE. TeleOp keeps its existing mode. Its Pinpoint adapter converts cached meters to inches without changing field axes or rotating global velocity twice. Start-pose writes wait until Pinpoint is READY so IMU calibration cannot overwrite them. Each loop clears REV caches, reads Pinpoint once, updates the turret, updates Pedro once, runs Ivy, applies mechanism outputs, and flushes telemetry once.

Tests cover unit conversion, mecanum signs and saturation, actual Pedro hold/path-speed behavior, the pollen planner and replanning, the packaged pollen asset, and Zenith curves, markers (including endpoint and interrupted markers), parallel timeouts, branches, deferred starts, cancellation, and invalid files. These are JVM checks, not evidence of physical robot tuning.

The test asset is maintained directly at TeamCode/src/main/assets/autos/pollen-navigation-test.auto.json so the hand-written Test-group OpMode stays isolated; competition deploy does not generate a second pollen OpMode. Its cleared-area field model is autos/field/pollen-test.field.json, with a packaged copy beside the test asset. If that model changes, copy it into the asset directory too. Its intentional lack of a season plugin disables competition scoring/ledger checks; the CLI prints that notice. The 28-second timeout matching the bounded experiment may also produce TIMEOUT_TIGHT information.

Field model provenance: autos/field/biobuzz.field.json was copied unchanged from Horizon-36596/zenith v0.1.2, commit f7c3681a158c6f504ab211464559d52da8a46955, examples/starter/autos/field/biobuzz.field.json. Its per-feature provenance distinguishes specifications from approximations. [Zenith's MIT license](ZenithFieldLicense.txt) accompanies the copy.
