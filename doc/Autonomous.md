# Autonomous: Pedro Pathing v3, Ivy, and Zenith

Every registered autonomous now extends Zenith's Ivy `AutoFromFile`. `BioBuzzRobot` connects the runtime to the existing hardware subsystems and `ShotCoordinator`. `WaypointFollower`, `CompetitionAutonomousController`, and `AutonomousStrategy` have been removed. The pollen navigator is an experiment in the **Test** group, separate from the real match routines.

Use the [Zenith authoring guide](ZenithAuthoringGuide.md) for the complete command catalog and supported deployment workflow. The [integration audit](ZenithIntegrationAudit.md) compares the code with the plan, official resources and current robot configuration. The [coverage ledger](ZenithFunctionCoverage.md) and [verification record](ZenithVerification.md) distinguish implemented software from pending editor and robot evidence. The original [implementation plan](ZenithAutonomousImplementationPlan.md) remains the acceptance specification.

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

No competition route has been supplied. The legacy red/blue Route Required wrappers are disabled. New authored match files each generate their own full-robot entry; the integration example is clearly labeled and supplies no competition strategy.

1. Create routes in Zenith under autos/. The file's name must match its stem, such as `red-match`. Use explicit, measured start poses and waypoints in inches, field center origin, +x audience right, +y away from the audience, CCW radians.
2. Compose paths with the named commands below. Zenith's runtime preserves supported curves, heading modes, distance markers, sequences, parallel all/race/deadline groups, branches, waits, and current-pose starts. It validates names and file versions during INIT.
3. Install CLI dependencies once with `npm ci --ignore-scripts` (Node 22+). Run `npm run zenith:validate` and `npm run zenith:deploy -- --dry-run`, then `npm run zenith:deploy`. Deploy writes local APK assets and generated Ivy OpMode stubs; it does not install anything on the Control Hub.
4. Run `.\gradlew.bat :TeamCode:testDebugUnitTest :TeamCode:assembleDebug` and `.\scripts\check-zenith-apk.ps1`. Install the resulting APK through the team's normal robot workflow.

New routines get exactly one generated OpMode in the Generated group; authors need no Java edits. The generated robot uses the file's alliance. Team deployment retains pollen's sole hand-written Test entry. Use UI edit/save followed by `npm run zenith:deploy`, rather than treating the editor's native Deploy button as completing team deployment. `npm run zenith:check` rejects stale or duplicate outputs.

Each path's speed fraction is applied through `ForesightConfig.setPathSpeed`, in addition to the independent wheel-power cap. Estimates remain provisional until Java and robot.json contain measured, matching values. Put timeouts/end conditions on path and command steps. Parking and fallback decisions belong in the authored routine; the old controller's automatic park fallback was removed with it. The runtime does not provide live obstacle avoidance. A global 29-second deadline, localization loss, routine completion, Driver Station STOP, and exceptions stop outputs and close the gate. Explicitly canceling the Ivy routine ensures running commands receive their end hooks before scheduler reset. Camera shutdown follows motor shutdown.

## Named command contract

The [authoring guide](ZenithAuthoringGuide.md) documents all 43 supported actions and 23 cached conditions. `autos/robot.json` provides editor controls, units/defaults/bounds and resource keys. Old command names/defaults remain compatible. Independent yaw/hood/RPM modes cannot be overwritten by later background aim updates; forward intake respects the held full latch while explicit reverse is allowed. Transfer pause/end cleanup preserves queue knowledge.

Automatic shooting still requires verified calibration, homing, field pose, hood/yaw/RPM readiness and release timing; no visible tag or new motion-speed cutoff is added. Explicit `feedManual` is a separately named readiness bypass with the gate delay and inhibit priority. Preset measurements remain unverified and cannot automatically feed. `stopAll` is resumable and retains home/positional posture; terminal cleanup closes cameras and invalidates terminal turret state.

## Architecture and verification

`PedroDrive` owns the existing mecanum and Pinpoint subsystems. It selects open-loop motor power for Pedro during INIT, preserving existing aliases, cloned bare-motor specs, directions and BRAKE. TeleOp keeps its existing mode. Its Pinpoint adapter converts cached meters to inches without changing field axes or rotating global velocity twice. Start-pose writes wait until Pinpoint is READY so IMU calibration cannot overwrite them. Each loop clears REV caches, reads Pinpoint once, updates the turret, updates Pedro once, runs Ivy, applies mechanism outputs, and flushes telemetry once.

Tests cover unit conversion, mecanum signs and saturation, actual Pedro hold/path-speed behavior, the pollen planner and replanning, the packaged pollen asset, and Zenith curves, markers (including endpoint and interrupted markers), parallel timeouts, branches, deferred starts, cancellation, and invalid files. These are JVM checks, not evidence of physical robot tuning.

The canonical pollen source is `autos/pollen-navigation-test.auto.json`. Team deployment copies it and all field metadata into APK assets while preserving its isolated POLLEN_TEST profile and sole Test wrapper. Its start pose is explicitly a cleared-area local reference, not an absolute field reference. The cleared-area field intentionally has no season plugin, so scoring/ledger checks are inert and the CLI reports that notice. The camera-guided collection estimate is unknown; its 28-second bound is not a predicted trajectory.

Field model provenance: autos/field/biobuzz.field.json was copied unchanged from Horizon-36596/zenith v0.1.2, commit f7c3681a158c6f504ab211464559d52da8a46955, examples/starter/autos/field/biobuzz.field.json. Its per-feature provenance distinguishes specifications from approximations. [Zenith's MIT license](ZenithFieldLicense.txt) accompanies the copy.
