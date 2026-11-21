# Code review of the team's V1 changes, 2026-11-21

Scope: the code the team added on `V1` since `master` (30 commits, about 120 main Java files, roughly 13.5k lines, plus build files, scripts, tools and the three draft autos). The Zenith `volley` and feed-policy work from PR #1 is not re-reviewed here.

## How it was reviewed, and its limits

This was a **risk-triaged review, not a line-by-line read** of every file.

| Method | Result |
| --- | --- |
| Android Lint (`lintDebug`) | 0 errors; 20 warnings (12 SDK native libraries not 16 KB aligned, 6 `DefaultLocale`, 2 unused resources). |
| `javac -Xlint:all` on TeamCode | No warnings in the team's code beyond raw `Map[]` allocations in `AutonomousCommands`. The rest is SDK sample code. |
| Pattern scans of all main Java | No TODO/FIXME, no `System.exit`/`printStackTrace`, no unsafe `getMotorType()` mutation, no hard-coded secrets; two intentional `catch ... ignored` around gamepad rumble. `sleep` only inside the tuning procedures. |
| `AGENTS.md` rules | BRAKE on drivetrain, intake and transfer; FLOAT on the flywheel; `telemetry.update()` once per loop (two per OpMode file is the INIT loop plus the active loop); NaN guard and clamp on `setShooterRpm`; `dt` divisions guarded in `PinpointOdometry`. |
| Hand-read in full | `testdrive`, `FallbackAutonomous`, `RedFallbackAutonomous`, `FallbackAutoController`, `TransferSubsystem`, `GateSubsystem`, `LinearSlideSubsystem`, `RobotShutdown`, `PlannerVolley`, and the team's original `BioBuzzRobot`, `AutonomousCommands` and `AutonomousOutputCoordinator`. |
| Not hand-read | `TurretSubsystem`, `ShotCoordinator`, `PoseHistory`, `PlannerRobot` (apply loop only), `ForesightTuner`, `Tests`, `PollenVision`/`PollenNavigator`, Limelight classes and the field-geometry classes. They are covered by the test suite and the scans above, but nobody read them for this review. |
| Test suite | 343 unit tests pass. 3 SimLoop tests fail on the team's own `V1` (item M1). |
| Repo hygiene | No secrets in tracked text; tracked tool artifacts listed in L1. |

Overall the code is in good shape: small hardware-free controllers with tests, guarded math, consistent shutdown handling. The findings below are mostly decisions, not bugs.

## Fixed in this branch (behavior-neutral)

| # | Change | Why |
| --- | --- | --- |
| F1 | `MecanumTuner`, `PinpointTuner`, `IntakeShooterTestOpMode`: `toLowerCase()` and `String.format` now pass `Locale.ROOT`/`Locale.US` (6 sites). | Lint `DefaultLocale`. A device set to Turkish lower-cases `I` wrongly and some locales print decimal commas. Display text only. |
| F2 | `FallbackAutoController`: comment said the 0.35 power cap is the "same as Pedro autonomous". | Pedro autonomous no longer has a power cap (`MAX_WHEEL_POWER` is 1). The fallback's own cap is unchanged. |
| F3 | `doc/CompetitionTeleOp.md`: three statements contradicted the code. | The shot table is team-measured and final (2026-11-19), so "uncalibrated profile ... automatic feeding remains inhibited" was wrong; staggered feeding was removed, so "final staggered timings are pending" was wrong; the GP2 trim note assumed the uncalibrated profile. Dated plan and review records were left as history. |

Verified after the fixes: 343 unit tests pass, lint has no `DefaultLocale` warnings.

## Needs human approval

### High

| # | Finding | Evidence | Proposed change |
| --- | --- | --- | --- |
| H1 | **Transfer stall protection was removed.** Both transfer motors (goBILDA 5203, 6000 RPM) always get the controller's full-power command, even while jammed. | `TransferSubsystem` class comment: "no stall limit or safety cut-off (removed 2026-11-19 by request)". Current is read at 5 Hz for telemetry only. | Restore a protective limit: pause stage 1 when motor current stays above a measured threshold for a set time, with a driver-visible fault. Needs a measured stall current and a team decision, so it was not changed. |
| H2 | **`BioBuzzAuto` (`testdrive.java`) bypasses the Pedro tuning gate.** Every other Pedro/Zenith auto refuses to START while `PedroConfiguration.TUNING_VERIFIED` is false; this one drives at full speed regardless. It is registered as an `@Autonomous` in group "Auto", has `STEP_TIMEOUT_S = 0` (a drive step ends only on settle or the 29 s deadline), has no tests, and the class name is lower-case. | `BioBuzzRobot.onStart` checks `TUNING_VERIFIED`; `testdrive.runOpMode` does not. | Either add the same gate, or move it to the `Test` group and name it as a test (for example `Test: Planner Drive`), and rename the class. The team may be using it deliberately for test drives, so it was not changed. |

### Medium

| # | Finding | Evidence | Proposed change |
| --- | --- | --- | --- |
| M1 | **Three SimLoop tests fail on `V1`.** They fail identically on the untouched commit. | `bothAlliancesRegulateTheFlywheelWithBangBangFusion` and `benchFusionControllerSpinsUpHoldsAndFeedsThroughActualOpMode` expect 1400 RPM and get about 1281; `benchLoadsHoldsFullFeedsAndReloadsThroughActualMotorOutputs` fails an assertion. Recent commits are PID-tuning pushes. | Decide whether the tests' expectations or the tuned flywheel values are stale, then fix that one. Not done because it needs the team's intent. |
| M2 | **The build depends on a Pedro core snapshot.** `com.pedropathing:core:3.0.0-20260828.185437-17` comes from the Sonatype snapshots repo, which can purge old snapshots and break clean builds. Pedro core 3.0.2 and `tuning` 1.0.3 are now released on Maven Central. | `TeamCode/build.gradle`. Zenith v0.1.2 documents the snapshot as its tested pairing. | Plan a move to a released version with a full re-test (and re-measured tuning). Until then, keep a local copy of the artifact. Not changed: it risks behavior and the tuned values were measured on the snapshot. |
| M3 | **Draft autos have budget and timeout problems** (Zenith validator). | `red-far-cycle`: worst case 32.6 s against the 29 s hard deadline; `farFlowerPath` timeout 2.0 s is under its 2.25 s estimate; several paths are driven 60-100% sideways. `red-audience` worst case 26.5 s. | Strategy decisions with placeholder coordinates, so left to the team. |
| M4 | **Every Zenith auto is blocked on the robot** while `TUNING_VERIFIED` is false. Only the fallback autos can run. | `BioBuzzRobot.onStart`. | Complete tuning verification (Test: Pedro Drive and the web tuner), then flip the flag deliberately. Never as part of a refactor. |

### Low and informational

| # | Finding | Proposed change |
| --- | --- | --- |
| L1 | **Tool artifacts are tracked in git:** `.codex-attachments/` (1 image), `.codex-remote-attachments/` (2 images) and `.freebuff/project-id`. Contents checked: two Driver Hub screenshots and a field diagram, nothing sensitive (one webcam serial number). | `git rm --cached` them and add the folders to `.gitignore`. Deleting tracked files was left to you. |
| L2 | **Untested hardware wiring:** `FallbackAutonomous` (the only runnable auto), `testdrive`, `PollenVision`, `RobotLoop`, the tuners and the calibration OpMode have no tests. The controllers behind them are tested. | Add a fake-hardware test for `FallbackAutonomous` in the style of `AutonomousIntegrationTest`. |
| L3 | **Windows-only tooling.** `tools/*.ps1` and the `robotInstall`/`robotSloth` Gradle tasks call `powershell.exe`; `AGENTS.md` shows `./gradlew.bat`. Mac developers cannot use the one-command deploy. | Add a cross-platform deploy script, or document the manual adb steps (done in `ZenithAutoGuide.md`). |
| L4 | **The Pedro web tuner listens on the robot network** (`192.168.43.1:10158`, no authentication) and can start motion-capable tuning procedures. | Awareness only. It is on the robot's own Wi-Fi; confirm it is acceptable under event network rules and disable it for competition if not. |
| L5 | 12 Lint `Aligned16KB` warnings for SDK native libraries (`libRobotCore`, `libEasyOpenCV`, `libapriltag`, `libc++_shared`). | Nothing to do until the SDK vendor ships aligned libraries. |
| L6 | `libs/ftc.debug.keystore` is tracked. | Expected: it is the standard FTC template debug keystore. Do not reuse it for release signing. |

## What needs a human to complete

1. **Decide H1 and H2** (transfer stall limit; the ungated test-drive autonomous). These are safety choices.
2. **Triage M1** (the three failing SimLoop tests) so `V1` has a green suite.
3. **Approve and merge PR #1**, which changes how Zenith autonomous feeds on the real robot. This review's PR is based on PR #1's branch and should merge after it.
4. **Verify tuning and run on the robot** (M4). Nothing in these PRs has run on hardware.
5. **Approve the cleanup in L1** and decide M2 and M3.
