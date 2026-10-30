# Local TeleOp simulation

This implements the first milestone of [the TeleOp plan](../doc/SimLoopTeleOpImplementationPlan.md).
Simulation is optional for major behavioral changes. Routine tuning and minor edits use proportionate
unit tests and robot measurements. CI and full-robot simulation remain deferred.

From the repository root, with the usual FTC Android SDK/JDK setup:

```powershell
# Ordinary tests, APK and lint; no simulation runs.
.\tools\verify-teleop.ps1
# Select a mechanism (also checks the JVM JSON runtime).
.\tools\verify-teleop.ps1 -Sim -Scenario Gate
.\tools\verify-teleop.ps1 -Sim -Scenario Drivetrain
# All currently implemented simulations, plus ordinary checks.
.\tools\verify-teleop.ps1 -FullSim
# Direct opt-in task, optionally narrowed by class or method.
.\gradlew.bat :TeamCode:simTeleOpTest
.\gradlew.bat :TeamCode:simTeleOpTest --tests '*GateSimTest'
```

`-FullSim` means the entire **available** suite (drivetrain, gate, JVM runtime), not a full-robot model.
The verification script rejects missing/skipped tests, checks the named baseline tests still execute,
and propagates Gradle failures. The manifest in `tools/teleop-test-baseline.json` records the 154 baseline
JUnit 4 cases and the initial simulation cases; update it deliberately if tests are renamed or replaced.
New tests can be added without changing the baseline. Direct Gradle invocations still fail on no matching tests.

Tests run the actual `DrivetrainTestOpMode` and `GateTestOpMode` via `OpModeRunner.forTeamClass`,
including INIT/START/ACTIVE/STOP, without copying production coordination into a substitute OpMode.
Only primitive snapshots, SDK objects and SimLoop fakes cross class-loader boundaries.
The 20-ms default and selected 10-ms runs use the simulation clock for team timers.

Reports are under `TeamCode/build/reports/tests/simTeleOpTest/`. Unique scenario directories under
`TeamCode/build/sim/teleop/` contain parameters, inputs, decoded output/model/telemetry traces and runner
outcomes, retained even on failure. JUnit reports hold expected/actual assertion failures and print trace
paths. Compare decoded TSV values, not random directory names or logger internals. No RLOG logger or
desktop window is needed for this milestone. OpMode runs are serial within the test JVM; release and
ordinary debug tests exclude only `org.firstinspires.ftc.teamcode.sim`, so they cannot share these logs.

The model's mounting signs are deliberately independent of configured motor directions. Its geometry,
effective ticks/in, strafe response and motor lag are explicitly synthetic, unmeasured values. The suite
asserts signs, bounds, lifecycle and command sequencing; it does not certify real distances, loaded gate
clearance, servo timing, electrical behavior or Control Hub performance. See the
[verification matrix](../doc/TeleOpVerificationMatrix.md) for remaining hardware checks and deferred coverage.

## Dependency and Android boundaries

`simloop.gradle` adapts the starter at **v0.2.0-alpha.2**, inspected at that tag, under the BSD-3-Clause
license in [LICENSE](LICENSE). Upstream: [SimLoop](https://github.com/Horizon-36596/simloop/tree/v0.2.0-alpha.2),
copyright 2026 Horizon (FTC 36596). No upstream example robot sources or agent policies are imported.
This adaptation omits the unused keyboard desktop artifact, keeps both ordinary unit-test variants, adds
the opt-in debug task and separate trace directories, and preserves Vintage/launcher/real JSON setup.
SimLoop and its PsiKit logging dependencies are test-only and excluded from production APK dependencies.
FIRST SDK and PsiKit retain their respective licenses; Horizon does not endorse this adaptation.

JUnit 4 uses Vintage 5.11.3, new tests use Jupiter 5.11.3, and Gradle 9 uses the explicit Platform 1.11.3
launcher. Real `org.json:json:20250517` supplies JVM parsing; `SimRuntimeTest` verifies real values rather
than Android stub defaults. The pinned SimLoop artifact resolves against FTC SDK 12.0.0 in this project.

`unitTests.returnDefaultValues=true` follows the starter's SDK matrix requirement. Existing baseline
tests have no direct Android imports and exercise pure controllers/geometry/file helpers; they all still
execute. Android calls can now return null/zero/false silently. That setting is not evidence of working
Android context, services, USB, cameras, storage paths or hub caching. The harness explicitly checks an
empty Lynx inventory; actual cache timing remains a robot measurement. Real JSON parsing is tested, but
Limelight geometry, simulated observations and full competition localization are not yet SimLoop-covered.
