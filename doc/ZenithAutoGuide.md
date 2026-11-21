# Creating and testing autonomous routines in Zenith

Step-by-step guide for BioBuzz 10100 (`V1` branch). New to autonomous? Start with the [beginner's guide](ZenithAutosBeginnerGuide.md); this one is the detailed reference for the `volley` command and the test ladder. It covers building a routine in the Zenith editor, testing it in five stages from "no robot" to "on the field", and putting it on the Control Hub. For the full command reference see the [authoring guide](ZenithAuthoringGuide.md); for the match plans see [MatchAutonomousPlan.md](MatchAutonomousPlan.md).

Updated 2026-11-21. **What is verified and what is not** is listed at the end. Read that section before trusting any step marked *untested*.

---

## 0. Before you start

| Need | Detail |
| --- | --- |
| Tools | Node 22+ and `npm ci --ignore-scripts` once in the repo root; JDK 17; Android SDK (see the setup checklist). On a Mac use `./gradlew`, not the `.\gradlew.bat` written in older docs. |
| Editor | **Windows:** the desktop app from the [Zenith releases](https://github.com/Horizon-36596/zenith/releases) page (`Zenith-Setup-0.1.2.exe` or `Zenith-Portable-0.1.2.exe`). **Mac/Linux:** the web editor at <https://zenith.horizon36596.org> in **Chrome or Edge** (needed to open a local folder). The web editor has no deploy, high-fidelity sim or commit buttons; those steps below use the terminal instead. |
| Version | Runtime `v0.1.2`, CLI `0.1.2`, Ivy `1.1.1`, Pedro `3.0.0-20260828.185437-17`. These are Zenith's own documented, tested set and the latest releases as of 2026-11-21. Do not bump one alone. |

> **Gate: Pedro tuning.** While `PedroConfiguration.TUNING_VERIFIED` is `false`, every Zenith routine refuses to START on the robot with *"Measure and verify PedroConfiguration before running autonomous"*. It has been `true` since 2026-11-21, set on the team's instruction; the checks in [TuningVerificationChecklist.md](TuningVerificationChecklist.md) were not recorded, so verify with **Test: Pedro Drive** and the web tuner (see [Autonomous.md](Autonomous.md#before-robot-motion)) and run new routines slowly. If the flag is set back to `false`, the **Auto: Red/Blue Fallback (Shoot + Leave)** routines are the only autonomous that can run.

---

## 1. Open the project

1. Open the editor. In the web app, use the open-folder action and choose the repository root (the folder that contains `zenith.json`). The title bar shows the project and the open auto.
2. The editor loads `autos/robot.json` (the command catalog), `autos/field/biobuzz.field.json` and `autos/waypoints.json`. **Never edit these in the editor's code panel or in generated Java or APK assets.** `autos/` is the only source.
3. After anyone changes `autos/robot.json`, reopen the project so the editor reloads the catalog (it currently lists 44 commands and 23 conditions).

Layout: the field in the middle, the step list and timeline on the left, the inspector (every setting of the selected step) on the right, and the **Problems** panel under the field. Tools: **V** select, **P** add path, **M** marker, **H** heading, **U** measure, **S** snap, **C** add command.

---

## 2. Create a routine

1. **New auto.** It starts as a generic `new-auto.auto.json` and is written into `autos/` when you save.
2. **Name it.** The file name must be lowercase kebab-case and the JSON `name` must equal the file stem: `red-audience.auto.json` has `"name": "red-audience"`. Set `title` to what the Driver Station should show. If you rename, change the file, the `name` and the `title` together (the editor has no rename button).
3. **Alliance and start pose.** Set `alliance` (RED or BLUE) and drag the start pose to where the robot really sets up, with its true heading. Blue routines are usually the red file run point-mirrored by a hand-written wrapper, so author the red one and keep coordinates in field inches (center origin, +X audience right, +Y away from the audience, heading CCW radians).
4. **Save** (Save button or Ctrl+S). Then validate (stage 2).

Add a measured `provenance` note to any pose you measured; unmeasured poses are reported as "SET FROM EDITOR" info, which is fine for drafts.

---

## 3. Build the route

A routine is a list of **steps** that run top to bottom. There are seven kinds: `path`, `command`, `wait`, `sequence`, `parallel`, `branch` and (planner-only) `transfer`. Use **Insert** to add a step after the selected one.

| Want to... | Do this |
| --- | --- |
| Drive somewhere | Add path (**P**), click the field. Choose a heading mode (`tangent`, `tangentReversed`, `constant`, `linear`, `facePoint`). Set `speedFraction` (omit for the robot default, which is currently 1.0). Always set `timeoutS`. |
| Do something part-way along a path without stopping | Marker (**M**), click the path; pick a command such as `intakeOn`. A marker cannot own the drive. |
| Pre-aim while driving | A `command` step `aim` (side, frame) *before* the path. It latches yaw/hood tracking and pre-spins the flywheel; it finishes instantly. |
| Collect while driving | A `parallel` step (mode `all`) containing `loadUntilFull` (with `seconds`) and the path. |
| **Shoot a volley** | A `command` step `volley` (section 4). |
| Wait for a robot condition | A `wait` step with `until` set to a named condition **and `timeoutS`** (the validator rejects a condition wait without one). |
| React to a failed action | A `branch` on `commandFailed`, with a safe `stopAll` park in the failure arm. |
| Deploy/stow the depollinator | `depollinatorDeploy`, `depollinatorHome` (about 1 s settle, an estimate, not feedback). |
| End safely | A final `stopAll`. It is **resumable**: later steps can turn outputs back on, so put remaining scoring inside the success branch. |

### Rules that prevent most problems

1. **Every `path`, `command` and `wait` that can block gets a `timeoutS`.** Group steps (`sequence`, `parallel`, `branch`) cannot have one; bound the steps inside them. A timeout that is shorter than the action's own bound reports `CANCELLED`, not `TIMEOUT`, so give actions a little headroom.
2. **One writer per channel.** The channels are `drive`, `intake`, `transfer`, `gate`, `depollinator`, `turretYaw`, `hood`, `flywheel`, `pollenCamera`, `limelight`. Do not put two commands that use the same channel in one parallel group, and do not put a drive-owning command on a path marker.
3. **Total time.** The global autonomous deadline is 29 s. The editor's Problems panel gives a `TIME_BUDGET` line; remember waits and volleys can run to their full timeouts.
4. **Read `commandFailed` only right after the action it describes.** It reports the *last* bounded outcome (`SUCCESS`, `TIMEOUT`, `INHIBITED`, `FAULT`, `CANCELLED`), which later instant steps do not overwrite.
5. **Start poses must be legal.** The field file carries start rules, and the editor flags a start pose that breaks them (for example one that touches the wall). Drag it to where the robot really sets up.

---

## 4. The volley (the TeleOp "full auto-shoot")

`volley` is the autonomous version of **GP1 RT + LT** with the **GP1 A** fallback, and it behaves like the path planner's *Volley* step. It does the whole procedure in one step:

1. Latch the target hive cell (`side`/`frame`) and turn on automatic yaw, hood and RPM from the measured shot table. If `aim` already latched the same side, it is kept, so a pre-aim during the path is not thrown away.
2. Wait up to `readyTimeoutSeconds` for the shared firing coordinator: turret, hood and RPM ready and stable for 0.15 s.
3. Feed **continuously** for `seconds` (gate and both transfer motors start together; this is the final TeleOp feed policy).
4. Not ready in time: `FIRE` feeds anyway with readiness bypassed (like TeleOp GP1 A) **only if the turret is homed**; `SKIP` feeds nothing.
5. Apply `after`, then end.

| Parameter | Default | Range | Meaning |
| --- | --- | --- | --- |
| `side` | `AUDIENCE` | `AUDIENCE`, `FAR` | Hive cell to shoot at. |
| `frame` | `FIELD` | `FIELD`, `AUTHORED` | `AUTHORED` swaps the side for the mirrored alliance, so one file serves both. |
| `seconds` | 2.5 | 0.001-28 | Feed time once feeding starts. Time, never a ball count. |
| `readyTimeoutSeconds` | 3 | 0-28 | Longest wait for readiness. |
| `onTimeout` | `FIRE` | `FIRE`, `SKIP` | What to do if not ready in time. |
| `after` | `KEEP` | `KEEP`, `IDLE`, `OFF` | `KEEP` aim + spin; `IDLE` TeleOp idle RPM (3000) with yaw held; `OFF` stop the flywheel, hold yaw. |
| `yawTrimDeg` / `hoodTrimDeg` / `rpmTrim` | 0 | ±10° / ±5° / ±500 | TeleOp's GP2 auto-aim trims, applied from this volley until the next volley or `aimOff`. |
| `latestStartS` | 0 (automatic) | 0-29 | Match-clock second after which the gate will not open. 0 means the 29 s global deadline minus `seconds`, so a late volley is skipped instead of being cut off mid-feed. |

The defaults match the planner (`FEED_SECONDS` 2.5, `READY_TIMEOUT_S` 3).

**Outcomes** (read with `commandFailed`, shown on the Driver Station as *Last bounded action*):

| Outcome | Meaning |
| --- | --- |
| `SUCCESS` | Became ready on its own and fed for `seconds`. |
| `TIMEOUT` (reason "Fed before READY...") | Not ready in time; `FIRE` fed with readiness bypassed. Balls were launched, but readiness was missed. |
| `TIMEOUT` (reason "SKIPPED...") | Nothing was fed: `SKIP`, or `FIRE` with an un-homed turret. |
| `TIMEOUT` (reason "SKIPPED: too late...") | The time guard: the match clock passed `latestStartS` before the gate opened, so nothing was fed. A feed already under way is never cancelled by the guard. |
| `INHIBITED` | Feed inhibit was set; inhibit always wins, even over `FIRE`. |
| `CANCELLED` | Interrupted (outer timeout, race group, STOP). |

**Choosing `FIRE` vs `SKIP`.** `FIRE` is the planner and TeleOp default and means the shot always happens, a little late, even when readiness never arrives. Use `SKIP` where firing unready would be worse than missing the shot, and branch on `commandFailed` to decide what comes next.

**Step timeout.** Set the volley step's `timeoutS` to at least `readyTimeoutSeconds + seconds` plus a margin. The editor's estimate for a volley is `seconds + readyTimeoutSeconds`, so its `TIMEOUT_TIGHT` warning protects you.

Example (this exact routine is the regression fixture `TeamCode/src/test/resources/zenith/volley-check.auto.json`, validated by the Zenith CLI and run through the real builder in `AutonomousIntegrationTest`):

```json
{ "id": "preAim", "kind": "command", "name": "aim",
  "args": { "side": "AUDIENCE", "frame": "AUTHORED", "spin": true } },
{ "id": "driveOut", "kind": "path", "timeoutS": 6, "speedFraction": 0.35,
  "segments": [ { "kind": "bezier", "from": "current",
    "control": [ { "xIn": -6, "yIn": -51 } ], "to": { "xIn": -12, "yIn": -39 } } ],
  "heading": { "mode": "linear", "fromRad": 1.5708, "toRad": 1.5708 } },
{ "id": "volley1", "kind": "command", "name": "volley", "timeoutS": 2,
  "args": { "side": "AUDIENCE", "frame": "AUTHORED", "seconds": 1,
            "readyTimeoutSeconds": 0.5, "onTimeout": "FIRE", "after": "IDLE" } },
{ "id": "volleyResult", "kind": "branch", "condition": "commandFailed",
  "then": [ { "id": "volleyFailedPark", "kind": "command", "name": "stopAll" } ],
  "else": [ { "id": "volleyOkPark", "kind": "command", "name": "stopAll" } ] }
```

The three draft match autos (`red-audience`, `red-far`, `red-far-cycle`) still use the older hand-built pattern `aim` → `wait until shotReady` → `shoot`. It works, but `shoot` has no ready timeout and no fire-anyway fallback. Converting them to `volley` is a separate, deliberate change: it alters on-robot behavior, so do it with the team and re-run stages 2-5.

---

## 5. Test ladder

Run the stages in order. Each one catches a different class of mistake, and a pass at one stage says nothing about the next.

### Stage 1: In the editor (no robot, no build)

- Open **Problems**. Fix every error; read warnings (`TIMEOUT_TIGHT`, `STRAFE_FRACTION`, `TIME_BUDGET`) and decide on purpose. Click a problem to jump to its step.
- Play the routine (Space). The default **Ideal** playback follows the plan exactly; **Instant sim** shows overshoot and drift from a Pedro follower model. Both use the numbers in `robot.json`, several of which are still unmeasured (robot height, the depollinator-down length, acceleration, angular speed), so the times are estimates.
- The high-fidelity sim (desktop only) is **not configured** for this repo. There is no "Full sim" trace to load.

### Stage 2: Validate against the real robot code

```bash
npm run zenith:validate
```

This is stricter than the editor: it checks every command name, argument, enum and range, hardware profile, file stem, step ID, and the native schema, and it checks that `autos/robot.json` and `AutonomousCommands.java` agree. Expect `0 Zenith error(s)`. The long list of `REGISTRY` info lines is the pinned scanner's known noise (it cannot read the indirect factories); real problems print as errors.

### Stage 3: Run the real code in a unit test

Unit tests run the actual command factories, output coordinator, shot coordinator and Zenith's native builder against fake motors, so they catch sequencing and cleanup mistakes. They do **not** model chassis physics, camera or loaded mechanisms.

```bash
./gradlew :TeamCode:testDebugUnitTest
```

To regression-test a new routine, copy it into `TeamCode/src/test/resources/zenith/`, then copy the test `actualVolleyRoutineRunsThroughNativeBuilderBranchesOnItsOutcomeAndStops` in `AutonomousIntegrationTest.java` and change the file name and the assertions. Keep these fixtures outside `autos/` so they are never deployed.

### Stage 4: Deploy and build the APK

```bash
npm run zenith:deploy -- --dry-run     # lists what would change; check it
npm run zenith:deploy                  # writes the APK assets and one OpMode stub per routine
npm run zenith:check                   # must report 0 writes, 0 removals
./gradlew :TeamCode:testDebugUnitTest :TeamCode:assembleDebug
```

- Use team deployment, **not** the editor's native Deploy button; the native one can generate a duplicate pollen wrapper that `zenith:check` rejects.
- Saving a routine does nothing to the Control Hub on its own. The chain is **save → deploy → build → install**.
- If you forget to deploy after changing `autos/robot.json` or a routine, `zenith:check` fails with *"Zenith outputs are stale/duplicated"*.
- `scripts/check-zenith-apk.ps1` is a Windows PowerShell script that verifies the packaged JSON assets against `deployment-manifest.json`. It was not run on macOS here. A quick manual check is `unzip -p TeamCode/build/outputs/apk/debug/TeamCode-debug.apk assets/autos/robot.json | grep '"name": "volley"'`.

### Stage 5: On the robot

**Install (untested on a real Control Hub from this Mac):** join the Control Hub Wi-Fi, run `adb connect 192.168.43.1:5555`, then `./gradlew :TeamCode:installDebug` and open the Robot Controller app (or run `./gradlew :TeamCode:robotLaunchApp`). The `robotInstall` task and `tools/*.ps1` scripts call `powershell.exe` and are Windows-only. Android Studio's Run button also works with the Hub connected over adb.

**Before any motion**, in this order:

1. `TUNING_VERIFIED` must be `true` (section 0). Until then START throws.
2. Run the Test OpModes for each mechanism the routine uses: **Test: Pedro Drive**, **Test: Pinpoint Odometry Only**, **Test: Turret**, **Test: Auto Aim**, **Test: Intake + Index + Shooter**, **Test: Gate Only**, **Test: Depollinator Only**.
3. Clear the area, with the robot's maximum extension (depollinator deployed) in mind. Its deployed length (537 mm measured) is not yet in `robot.json`; see the tuning checklist.

**First run (low risk):** put the robot on blocks, wheels off the floor. Select the routine from the **Generated** group on the Driver Station (the entry is named by the routine's `title`). The hand-written blue mirrors of the draft autos (`DRAFT Auto: Blue ...`) are in **Competition** instead. Run INIT, then START. Watch the mechanism steps (intake, aim, volley feed, depollinator). Paths will not move the robot and should end by timeout.

**Second run:** on the floor in a cleared area with every `speedFraction` temporarily low (0.3), with someone holding STOP. Then raise it.

**INIT checklist** (watch the Driver Station telemetry):

- Keep the robot still while the Pinpoint calibrates. The start pose is seeded from the authored pose.
- *Start relocalization* shows progress as the turret homes and looks at hive tags. **CHECK PLACEMENT** means the tags disagree with the authored start pose by more than the correction limit: re-place the robot or fix the start pose.
- Pedro line: tuning must read VERIFIED.

**During a run, read:** *Autonomous step / remaining (s)*, *Last bounded action* (step, outcome, reason, time), *Output modes yaw / hood / flywheel*, *Feed inhibited*, and the shot lines.

**Stopping:** Driver Station STOP stops all outputs and closes the gate. The routine also stops on completion, on localization loss, at the 29 s deadline and on an exception. Test STOP and a fault in the cleared area before relying on a routine in a match.

---

### Automatic checks

- **CI** (`.github/workflows/ci.yml`) runs on every pull request: the three Zenith commands, then the unit tests and a debug APK build. It does not run the SimLoop suite, because three of its tests already fail on `V1`.
- **Shot-range check** (`AutoVolleyRangeTest`): every `volley` and `shoot` under `autos/` must be fired from inside the measured shot table (12-60 in from the hive opening). The profile extrapolates outside it, which gives a hood and RPM from an unmeasured line. Routines titled DRAFT or EXAMPLE only print a report, so placeholder coordinates do not fail the build; remove the word from the title once the route is measured and the check starts enforcing.

## 6. Troubleshooting

| You see | Cause and fix |
| --- | --- |
| *Measure and verify PedroConfiguration before running autonomous* on START | `TUNING_VERIFIED` is false: the robot runs a build from before 2026-11-21, or the flag was set back (section 0). |
| *Pinpoint has no READY, fresh autonomous start pose* on START | The Pinpoint was still calibrating or moved. Keep the robot still in INIT and wait. |
| Outcome `FAULT`: *Pinpoint localization unavailable or stale* | Odometry dropped out mid-run; the routine stops. Check the Pinpoint cable and pod directions. |
| Outcome `TIMEOUT`: *Global autonomous deadline* | The routine hit 29 s. Shorten waits and volley timeouts. |
| `zenith:validate`: *condition waits require timeoutS* | Add `timeoutS` to that wait step. |
| `zenith:validate`: *Java/robot.json registry names disagree* | A command exists in only one of `AutonomousCommands.java` and `autos/robot.json`. |
| `zenith:check`: *Zenith outputs are stale/duplicated* | Run `npm run zenith:deploy`. |
| Editor `TIMEOUT_TIGHT` on a path or action | The step's `timeoutS` is shorter than its estimate. Raise it, or accept that a slow run will be cut short. |
| `volley` ends `TIMEOUT` with "Fed before READY" | Turret, hood or RPM never became stable inside `readyTimeoutSeconds`. Check the shot table range (12-60 in), that the turret is homed, and the flywheel RPM telemetry. |
| `volley` ends `TIMEOUT` with "SKIPPED" | `onTimeout` is `SKIP`, or the turret was not homed. |
| Nothing feeds and the outcome is `INHIBITED` | `feedInhibit` ran earlier and `feedResume` has not. |

---

## 7. Adding a new command (needs a Java change)

Most routines need no Java. A new robot action needs all of these, together:

1. A factory in `AutonomousCommands.register()`. **Keep the registration on one line** (`command("name", "resources", "params", (a,c) -> {`): the Node parity check reads that exact shape, and a line break makes it report "registry names disagree".
2. An entry in `autos/robot.json` with typed params, defaults and ranges, the required channels and an `estimateS`.
3. Tests in `AutonomousIntegrationTest` (arguments rejected at INIT, behavior, cleanup). The registry-parity test runs every catalog entry through the real factory.
4. `npm run zenith:validate`, then `zenith:deploy`, `zenith:check` and a full build; reopen the editor.

---

## 8. What is verified, and what is not

**Verified (2026-11-21):** the catalog validates with 0 errors; 343 unit tests pass, including the volley behavior (hold-fire then fire, skip, inhibit, cancel, after-states, argument bounds) and a `volley` routine run through Zenith's native builder; deployment is fresh; the debug APK builds and contains the current catalog.

**Not verified:**

- **Nothing here has run on the robot.** Values marked UNMEASURED in `robot.json` (robot height, depollinator-down length, acceleration, angular speed) are still placeholders.
- The Mac workflow (web editor on a Mac, `./gradlew :TeamCode:installDebug` to a Control Hub) is untested. The team's earlier editor acceptance was done with the Windows desktop app ([ZenithUiAcceptance.md](ZenithUiAcceptance.md)).
- The volley's feed uses the final TeleOp continuous policy, but its timing, the `FIRE` fallback and the aim lead are unmeasured on the real robot.
