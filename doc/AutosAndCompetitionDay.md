# Autos in the repo, getting them onto the robot, and tuning on competition day

Written 2026-11-21 against `V1.0.1` plus the open pull requests (#1-#5). It will go stale as those merge and as autos change: re-check the list against `@Autonomous` annotations in `TeamCode/src/main/java` before relying on it. Nothing here has been run on the robot.

## 1. The autos in the repo

**Update 2026-11-21:** `PedroConfiguration.TUNING_VERIFIED` has been `true` since 2026-11-21 (set on the team's instruction; the checks in [TuningVerificationChecklist.md](TuningVerificationChecklist.md) were not recorded), so the gate no longer blocks any routine. "Gated" in the table below means the routine was blocked by that gate when this was written, and would be again if the flag were set back to `false`.

| Driver Station name | Group | What it is | Can it run today? |
| --- | --- | --- | --- |
| **Auto: Red Fallback (Shoot + Leave)** | Competition | No Pedro. INIT: GP1 D-pad sets a 0-15 s start delay, X toggles the preload shot (default off). Leaves 24 in forward. Preselects the Red TeleOp. | **Yes**, not gated. The leave distance and turret heading are unmeasured. |
| **Auto: Blue Fallback (Shoot + Leave)** | Competition | The blue version of the above. | **Yes**, not gated |
| **BioBuzzAuto** (`testdrive.java`) | Auto | Planner-generated: volley, three drives, volley. | Runs, but it **bypasses the gate** and drives at full speed. See finding H2 in `doc/CodeReview-2026-11-21.md` (PR #2). |
| **DRAFT Auto: Red Audience (4+4+4+4)** | Generated | Zenith: three volleys with garden and flower collection. | Gated; placeholder coordinates |
| **DRAFT Auto: Red Far (5 s delay, 4+4)** | Generated | Zenith: preload, flower, then the audience cell. | Gated; placeholders |
| **DRAFT Auto: Red Far Cycle** | Generated | Zenith: four volleys, far cell then audience cell. | Gated; placeholders; worst case can exceed the 29 s limit |
| **DRAFT Auto: Blue Far / Blue Audience / Blue Audience Cycle** | Competition | Point-mirrored wrappers of the three red routes; they preselect the Blue TeleOp. | Gated; placeholders |
| **EXAMPLE: Zenith integration** | Generated | An authoring example, not a match route. | Gated |
| **Test: Pollen Navigation (Pedro + Ivy)** | Test | Camera-guided collection experiment. | Gated |
| **TEST: Four corners, Lap x2, Line, Turns** | Generated | The drivetrain tests from **PR #3**; not on `V1.0.1` until it merges. See `doc/ZenithTestRoutines.md` on that branch. | Gated |

Two more, `Auto: Red Match (Route Required)` and `Auto: Blue Match (Route Required)`, are marked `@Disabled` and do not appear on the Driver Station.

## 2. Pushing autos to the robot

Autos are APK assets, so they need a **full install**. Sloth hot reload is Java-only and does not carry them.

1. Edit and save the routine in Zenith, then run:
   ```
   npm run zenith:validate
   npm run zenith:deploy
   npm run zenith:check
   ```
2. **Windows:** `.\tools\deploy.ps1 -Full`, or the Android Studio run configuration "Robot - Full Install". It builds, joins the robot Wi-Fi (`10100-RC`), installs, launches the app and rejoins your home network.
3. **Mac:** the `robotInstall` task calls `powershell.exe`, so use the manual steps. They are untested on a real Control Hub.
   - Join the Control Hub Wi-Fi.
   - `adb connect 192.168.43.1:5555`
   - `./gradlew :TeamCode:installDebug` (a `removeSlothRemote` step runs first and needs the adb connection)
   - Start the Robot Controller app, for example `./gradlew :TeamCode:robotLaunchApp`.
4. On the Driver Station, pick the routine from its group and run INIT. The telemetry shows the routine name and hardware profile. Commit the `.auto.json` file together with everything `zenith:deploy` changed.

## 3. Tuning on competition day

Some of this is general FTC practice rather than something the repo defines.

- **Baseline first.** Tag the known-good commit and keep its APK. Change one thing at a time, re-test it, and revert if it is worse. Keep a short log.
- **Before the first match:** use a full battery and note its voltage. Run the sign checks and a short drive test (see `doc/TuningVerificationChecklist.md`, PR #5). Spot-check the shot table at two or three distances. Confirm INIT shows Pinpoint READY and the turret HOMED.
- **Between matches:** do not retune Pedro. The Foresight Tuner needs lots of clear floor at full power, so it does not belong in the pit. Re-check only what telemetry says is off.
- **Low-risk live knobs:** shot trims (GP2 in TeleOp, or `volley`'s trim parameters) and flywheel gains in Dashboard (`192.168.43.1:8080/dash`). **Dashboard values do not persist**, so write the final numbers into code afterwards.
- **Use Sloth hot reload for Java tweaks**, and a full install only for autos, in the pit, with time to re-verify.
- **Field versus practice field:** re-check the start pose on the real field. Start touching the wall for repeatability, and watch for **CHECK PLACEMENT** in INIT telemetry.
- **Wireless limits:** check the event's wireless rules before tuning near the field.
- **Keep a fallback ready**, and record each run's outcome, battery voltage and the "Last bounded action" telemetry line.

The flag is `true`, but the verification checks have not been recorded, so treat the Zenith autos as unverified: run them slowly, in a cleared area, with someone holding STOP. The checklist in `doc/TuningVerificationChecklist.md` is how to record the checks.

## 4. How to test the volley

`volley` is the autonomous version of TeleOp's RT + LT auto-shoot (with the GP1 A fallback). Test it in layers, cheapest first. Stages 2-4 need a Zenith routine, which START only while `TUNING_VERIFIED` is `true` (it has been since 2026-11-21); stage 1 does not need the robot.

### Stage 1: in software (no robot)

- `./gradlew :TeamCode:testDebugUnitTest --tests '*AutonomousIntegrationTest'` runs the volley tests against fake hardware: hold fire until ready, fire anyway with a homed turret, never fire un-homed or on `SKIP`, inhibit wins, cancel stops outputs, the `after` states, argument and trim bounds, the `latestStartS` time guard, and a full routine run through Zenith's native builder (`TeamCode/src/test/resources/zenith/volley-check.auto.json`).
- `npm run zenith:validate` checks the arguments of every `volley` in `autos/`.
- `AutoVolleyRangeTest` (PR #4) checks every `volley` and `shoot` in `autos/` is fired from inside the measured 12-60 in shot table.

Fake hardware does not model real flywheel dynamics, gate travel or ball flow, so passing here proves the sequence, not the shot.

### Stage 2: bench, robot on blocks, no balls

Write a short routine (the pattern in `volley-check.auto.json`: `aim`, a path, then `volley`, then a branch on `commandFailed`) with `onTimeout: SKIP` so nothing is fed if readiness never arrives. The authored start pose decides where the turret thinks the hive is, so the robot can sit on blocks anywhere. Watch the Driver Station telemetry:

| Telemetry line | What you should see |
| --- | --- |
| INIT: turret home line | Home ready (the wording is HOMED or READY depending on the OpMode). A turret that is not homed cannot fire, even with `FIRE`. |
| *Output modes yaw / hood / flywheel* | AUTO / AUTO / AUTO once the volley starts |
| *Targets yaw / hood / flywheel* | Hood and RPM from the shot table for the range (below); yaw pointing at the chosen cell |
| Transfer mode | FEEDING SHOT once ready: the gate opens and both transfer motors run **together**, with no gate delay (continuous feed) |
| *Last bounded action* | The outcome and reason when the volley ends (below) |

Expected hood (degrees of downward travel) and RPM from the team-measured table (2026-11-19, final), interpolated between rows:

| Range (in) | RPM | Hood (deg) |
| --- | --- | --- |
| 12 | 2800 | 13 |
| 24 | 2900 | 8 |
| 36 | 2950 | 6 |
| 48 | 3200 | 6 |
| 60 | 3200 | 4 |

The range is measured from the turret pivot (about 51 mm ahead of the chassis centre) to the hive opening, as `AutoVolleyRangeTest` computes it. Targets that do not match the table for the authored range mean the pose or the target side is not what you think.

Outcomes to expect in *Last bounded action*:

| Outcome and reason | Meaning |
| --- | --- |
| `SUCCESS`, "Volley fed N s" | Became ready by itself, then fed for `seconds` |
| `TIMEOUT`, "Fed before READY (ready timeout); readiness bypassed" | `FIRE` fed after `readyTimeoutSeconds` without readiness. Balls would have been launched. |
| `TIMEOUT`, "SKIPPED: not ready in time" or "turret not homed" | `SKIP`, or `FIRE` with an un-homed turret. Nothing fed. |
| `TIMEOUT`, "SKIPPED: too late..." | The `latestStartS` time guard fired. Nothing fed. |
| `INHIBITED` | Feed inhibit was set |

Also check what the volley leaves behind: `after: KEEP` leaves aim and spin running, `IDLE` drops to 3000 RPM with yaw held, `OFF` stops the flywheel. The gate must be closed and both transfer motors stopped after the step ends.

### Stage 3: negative tests (still no balls)

- **Turret not homed, with `FIRE`:** it must not feed.
- **`feedInhibit` before the volley:** the outcome must be `INHIBITED`, even with `FIRE`.
- **Time guard:** set `latestStartS` to a small value such as 1 and start the volley after that second. It must be skipped before the gate opens.
- **STOP during a volley:** the gate closes and the transfer, flywheel and drive stop.

### Stage 4: live balls, practice field

1. Mark floor spots at three or more ranges inside 12-60 in (for example 24, 36 and 48 in from the hive opening). Test from outside the table only to see the extrapolation, not to rely on it.
2. At each spot run the same volley and record: hits out of the balls fed, the RPM and hood from telemetry, the battery voltage, and the outcome. Four balls in one volley is the normal load.
3. **Parity with TeleOp:** from the same spot, hold RT + LT in TeleOp and note the hood and RPM it uses. The volley should show the same numbers, because both use the same table. A difference means the autonomous pose is off, not the table.
4. If shots land consistently off, fix the start pose and odometry first. Only then adjust the small trims (`yawTrimDeg`, `hoodTrimDeg`, `rpmTrim`), which are the same as the GP2 trims in TeleOp. Re-measure the table with **Calibrate: Hive Shot Table** if the shooter changed.
5. Compare `readyTimeoutSeconds` against the real time to ready (watch how long AIMING lasts at each spot) and set it with margin. Measured first-ball and per-ball times are still unmeasured on this robot, so record them.

### Stage 5: inside a full auto

Run the real routine slowly in a cleared area. Check that the volley's range from the final pose of the preceding path is inside the table, that pre-aiming in parallel with the drive makes it ready sooner, and that the whole routine fits the 29 s limit with the volley's worst case (`readyTimeoutSeconds + seconds`) counted.

If Zenith autos are blocked again (flag set back to `false`), you can still check the same procedure by hand in TeleOp (RT + LT, GP1 A for the fallback) and with **Test: Auto Aim** and **Test: Intake + Index + Shooter**.

