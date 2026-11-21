---
name: update-robot
description: Get code or autonomous-routine changes onto the BioBuzz Control Hub safely. Decides full APK install versus Sloth hot reload, runs the preflight checks (Zenith validate/deploy/check, unit tests, build), installs over adb, and verifies on the Driver Station. Use for "update the robot", "push this to the robot", "deploy the auto", "install the APK", "get this onto the Control Hub", "hot reload".
---

# Update the robot

Software updates only: code and Zenith autos to the Control Hub. It does not update Control Hub firmware or the Driver Station app. Follow the steps in order and stop at the first red result. Report what each step actually printed; never say "done" for a step you skipped.

## Hard rules

- **Ask before touching the robot.** Installing restarts the Robot Controller app. Before any `adb` or install command, confirm with the user that the robot is powered, the Hub is reachable, and nobody is in a match queue or about to drive. Building and checking are always safe to run without asking.
- **Ask before changing Wi-Fi.** Joining the Hub's network disconnects the laptop from its normal one. On Windows `tools/deploy.ps1` does the switch for you and switches back; on a Mac the user joins the Hub network themselves.
- **Never set `PedroConfiguration.TUNING_VERIFIED` to `true`.** That is a human decision backed by measured results (`doc/TuningVerificationChecklist.md`). Only report its value.
- **Never commit, push, tag or open a PR unless the user asks.** Offer it at the end.
- **Never use Sloth hot reload for autos.** Auto JSON files are APK assets and need a full install.
- **No installs close to a match.** If the user says a match is imminent, say so and let them decide.

## Step 1: Know what changed

Run the helper from the repo root. It is read-only.

```bash
.claude/skills/update-robot/scripts/classify-change.sh <what-is-on-the-robot>
```

Pass the commit, tag or branch the robot currently runs. If the user does not know, ask; the default (the branch's upstream, else `origin/V1`) may be wrong. Uncommitted and untracked files always count. It prints one verdict:

| Verdict | Meaning |
| --- | --- |
| `FULL INSTALL` | Autos, APK assets, build files, SDK/libraries, manifest/resources, a `@Pinned` class, or a **new OpMode class**. Whether hot reload registers a new OpMode is unverified, so it is treated as full. |
| `SLOTH HOT RELOAD` | Only existing Java under `TeamCode/src/main/java`. A full install also works. |
| `NOTHING TO DEPLOY` | Only docs, tests, tools or CI changed. Tell the user no robot update is needed. |

## Step 2: Preflight (always, no robot needed)

```bash
git status -sb                         # branch and uncommitted files: tell the user what will be built
npm ci --ignore-scripts                # only if node_modules is missing
npm run zenith:validate                # expect "0 Zenith error(s)"
npm run zenith:test                    # Zenith deployment tests
```

If anything under `autos/` changed:

```bash
npm run zenith:deploy -- --dry-run     # show the user which assets and OpMode stubs it will write or remove
npm run zenith:deploy                  # after the dry run looks right
npm run zenith:check                   # must report 0 writes, 0 removals
```

Then build and test (use `./gradlew.bat` on Windows):

```bash
./gradlew :TeamCode:testDebugUnitTest :TeamCode:assembleDebug
```

Report the unit-test count and any failure. The SimLoop suite (`:TeamCode:simTeleOpTest`) is not part of preflight; three of its tests failed on `V1` before these changes (see `doc/CodeReview-2026-11-21.md`, item M1).

If validation fails, read the error. Common ones: a condition wait needs `timeoutS`; `Java/robot.json registry names disagree` (a command is in only one of `AutonomousCommands.java` and `autos/robot.json`); `Zenith outputs are stale` (run `zenith:deploy`). Never edit `TeamCode/src/main/assets/autos/` or `zenith/generated/` by hand; they are overwritten.

## Step 3: Check the gate

Read `PedroConfiguration.TUNING_VERIFIED` (it has been `true` since 2026-11-21, set on the team's instruction without the checklist results being recorded; say so if asked). If it is `false`, tell the user every Zenith auto and the Pollen test will refuse to START (*"Measure and verify PedroConfiguration before running autonomous"*). Only the fallback autos (`Auto: Red/Blue Fallback`) and the ungated `BioBuzzAuto` can run. Installing is still fine; do not change the flag.

## Step 4: Install (after the user confirms the robot is ready)

**Windows** (does the Wi-Fi switch, install and app launch):

```powershell
.\tools\deploy.ps1 -Full     # full install
.\tools\deploy.ps1           # Sloth hot reload
```

The Android Studio run configurations "Robot - Full Install" and "Robot - Sloth Deploy" run the same tasks.

**Mac or Linux** (these tasks call `powershell.exe`, so use the manual steps; **untested on a real Control Hub**). The user joins the Hub Wi-Fi first (profile `10100-RC` by default), then:

```bash
adb connect 192.168.43.1:5555
adb devices                                        # the Hub must be listed as "device"
./gradlew :TeamCode:installDebug                   # full install; removeSlothRemote runs first
./gradlew :TeamCode:robotLaunchApp                 # starts the Robot Controller app
```

Hot reload on a Mac is likewise untested: `./gradlew :TeamCode:removeSlothRemote :TeamCode:deploySloth`.

If `adb connect` fails: confirm the laptop is on the Hub's Wi-Fi, run `adb kill-server`, and retry. Newer Hub firmware may need adb enabled or a first USB-C connection. After the install, remind the user to rejoin their normal Wi-Fi.

## Step 5: Verify on the Driver Station

Ask the user to check, and report only what they tell you:

1. The new or changed OpMode appears in its group (autos from Zenith are in **Generated**; titles come from each routine's `title`). If an OpMode is missing, the install was probably hot reload instead of full, or `zenith:deploy` was skipped.
2. INIT telemetry shows the routine name and profile (*Routine / profile*), Pinpoint READY, and the turret home ready.
3. The Pedro line reads **REQUIRED** while the gate is closed and **VERIFIED** after it is flipped.
4. If drive code changed, run **Test: Pedro Drive** once (A = 12 in forward, B = 90° hold, X stops) in a cleared area.

## Step 6: Leave a way back

- Before replacing a working install, save its APK: `cp TeamCode/build/outputs/apk/debug/TeamCode-debug.apk <somewhere-safe>/robot-<date>-<commit>.apk`.
- Offer, but do not do unless asked, a git tag on the commit now on the robot (for example `robot-YYYYMMDD-N`).
- Rollback: `adb install -r <old.apk>` (add `-d` to allow a downgrade), or check out the known-good tag and do a full install.

## Competition-day notes

- Do not run the Pedro Foresight Tuner in the pit (full power, lots of clear floor); do not retune Pedro between matches.
- Use hot reload for Java tweaks and a full install only for autos, with time to re-verify (Step 5).
- Dashboard values (`192.168.43.1:8080/dash`) do not persist; write final numbers into code afterwards.
- Check the event's wireless rules before connecting near the field.
- Keep a fallback auto ready, and record each run's outcome, battery voltage and the "Last bounded action" line.

## Report format

Finish with a short summary: what changed and the verdict from Step 1; each preflight command and its result; the `TUNING_VERIFIED` value; what was installed and how (or that it was not, and why); what the user still has to verify on the robot; anything untested. Cross-reference `doc/AutosAndCompetitionDay.md` and `doc/TuningVerificationChecklist.md` where they exist on the branch.
