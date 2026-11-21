# The `update-robot` skill

A Claude Code skill that takes code or autonomous-routine changes from your laptop to the BioBuzz Control Hub, step by step and with checks. It covers **software only**: code and Zenith autos. It does not update Control Hub firmware or the Driver Station app.

The skill itself is `.claude/skills/update-robot/SKILL.md`, with a helper at `.claude/skills/update-robot/scripts/classify-change.sh`. This page describes how to use it. It has not been run on a real Control Hub, and the Mac and Linux install steps in particular are untested.

## Using it

Either way works in Claude Code, from the repository root:

- **Type the command**, optionally followed by what the robot currently runs:
  ```
  /update-robot
  /update-robot origin/V1.0.1
  ```
- **Describe the task in plain words**, for example "push this auto to the robot", "install the APK", or "hot reload my Java change".

**Where it is available.** Project skills load from `.claude/skills/` in the repo, so you need a branch that contains it. `V1.0.1` has it (it arrived with PR #4), as does any branch cut from `V1.0.1`. If it does not appear, start a new Claude Code session in the repo or run `/reload-plugins`.

**Tell it what is on the robot.** The skill compares your changes against whatever the robot already runs. If you do not say, it asks; its default (the branch's upstream, else `origin/V1`) can be wrong.

## What it does

1. **Classifies the change** with `classify-change.sh`, which is read-only. The verdict is one of:

   | Verdict | Meaning |
   | --- | --- |
   | `FULL INSTALL` | Autos, APK assets, build files, libraries or SDK, manifest or resources, a `@Pinned` class, or a new OpMode class. |
   | `SLOTH HOT RELOAD` | Only existing Java under `TeamCode/src/main/java`. A full install also works. |
   | `NOTHING TO DEPLOY` | Only docs, tests, tools or CI changed. |

   A new OpMode class counts as a full install because it is not known whether hot reload registers it with the Driver Station.

2. **Runs the preflight checks.** None of these touch the robot:
   - `npm run zenith:validate` and `npm run zenith:test`
   - if anything under `autos/` changed: `npm run zenith:deploy -- --dry-run` (it shows you what will be written), then `zenith:deploy`, then `zenith:check`
   - `./gradlew :TeamCode:testDebugUnitTest :TeamCode:assembleDebug` (`.\gradlew.bat` on Windows)

   It stops at the first red result and reads the error to you. Three SimLoop tests that already fail on `V1` are not part of the checks.

3. **Checks the tuning gate.** It reads `PedroConfiguration.TUNING_VERIFIED` and tells you whether Zenith autos will actually START. It never changes the flag.

4. **Installs, only after you confirm the robot is ready.**
   - **Windows:** `.\tools\deploy.ps1 -Full` for a full install, or `.\tools\deploy.ps1` for hot reload. It joins the robot Wi-Fi (`10100-RC`), installs, launches the app and rejoins your network.
   - **Mac or Linux:** the Windows tasks call `powershell.exe`, so you join the Hub Wi-Fi yourself and it runs:
     ```
     adb connect 192.168.43.1:5555
     adb devices
     ./gradlew :TeamCode:installDebug
     ./gradlew :TeamCode:robotLaunchApp
     ```
     Hot reload on a Mac is `./gradlew :TeamCode:removeSlothRemote :TeamCode:deploySloth`. Both are untested on a real Hub.

5. **Verifies on the Driver Station.** It asks you to check, and reports only what you tell it:
   - The new or changed OpMode is in its group (Zenith autos are in **Generated**).
   - INIT telemetry shows the routine and profile, Pinpoint READY, and the turret home ready.
   - The Pedro line reads REQUIRED while the gate is closed, VERIFIED after it is flipped.
   - If drive code changed, run **Test: Pedro Drive** once in a cleared area.

6. **Leaves a way back.** It suggests saving the current APK before replacing it, offers a git tag for the commit now on the robot, and describes the rollback (`adb install -r <old.apk>`, or check out the known-good tag and do a full install).

It finishes with a summary: the verdict, each check's result, the `TUNING_VERIFIED` value, what was installed and how, what you still have to verify on the robot, and anything untested.

## What it will not do

- Install, or change your Wi-Fi, without asking first.
- Use hot reload for autos. Auto JSON files are APK assets and need a full install.
- Set `TUNING_VERIFIED` to `true`. That is a human decision backed by measured results (see [TuningVerificationChecklist.md](TuningVerificationChecklist.md)).
- Commit, push, tag or open a PR unless you ask.
- Install when you say a match is imminent.

## Prerequisites

- Node dependencies: `npm ci --ignore-scripts` once.
- JDK 17 and the Android SDK, as set up for the repo.
- `adb` on your PATH (installed with Android platform-tools).
- The laptop must be able to reach the Control Hub (on its Wi-Fi, or over USB-C).

## Troubleshooting

| You see | Likely cause |
| --- | --- |
| The OpMode is missing on the Driver Station | The install was hot reload instead of full, or `zenith:deploy` was skipped. |
| `adb connect` fails | The laptop is not on the Hub's Wi-Fi. Run `adb kill-server` and retry. Newer Hub firmware may need adb enabled or a first USB-C connection. |
| `zenith:validate` fails with "registry names disagree" | A command exists in only one of `AutonomousCommands.java` and `autos/robot.json`. |
| `zenith:check` says "Zenith outputs are stale" | Run `npm run zenith:deploy`. |
| START throws "Measure and verify PedroConfiguration" | `TUNING_VERIFIED` is `false`. It has been `true` since 2026-11-21, so this means the robot runs an older build, or the flag was set back. See the checklist above. |

See also [AutosAndCompetitionDay.md](AutosAndCompetitionDay.md) for the list of autos, how to push them, and competition-day tuning notes.
