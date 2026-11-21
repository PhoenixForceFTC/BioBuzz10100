# Pedro tuning verification checklist

> **Status 2026-11-21:** `TUNING_VERIFIED` was set to `true` on the team's instruction without these checks being recorded here. The tables below are still blank. Fill them in as the checks are run; they remain the way to confirm the tune, and the flag can be set back to `false` at any time.

Use this to earn the right to set `PedroConfiguration.TUNING_VERIFIED = true`. Until it is true, every Zenith routine refuses to START (`BioBuzzRobot.onStart` throws *"Measure and verify PedroConfiguration before running autonomous"*). Copy this file into the pull request that flips the flag and fill it in. The background is in [Autonomous.md](Autonomous.md#before-robot-motion).

The drivetrain dynamics, braking and gains were already measured on this robot on 2026-11-19 (Foresight tuner, second run). This checklist verifies them; it does not re-measure them. Re-run the Foresight Tuner only if the robot's mass, wheels or drivetrain changed since then.

| | |
| --- | --- |
| Tester / date | |
| Build installed (commit) | |
| Battery voltage at start / end | | 
| Floor surface | |
| Area cleared (about 8 ft by 8 ft, 6 ft ahead of the robot), someone holding STOP | ☐ |

Record the battery voltage: the dynamics depend on it, so a low battery can make a good tune look bad.

## 1. Sign checks

| Check | Expected | Result |
| --- | --- | --- |
| **Test: Pinpoint Odometry Only**, GP1 A zeroes, then push forward | +X | ☐ |
| Push left | +Y | ☐ |
| Spin counterclockwise by hand | +heading, X and Y stay within a few cm | ☐ |
| **Test: Pedro Drive**, press A (12 in forward) | Robot drives forward and stops; odometry reads about +12 in X | ☐ |
| Press B (90° CCW hold) | Robot turns counterclockwise; odometry heading about +90° | ☐ |
| Press X, or move a stick while it runs | The robot stops at once | ☐ |
| INIT telemetry line "Pedro v3 ... tuning" | Reads VERIFIED while the flag is `true` (REQUIRED if it is set back) | ☐ |

## 2. Pedro web tuner Tests

Connect to the robot Wi-Fi and open `http://192.168.43.1:10158`. Run the **Tests** procedures; they use the installed follower at `MAX_WHEEL_POWER` (1, no cap), like match routines. Watch for overshoot and for the robot driving back after a stop.

The repo does not define an acceptable miss, so the team sets the limits below before running. A natural choice is the follower's own end tolerances: **0.5 in**, **3°**, **1 in/s** (`PedroConfiguration`).

| Test | What to look for | Limit set by the team | Measured (stop error in, heading error deg, overshoot in) | Pass |
| --- | --- | --- | --- | --- |
| Hold | Holds position when pushed lightly; no oscillation | | | ☐ |
| Line | Stops on the target; no overshoot and no drive-back | | | ☐ |
| Curve | Follows the curve and ends on the target | | | ☐ |
| Odometry | Pose stays consistent after the moves above | | | ☐ |
| Driving | Smooth start, stop and turn | | | ☐ |

## 3. Footprint and height

| Item | Value | Status |
| --- | --- | --- |
| Chassis, depollinator up | 396 mm long x 374 mm wide (15.591 x 14.724 in) | Measured 2026-11-21, applied in `autos/robot.json` |
| Width with the depollinator down | 374 mm (14.724 in) | Measured, applied |
| Length with the depollinator down | 537 mm (21.142 in) | Measured, **not applied**: see section 5 |
| Robot height | Unknown; `robot.json` still has the 18 in placeholder | ☐ Measure |
| Pinpoint tracking point is the chassis centre | Not confirmed (pod offsets +32 / -74 mm) | ☐ Confirm |

## 4. Flipping the flag

Only after sections 1-3 are filled in and every row passes.

1. In `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/PedroConfiguration.java`, change `TUNING_VERIFIED = false` to `true`. Make this its own commit, in a PR that includes this filled-in checklist.
2. Run `npm run zenith:validate`, `npm run zenith:deploy`, `npm run zenith:check`, then `./gradlew :TeamCode:testDebugUnitTest :TeamCode:assembleDebug`. No test pins the flag, so nothing should change.
3. Install the build. INIT must now show "tuning VERIFIED". Run **Test: Pedro Drive** once more from this build.
4. Run one low-speed Zenith routine in a cleared area (for example `test-line` from `doc/ZenithTestRoutines.md`, added by PR #3), with someone holding STOP.
5. Rollback: set the flag back to `false` and redeploy. Do this immediately if any later run drives back, overshoots or loses localization.

| Sign-off | |
| --- | --- |
| Verified by / date | Declared verified on the team's instruction, 2026-11-21. **No test measurements are recorded in this file;** the tables above were not filled in. |
| Reviewed by / date | |

## 5. Applying the depollinator-down length later

The validator checks the whole route against the expanded box. Zenith models only a box centred on the chassis, but the depollinator extends from **one end only** (which end is not recorded). Applying 21.142 in as a centred box overstates the clear end by about 2.8 in. That makes the far-wall draft starts infeasible by 0.17 in: the box does not fit between the centre line and the flower foot.

Open question: which end does the depollinator extend from? Once it is known, decide how to model it (an offset box is not supported today), then apply the measured length and make these small nudges to the draft coordinates (placeholder coordinates, each 0.5-0.6 in):

| Draft | Change |
| --- | --- |
| `red-audience` | `gardenPath` end x -62 -> -61.4; `flowerPath` end y 57 -> 56.5 |
| `red-far` | start x -10 -> -10.6; `flowerPath` first control x -10 -> -10.6; `flowerPath` end y 57 -> 56.5 |
| `red-far-cycle` | start x -10 -> -10.6; `farFlowerPath` first control x -10 -> -10.6 and end y 57 -> 56.5; `audienceFlowerPath` end x -57 -> -56.5; `gardenPath` end x -62 -> -61.4 |

Even with these, the two far-wall starts stay 0.17 in short with a centred 21.142 in box, so the end direction (or a start pose that is not squeezed by the flower foot) has to be settled first.
