# Drivetrain test routines

Four Zenith routines for checking basic robot motion. They live in `autos/` and appear on the Driver Station in the **Generated** group with titles starting `TEST:`. They use the cleared-area field (`autos/field/pollen-test.field.json`) and a **local frame**: the robot's start mark is (0, 0), +X is the robot's front at START and +Y is its left. They are not field positions, so run them on any clear floor, not only on the competition field. They were built to be the regression suite once Pedro tuning is verified (see "Before you run").

| Routine | Title on the Driver Station | What it does | Estimated time |
| --- | --- | --- | --- |
| `test-corners` | TEST: Four corners (60 in square) | Tape a 60 in square with the start mark on one corner. Drive its four corners at 50% speed with heading held at 0: forward, strafe left, backward, strafe right, back onto the mark. Pause 1 s at each corner. | 14.7 s |
| `test-lap` | TEST: Lap x2 of a 48 in square (drift) | The same pattern on a 48 in square, twice, at 60% speed, pausing 1 s on the start mark after each lap. | 18.1 s |
| `test-line` | TEST: Line 48 in out-and-back (30/60/100%) | Drive 48 in straight ahead and back at 30%, 60% and 100% speed, pausing 1 s at each end. | 20.3 s |
| `test-turns` | TEST: Turns in place (90 deg steps) | No translation. 2 quarter turns CCW, 2 CW, then 4 CCW and 4 CW, pausing 1 s between groups. | 10.3 s |

Times are Zenith's estimates, using several unmeasured robot numbers (acceleration, angular speed), so treat them as rough. All four fit the 29 s autonomous limit with margin (the slowest case, `test-line`, is about 24 s).

## Volley tests

Two more routines test the shooting procedure (`volley`). Unlike the drivetrain tests they use the **real competition field** and the real red audience-side start (robot facing -X, left side on the audience wall, back about 1.4 in from the centre line), because the turret aims at the hive cell by field position. From there the audience cell is **about 49.6 in** away, inside the measured shot table, so the shot should show **about 3200 RPM and 5.7° of hood travel**. A bench run can sit on blocks anywhere: the authored pose only decides where the turret thinks the hive is.

| Routine | Title on the Driver Station | What it does | Estimated time |
| --- | --- | --- | --- |
| `test-volley` | TEST: Volley from the wall start (bench or live) | Pre-aim and spin up for 2 s, one volley with `onTimeout: SKIP` (never fired unready), then a 4 s pause to read the outcome. With balls loaded and a ready shot it feeds for 2.5 s; with no balls nothing is launched. | 11.5 s |
| `test-volley-checks` | TEST: Volley checks, NO BALLS (inhibit, fallback, normal) | Three volleys with a 3 s pause after each. (1) Feed inhibit set: expect `INHIBITED`. (2) `FIRE` with a zero ready timeout: expect `TIMEOUT` "Fed before READY", with the gate and both transfer motors running together for 1 s. (3) A normal `SKIP` volley: `SUCCESS` or `TIMEOUT` "SKIPPED". | 18.0 s |

**Load no balls for `test-volley-checks`:** check 2 uses `FIRE`, which feeds with readiness bypassed. Read *Last bounded action* during each pause; it shows only the latest result. The full procedure, expected values and live-ball stages are in `doc/AutosAndCompetitionDay.md` (section 4, on PR #4's branch). The unit test `deployedVolleyTestRoutinesBehaveAsTheirNotesSay` runs both files through Zenith's native builder and checks the inhibit, fallback and skip behaviour against fake hardware.

## What each one tells you

- **Corners:** the miss at each corner shows whether odometry scale and direction are right in all four directions. The forward and backward legs use the X pod, the strafe legs the Y pod, so a bad Y-pod offset shows up only on the strafe legs.
- **Lap:** the miss on the start mark after lap 1 and lap 2. If it grows from one lap to the next, odometry is drifting (pod, offset or IMU).
- **Line:** overshoot past the 48 in mark and the miss back at the start, at three speeds. Overshoot that grows with speed means braking is under-modeled.
- **Turns:** after each group the robot should face the heading stated in the step note. A full turn that does not end on the start heading means heading odometry is off. Put tape on the chassis front so you can see it.

Measure with tape marks and a ruler and write the numbers down per run. This repo does not define an acceptable miss; the team sets that.

## Before you run

1. **Pedro tuning must be verified.** Every Zenith routine refuses to START while `PedroConfiguration.TUNING_VERIFIED` is `false`, so until then use **Test: Pedro Drive** and the Pedro web tuner's Tests (see [Autonomous.md](Autonomous.md)). These routines are the next step after those checks pass, not a substitute for them.
2. Clear about 8 ft by 8 ft (6 ft ahead for the line test), with someone holding STOP. Start at low speed: if the first run looks wrong, edit `speedFraction` in the editor, then redeploy.
3. These wrappers run the **full robot**, so the turret, Limelight, camera and the rest must be connected. INIT still homes the turret and tries tag relocalization. If the hive tags are visible and disagree with the local start pose you may see **CHECK PLACEMENT** in telemetry; the refinement only applies a small correction, so for these tests it is a warning to read, not to fix (untested on the robot).
4. Place the robot on the start mark, facing +X, before INIT, and keep it still until the Pinpoint finishes calibrating.

## After you run: do not start TeleOp straight away

The full-robot autonomous saves its final pose for TeleOp (valid for 60 s, same alliance). These routines are in a local frame, but the full robot treats them as field-referenced, so a TeleOp started within 60 s would load a **wrong pose** (about the test's start mark) and aim from it. Either wait more than 60 s, restart the Robot Controller app (this clears it), or press **GP2 D-pad Left** (corner rehome) in TeleOp INIT. This is a limitation of the generated wrappers, not of the routines; a hand-written local-frame wrapper like the pollen test's would avoid it.

## Editing and regenerating

Edit them in Zenith like any routine (see [ZenithAutoGuide.md](ZenithAutoGuide.md)), then run `npm run zenith:validate`, `npm run zenith:deploy`, `npm run zenith:check` and a build. `AutonomousIntegrationTest.deployedDrivetrainTestRoutinesRunToCompletionAndLeaveEverythingStopped` loads all four real files through Zenith's native builder and checks the path and hold counts and that everything ends stopped, so an edit that changes a routine's shape will fail that test on purpose; update the expected counts if you change them deliberately.

Not verified: none of these has driven a robot. The tests use a fixture follower, not real motion.
