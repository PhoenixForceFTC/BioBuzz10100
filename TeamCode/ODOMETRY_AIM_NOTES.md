# Odometry auto-aim dev notes

The robot turns its shooter toward the goal using its position from the Pinpoint.
Unlike Limelight aiming, this still works when no tag is in the camera's view. The
driver keeps drive and strafe control while it aims.

**Status**

- Compiles with the full Gradle build. The heading math was checked off the robot
  against 11 known cases.
- **Not yet run on a robot.** The Pinpoint signs and the PID gains are unverified.
- **Off until measured.** The start and goal poses are empty, so `isReady()` is false.

## Where things live

| File | Job |
| --- | --- |
| `OdometryAim.java` | Reads the Pinpoint, holds the start and goal poses, works out the heading error |
| `RotateToLimelight.java` | Turns an error in degrees into a turn power. Tag and odometry aiming share this PID |
| `testopmodethingy.java` | Reads the right trigger, picks the aiming source, mixes the turn into the drive |
| `pedro/Constants.java` | Pinpoint settings (`localizerConfig`), shared with Pedro's tuners |

## How the aim is calculated

1. `atan2(goalY - robotY, goalX - robotX)` gives the field direction to the goal.
2. Subtract the shooter's heading (robot heading plus `SHOOTER_OFFSET_DEG`).
3. Wrap to -180..180 so the robot takes the short way around.
4. Flip the sign. Pinpoint headings are positive to the left, but `turn` is positive
   to the right.

The result uses the same convention as Limelight `tx`: positive means the target is
to the right. That's why one PID can serve both. Moving the robot changes the
direction to the goal, so aim updates as you strafe.

## Setup

1. Check that the robot configuration has the Pinpoint named `pinpoint`, the name
   set in `Constants.localizerConfig`.
2. Fill in the `[MEASURE]` poses at the top of `OdometryAim.java`, in inches. Headings
   are in radians, so wrap degrees in `Math.toRadians()`.
3. Set `SHOOTER_OFFSET_DEG` if the shooter doesn't point out the robot's front.

The SDK can't supply goal positions: its BioBuzz tag library lists every cluster at
`(0,0,0)`. Measure them on the field or take them from the official field drawing.

**Quickest first test:** use the robot's starting spot as the origin. Set the start to
`new Pose(0, 0, 0)` and the goal to `new Pose(inchesForward, inchesLeft)`, measured
from there with a tape. This only works from that one spot.

**For matches:** use one field frame for all four poses. The red and blue values are
separate, so don't assume one is a mirror of the other without checking the drawing.

## Test on the robot, in this order

Keep the robot still during init, because the Pinpoint recalibrates its IMU then.

1. **Odometry signs (no driving).** Push the robot by hand and watch `Odo pose`:
   - Forward: `x` goes up.
   - Left: `y` goes up.
   - Rotate left (counter-clockwise): `h` goes up.

   If one is backwards, fix it in `localizerConfig` before going on. A flipped sign here
   makes the robot turn away from the goal.
2. **Error reading (no driving).** Use start `(0, 0, 0)` and goal `(60, 24)`. `Odo goal`
   should read about 64.6 in away with an error of -21.8°. Negative means turn left.
3. **Turn direction, wheels off the ground.** Hold the trigger and check that the wheels
   try to turn the robot toward the goal.
4. **On the floor, slow mode.** Hold the trigger from a few different angles, with your
   thumb ready to release it.
5. **Strafe while holding.** The shooter should stay pointed at the goal.
6. **Drift.** Drive around for a minute, return to the start spot, and compare `Odo pose`
   with the start pose. That gap is your aiming error late in a match.

## Tuning

The gains are in `RotateToLimelight.java` and affect both aiming sources. They haven't
been tuned yet. Raise `kP` until the robot starts to overshoot, then back off and add
`kD` to settle it. Slow mode multiplies all drive output by 0.6, including aim, so the
same gains respond differently in each speed mode. Tune in the mode you shoot in.

## Things to watch out for

- **Don't call `Constants.create()` in this TeleOp.** It builds Pedro's `Mecanum`, which
  sets motor directions and float mode. That's why `OdometryAim` builds only the
  `PinpointLocalizer`.
- **Open question: back-right motor direction.** `Constants.drivetrainConfig` says
  `REVERSE` and `testopmodethingy` says `FORWARD`. Only one can match the robot, so check
  this before relying on Pedro paths. See `DRIVETRAIN_CHECKS.md`.
- **Autonomous hand-off.** `start()` overwrites the pose with the start constant, so a
  TeleOp run after a real auto would lose the robot's true position.
- **Drift isn't corrected.** Bumps and wheel-pod slip add error for the rest of the
  match, and nothing currently corrects the pose from tags.
- **Sideways shooter offset isn't handled.** `SHOOTER_OFFSET_DEG` only covers direction.
  The aim assumes the shooter sits on the robot's center point.
- **No lead for shooting on the move.** It aims at where the goal is, without adjusting
  for the robot's speed.

## Ideas for later

- Correct the pose whenever the Limelight sees a scoring tag.
- Save the pose at the end of auto and load it in TeleOp.
- Add a driver button that resets the pose at a known spot on the field.
