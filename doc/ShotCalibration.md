# Guided hive shot calibration

Select **Calibrate: Hive Shot Table** in the Driver Station's **Test** group.
It initializes only the turret assembly (including its encoder), gate, both transfer
motors and transfer full sensor. It does not require drivetrain, intake, Pinpoint or
Limelight hardware. All controls use gamepad 1.

Use the same hive opening, height, ball type and mechanical configuration throughout
a session. Measure **horizontal distance from the turret rotation axis to the vertical
projection of the upward-facing cell opening center**, not distance to a tag, the
hive pivot, or the hive base. The ball exit is centered on the turret axis in plan
view, so this is also the horizontal muzzle-to-opening distance. Enter it on the
PLACE screen after START using Dpad left/right in 1-inch steps, then A to confirm.
From TUNE, B returns to PLACE to edit the distance; there is no typed numeric field.
Hood degrees mean 0–30 degrees of downward travel from home. Home is +15 degrees
above horizontal; 15 degrees of travel is horizontal; 30 degrees is -15 degrees.
Launch elevation = 15 - hood travel. Dpad Up raises elevation and Down lowers it;
telemetry displays both values. RPM is motor encoder RPM, not an independently measured
flywheel shaft speed. A changed drive ratio requires recalibration.

1. During INIT, allow turret homing to finish. After START, position the robot at
   a measured distance. Dpad left/right edits that distance in inches; A confirms.
   The first suggested distance is 24 inches. Suggested distances are editable.
2. Load **one ball per trial**, keeping clear of the mechanism. The intake and
   automatic transfer loading are disabled; no motion occurs to refill the queue.
   In TUNE, RS-X aims yaw. Dpad left/right changes RPM by 50; up/down changes hood
   command by 0.5 degrees. Hold RB to spin; X requests one timed feed attempt.
3. Keep RB held through the attempt. Feeding waits for stable RPM, hood settling
   and yaw tracking, then opens the gate before running the transfer stages.
   Timing uses the existing `ShootingConfiguration.firing()` estimates. Verify
   those timings on the actual robot; a timed attempt cannot guarantee one ball.
4. After the attempt, flywheel and transfer stop and the gate closes. Observe the
   result, then press A for HIT or B for MISS. X discards an uncertain/no-ball trial.
   A confirmed HIT updates the running distance average shown on the Driver Hub.
   Misses and discarded trials do not change the averages.
5. Repeat as many times as desired; there is no shot-count limit or three-hit
   requirement. Every A-confirmed HIT immediately updates the arithmetic mean of
   target RPM and hood travel at that exact entered distance. MISS and discarded
   trials do not affect the means. Changing settings or revisiting a distance
   preserves its prior confirmed samples for the current session. Telemetry shows
   the sample count and averaged RPM/launch elevation. Y moves to the next distance
   after at least one confirmed HIT, initially suggesting 12 inches farther away.
   Move the robot manually and enter its actual measured distance.
6. Collect at least two distinct distances; cover the full intended shooting range,
   with additional points where the response changes rapidly. BACK from PLACE/TUNE
   opens REVIEW. Dpad left/right queries the interpolated RPM and hood
   at one-inch intervals. Test intermediate distances physically as well. BACK
   returns to distance entry. All distance averages remain visible throughout the
   active loop, including while tuning and rating shots. Record the displayed
   rows before pressing STOP; STOP exits without saving.

LB aborts, closes the gate and stops both transfer stages and flywheel. Releasing
RB during a trial also aborts; no hit/miss is recorded. A ball already fed can still
exit. LS click in PLACE/TUNE rehomes. Hardware outputs stop in `finally` on exit.

The TeleOp uses memory and Driver Hub telemetry only. It does not read or write
calibration files, maintain trial logs, or export data. The live table displays
distance in inches, average target motor RPM, average launch elevation above
horizontal, and average downward hood travel from home. Values refresh every loop.
A new run starts empty. Keep the OpMode running in REVIEW while recording the
rows or taking a screenshot of the Driver Hub, then share those rows to enter
them into the source-code lookup table.

## Using the table

After collecting the displayed averages, enter them as ordered rows in
`ShootingConfiguration.PROFILE` in place of `ShotProfile.example()`. Use a geometry
ID matching the measured target setup. Each row uses distance in meters, downward
hood travel in degrees, average target RPM, and unknown flight time:

```java
new ShotProfile.Row(distanceInches * .0254,
        15 - averageLaunchElevationDeg, averageRpm, Double.NaN)
```

If using the displayed hood travel directly, use it as the second argument instead
of converting launch elevation. At least two distinct distances are required.
Calibration does not modify the competition profile. `ShotProfile.sample()` in
the robot code already performs linear interpolation independently
to both hood and RPM between adjacent rows. Outside the measured range, `sample`
returns `null`; callers must inhibit firing instead of extrapolating.

Closest/middle/farthest samples define two linear segments and provide estimated
settings at every distance between those endpoints. They do not establish settings
outside that interval or guarantee successful shots at unsampled distances. Test
the interpolated settings, particularly halfway between samples, and add distances
where errors appear. Averaging successful but very different angle/RPM pairs can
produce an unsuccessful intermediate pair; keep one consistent shot trajectory
and validate the averages themselves.

Use `Double.NaN` for flight time because this procedure does not measure it.
The existing planner supports stationary shots with this profile, but moving shots
still require measured flight times. Mount geometry, goal survey and timing
validation in `ShootingConfiguration` remain separately required for auto-aim.
