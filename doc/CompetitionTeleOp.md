# Competition TeleOp decisions - October 26, 2026

These settings apply to both Red and Blue competition TeleOps. Standalone diagnostics and primitive autonomous examples retain their own procedures unless explicitly listed here.

Desktop verification on October 26 passed all 153 tests (zero failures, errors or skipped tests), `:TeamCode:assembleDebug` and `:TeamCode:lintDebug` (0 errors, 14 existing warnings). Coverage includes correction hysteresis and persistent bounds, current-pose aiming during motion, 3-degree readiness, stage-2-off loading, 0.20 s gate-command delay and longer feed timing, all 16 tag coordinates and their fusion, and a 180-second no-camera run without an automatic-feeding timeout. This does not measure loaded servo stopping behavior or shooting accuracy. No deployment was performed.

| Item | Competition behavior | Intake + index + shooter test |
| --- | --- | --- |
| Flywheel | Starts at 3500 output RPM; Y/A steps 50 RPM; GP2 B unassigned | RT hold, initially 2800 RPM; Y/A steps 100 RPM; B stops motors |
| Full queue | Stop intake and transfers after 2 s continuous HIGH; bumpers override intake | Preserve intake reversal when full while loading |
| Transfer loading | Stage 1 loads; stage 2 stays off until gate-open feeding | Same stage-2 rule |
| Manual feeding | GP1 LT or GP2 RT bypasses shot readiness; continuous after gate travel; GP2 LT inhibits | RT + LT requires bench readiness; timed feeding |
| Timed feeding | Stable readiness, gate-open command, 0.20 s delay, 0.50 s feed, then 0.35 s recovery | Same timing; retain bench 4-degree / 3-degree-per-second yaw checks |
| Turret release | Capture measured yaw once on stick/auto-aim release, discarding queued travel | Preserve diagnostic commanded-target hold |
| External yaw correction | Enabled: start above 3 degrees after 250 ms at <=3 deg/s; stop adjusting below 1.5 degrees | Disabled by default |

The gate stays open throughout the 0.20 s command-to-feed delay and 0.50 s timed
feed, about 0.70 s total per uninterrupted attempt. Both transfers run forward
together during feeding; stage 2 stays off during loading and gate travel.
See [shared transfer sequence and diagnostic controls](TransferAndShooting.md).

GP1 RT enables odometry-based yaw aiming and calibrated hood/RPM selection. The nearest cell of the selected alliance is chosen continuously: audience cell for field Y < 0, far cell for Y > 0, previous selection at exactly zero. Crossing halves resets automatic feed readiness. D-pad left/right no longer selects a competition target. The robot does not classify hive state in this mode; drivers decide whether the selected cell is raised and when to request shooting. Manual yaw remains available when RT is released.

The field origin is its center; +X points toward blue and +Y toward the far/scoring-table end. Both TeleOps currently start at the blue loading-zone end: rear against the blue wall, one side against the audience wall, front facing red (heading 180 degrees). With center-to-rear 262.481 mm and center-to-side 158.35 mm, the center pose is (1.566319, -1.670450) m. Pinpoint pod offsets remain +32/-74 mm. This placement assumption must match how the robot is actually positioned during INIT. Diagnostic test OpModes do not inherit this starting pose.

Opening X is +/-0.32385 m (blue/red). Drawing-derived nominal aim height is 1.51257 m (opening mid-height), tag height 1.2676124 m, opening Y magnitude 0.38322519 m and tag-row Y magnitude 0.31383184 m. `FieldGeometry` and `HiveTagGeometry` share the TU06 dimensions. The Y values retain the approved half-length projection, whose exact pivot-to-surface transform still needs CAD/field verification. See [field reference](FieldReference.md).

Limelight is enabled as a periodic translation correction source in competition TeleOp (minimum 0.5 s between accepted corrections), never a direct turret-centering or hive-state firing gate. It uses the nominal tag map and documented mount, rejects invalid/stale frames, incompatible tag heights and position jumps over 0.5 m, and uses SDK capture/pipeline latency. The team requests a no-drift assumption: no elapsed-time uncertainty growth, camera-loss penalty or uncertainty cutoff blocks feeding. Uncertainty remains telemetry. Stale/invalid odometry still rejects automatic aiming/feed. Pinpoint itself supplies odometry, not an independent absolute relocalization measurement.

Yaw tracking works from odometry even before the shot table is entered. Distance-based hood/RPM needs the empirical profile; automatic feeding also needs release timing validation. Send distance, average flywheel-output RPM and average launch angle (or clearly labeled hood travel). Linear interpolation is already implemented between rows; out-of-range requests are rejected. Projectile lead, flight-time data and dedicated route firing flags have been removed. Range, yaw, hood and RPM still update from the current pose while moving. Calibration only displays session averages on the Driver Station; stopping discards the in-memory session and no files are saved.

The Axon MAX+ positional controllers perform normal motion. Full manual stick requests the rated 6 V shaft speed, 60/0.115 = 521.74 deg/s at 1:1; steady speed is linear outside the 0.08 deadband. Two short 10-ms velocity filters ease command acceleration changes; a 5-ms position-command filter softens destination changes and stops. Release captures the encoder once, not continuously. External correction applies bounded positional adjustments, allowing a 250-ms settled interval between them, until error drops below 1.5 degrees. The successful offset is retained to hold position and is capped at 25 degrees. It does not run a continuous PID. Target changes above 3 degrees reset qualification; smaller odometry changes do not constantly restart it. Actual loaded acceleration, jerk, speed and stopping distance depend on the servos' internal controllers and mechanics. Competition feed readiness accepts yaw error <=3 degrees and the existing hood/RPM checks. There is no chassis-speed, turret-speed, acceleration or stationary qualification gate. The 150-ms angle/RPM readiness window, timed recovery and physical gate travel remain. Drivers decide when to take the risk of firing while moving.

Gate direction, depollinator direction and hood home retain the installed implementation: gate 0.5 closed to 0.2 open with REVERSE direction (90 degrees CW); depollinator 30 to 165 degrees (135 degrees CW); hood home approximately 0.511268 with 30 degrees downward travel to approximately 0.680282. The turret-only test is unchanged. Pollen collection and general autonomous work remain deferred primitive examples. The match-autonomous adapter now uses odometry for both navigation and shooting, retains the route-selected target, and performs no camera correction or direct visual aiming during the route. It continuously updates aim/RPM while navigating, and requests feed only in scoring/preload states without an extra motion gate. No end-of-route relocalization has been added without an approved route and verified landmark map.


## Remaining calibration and intentional differences

The known competition behavior mismatches in this review are resolved, including the former time-based uncertainty cutoff. This is desktop verification, not a claim that the installed robot has no defects. No additional competition feature is required before calibration.

- Supply at least two measured distances covering the intended range, with average flywheel-output RPM and labeled launch elevation or hood travel. The competition profile is still an uncalibrated example, so automatic calibrated feeding remains inhibited.
- Confirm loaded gate/feed/recovery timing. Current settings are 0.20 s after the gate command, 0.12 s feed-to-exit estimate, 0.50 s feed and 0.35 s recovery. `RELEASE_TIMING_MEASURED` remains false until physical validation; no value was declared measured from a field drawing.
- Check that Limelight pipeline 0 uses 36h11, 3.25-in. individual tags and returns valid metric tag poses. Java enables periodic correction using the documented 335-mm-high, 30-degree-upward camera mount; camera settings were not remotely changed. A live test must verify the transform and capture timing. Translation corrections update the aiming pose; Pinpoint heading remains authoritative.
- Opening/tag Y coordinates remain drawing-model approximations until the pivot-to-surface CAD transform or an installed-field reference is available. Relative tag spacing and the source drawing dimensions are already entered; no need to remeasure sixteen unrelated tags.

The combined intake/shooter bench test intentionally retains 2800-RPM startup, its own RPM adjustment steps, B-to-stop, full-intake reversal and 4-degree/3-degree-per-second yaw readiness. Competition retains 3500 RPM, full-intake stop, no B stop and angle-only 3-degree feeding readiness. Turret-only diagnostics, pollen pickup tuning and match routes remain deferred as requested.
