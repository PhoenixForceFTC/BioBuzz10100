# Robot software configuration and verification

Current competition behavior is recorded in [CompetitionTeleOp.md](CompetitionTeleOp.md), including the blue-corner start, nearest alliance cell, driver-managed hive state, and the intentional bench differences. The software includes continuous current-pose aiming, shared firing, camera configuration, and pollen detection/navigation. The team's current autonomous scope is pollen detection and navigation only; match paths, preload/scoring sequences and parking routes are deferred. Existing match-strategy scaffolding is unconfigured and is not a requirement for completing pollen navigation. Passing desktop tests does not establish physical shooting accuracy.

## Servo maximum-speed configuration

The team identifies the supply as the [goBILDA 6V Servo Power Injector, 3125-0001-0001](https://www.gobilda.com/6v-servo-power-injector-6-channel-8-15v-input/). Its six channels share a regulated 6 V output rated up to 24 A total from an 8–15 V input; this is not 24 A per channel. The manufacturer specifies full 24 A output for 2 minutes 30 seconds at room temperature, with current/thermal limiting. The team's stated 6 V servo supply therefore matches the voltage used for the ratings below. Physical load/timing checks remain deferred.

All five servos remain in positional mode. Gate, depollinator, hood and home use positional targets. Turret commands have brief smoothing; competition manual velocity uses two 10-ms filters and the position path uses 5-ms smoothing. Full-stick manual yaw uses the MAX+ rated shaft speed through the 1:1 turret gearing; smaller stick inputs intentionally request proportionally slower movement. Hood button steps select a destination; they do not impose a servo speed limit. Mechanical angle limits remain enforced.

| Mechanism | Servo | Rated shaft speed at 6.0 V | Mechanism travel estimate |
|---|---|---|---|
| Turret rotation, ports 1 and 2 | Axon MAX+ | 0.115 s/60 degrees (521.74 deg/s) | 521.74 deg/s at 1:1 |
| Hood, port 0 | Axon MAX+ | 0.115 s/60 degrees | 260.87 deg/s through 2:1 linkage; 0.0575 s for 15 degrees of hood movement |
| Depollinator, port 4 | Axon MAX+ | 0.115 s/60 degrees | 0.25875 s for 135 degrees |
| Gate, port 3 | goBILDA 2000-0025-0004 Super Speed | 0.043 s/60 degrees (1395.35 deg/s) | 0.0645 s for 90 degrees |

Sources: [Axon MAX+ specifications](https://docs.axon-robotics.com/archive/max), [Axon programmer settings](https://docs.axon-robotics.com/archive/programmer), and [goBILDA Super Speed specifications](https://www.gobilda.com/2000-series-dual-mode-servo-25-4-super-speed/). These ratings assume the specified supply voltage; load, acceleration, and internal settings affect actual motion. Two rotation servos provide additional torque, not twice the speed. PWM pulse width selects position, not motor power. The goBILDA gate specifies a 50 Hz (20 ms) PWM frame, which matches the SDK default; increasing that frequency is not a documented speed upgrade.

The Axon programmer settings must be written to each MAX+ separately; this repository cannot read or change them through the FTC `Servo` API. For maximum available output on the robot's 6.0 V servo supply, set **PWM Power to 100%**. Axon permits 100% at 6.0 V, while recommending 85% as its standard setting; at 100%, monitor temperature because thermal protection may shut the servo down. If startup acceleration must also be unrestricted, disable **Soft Start** in the programmer. Keep the existing travel, neutral, and direction settings so home positions are preserved (the depollinator requires increasing PWM = clockwise viewed from the output shaft). Servo power comes from the confirmed injector; the Expansion Hub supplies the position commands.

Keep Axon's factory **Dampening Factor** and overload protection. The manufacturer advises against changing its internal D coefficient. FTC positional servos expose position commands, not `DcMotorEx` velocity/PIDF setters; adding motor PIDF calls cannot unlock servo speed. Competition turret control uses its external Through-Bore encoder for bounded positional adjustments starting above 3 degrees and stopping below 1.5 degrees, without a repeating external PID. This does not increase the servo speed rating. The other mechanisms have no external measured position feedback, so an added software PID loop would have no measured error to regulate. See the [FTC Servo API](https://javadoc.io/static/org.firstinspires.ftc/RobotCore/10.3.0/com/qualcomm/robotcore/hardware/Servo.html).

Transfer feeding starts 0.20 s after the gate-open command, with no extra settling allowance, and turret homing still waits for encoder stability. These waits do not slow physical movement. Rated travel and hood position estimates do not prove arrival; replace timing allowances with measured loaded values before automatic shooting calibration is enabled.

## Build environment and baseline

The existing Gradle configuration uses Gradle **9.1.0**, Android Gradle Plugin **8.13.2**, FTC SDK **12.0.0**, Android compile SDK **30**, minimum SDK **24**, target SDK **28**, and Java **8** source/bytecode compatibility. Earlier verification used **Java 21.0.8**; the latest October 26 build uses Android Studio's bundled **Java 25** runtime. Configure `JAVA_HOME` to a compatible installed JDK (Java 21 is verified), and configure the Android SDK using `ANDROID_HOME` or the ignored `local.properties`:

```properties
sdk.dir=C:/your/Android/Sdk
```

From the repository root, run:

```powershell
.\gradlew.bat :TeamCode:testDebugUnitTest :TeamCode:assembleDebug
```

The hardware-review baseline passed **81 tests**. The October 23 implementation passed **118 tests** with zero failures/errors/skips and produced the debug APK. Android lint reports **0 errors and 14 existing warnings** (12 native-library alignment warnings and two unused resources). See [implementation status](HardwareReviewImplementationStatus.md). The latest test count is recorded in [CompetitionTeleOp.md](CompetitionTeleOp.md). The build retains the SDK's Java 8 compilation warnings and Gradle deprecation notices. Machine-specific SDK/JDK paths are not committed. APK output: `TeamCode/build/outputs/apk/debug/TeamCode-debug.apk`.

## Coordinates, samples, and vision

Pinpoint is connected to **Control Hub I2C port 0**, configured as `pinpoint`. The SDK hardware map selects the bus from the active Robot Configuration; no Java bus reassignment is needed.

Shot planning uses **meters, seconds, radians**. The team's field convention is center origin, +X toward blue, +Y toward the scoring table, +yaw counterclockwise. This is a team convention: tag surveys, routes, and starting poses must all use it. Robot axes are +X forward, +Y left. Drivetrain command lateral/yaw are right/clockwise positive and are converted at the navigation boundary. Turret encoder zero is servo home at position 0.5, pointing robot-right. Driver-facing turret headings increase clockwise: front 0, right/home 90, back 180, left 270 degrees; the left 265..275 degree zone is excluded. Internal encoder/solver yaw remains counterclockwise from home (right = -90 degrees in robot math). Camera angles are tx right-positive, ty up-positive. Runtime metric geometry uses each tag's independent `getTargetPoseCameraSpace()` pose: optical +X right, +Y down, +Z forward, rotated through the known camera pitch. Configured high-tag height is a classifier comparison, not an input used to infer range.

`PinpointOdometry` performs one bulk update and publishes a timestamped `MotionSample` with pose, fixed-frame X/Y velocity, angular velocity, linear/angular acceleration estimates, uncertainty, status, and field-reference validity. X/Y velocity uses the same fixed axes as Pinpoint position; it is not rotated by robot heading again. The [manufacturer's output and setPosition documentation](https://www.gobilda.com/content/user_manuals/3110-0002-0001%20User%20Guide.pdf) describes position/velocity outputs and establishing a field reference. Installed displacement/velocity signs and turning behavior remain physical verification dependencies. Constructor zero and diagnostic zero deliberately do not establish a field reference. Both competition TeleOps initialize the blue loading-corner pose from `TeleOpFieldReference`; approved autonomous strategies retain their own starting pose. Let INIT calibration finish before START.

`PoseHistory` stores one second of pose and turret yaw, interpolating across heading wrap. Vision corrections modify a translation transform rather than resetting hardware pose or differentiating position jumps. Competition TeleOp uses this corrected estimate for aiming; the match-autonomous adapter disables camera fusion and uses odometry for both navigation and shooting. Current pose/velocity older than 100 ms is rejected. Observations older than 350 ms, missing history, duplicate/out-of-order frames, mismatched target maps, and position jumps over 0.5 m are rejected. Competition TeleOp uses known field openings without a camera-freshness or observed-state firing gate. Periodic corrections are enabled using the nominal manual-derived map and documented mount, limited to one accepted correction per 0.5 s. Individual frames must pass the existing quality, height, freshness and jump checks. Legacy diagnostic adapters retain their 750-ms target-observation and 150-ms state-observation gates. The match-autonomous adapter uses odometry with its route-selected target and no mid-route vision correction. These are configurable software limits, not measured accuracy guarantees.

SDK 12 bytecode was inspected locally: `LLResult.getControlHubTimeStamp()` uses `System.currentTimeMillis()` at parsing start; `getControlHubTimeStampNanos()` merely scales that wall-clock value. The result's Limelight-local timestamp identifies duplicate frames. `PoseHistory.captureTime` bridges wall-clock receipt age to `System.nanoTime()`, subtracting capture and pipeline latency once. JSON parse time is already included in receipt age. See the [SDK latency/timestamp API](https://javadoc.io/static/org.firstinspires.ftc/Hardware/10.1.0/com/qualcomm/hardware/limelightvision/LLResult.html) for the meanings of these latency fields. Enter any measured additional transport delay in `ADDITIONAL_VISION_TRANSPORT_SECONDS`; do not add staleness or parse latency again. `VISION_TIMING_MEASURED` remains false for legacy diagnostic firing. Competition TeleOp relocalization is enabled using SDK capture/pipeline timing and the existing zero additional-transport estimate; measured transport delay can refine it.

The former MegaTag camera/robot-pose conflation and competing direct-centering loops were removed. Field relocalization uses metric per-tag observations and the selected target's nominal drawing-derived raised-tag map plus the rotating camera mount. With a verified local tag-to-opening transform, target-relative aiming uses the local odometry frame without requiring an absolute map. It never assumes a Limelight `botpose` is a camera position.

## Entering shooting calibration

The shared [transfer and shooting sequence](TransferAndShooting.md) starts both
stages 0.20 s after the gate-open command, feeds forward for 0.50 s while the gate
stays open, and then recovers for 0.35 s. The gate settles immediately as reported
by the team; no extra settling allowance is added. Stage 2 stays off outside feeding.
The feed duration and overall release timing still need loaded validation.

The latest instruction prefers drawing values over earlier team estimates. `FieldGeometry` defines the TU06 dimensions and derives an opening mid-height aim point of **1.51257 m**, tag height **1.2676124 m**, and a **60-degree** opening plane. Opening Y is nominally +/-0.38322519 m, tag-row Y +/-0.31383184 m, with individual lateral positions from the sticker drawing. The Y projection's pivot/surface references remain approximate. The earlier 1.50/1.25-m heights and 0.3906-m opening Y are superseded. Only raised geometry is used for competition aiming; drivers decide whether the selected cell is shootable. The opening-plane angle is calibration geometry, not a commanded hood/launch angle.

The manual resolves tag IDs, 36h11 family, 3.25-in. size and lateral center offsets. See [FieldReference.md](FieldReference.md) for the complete nominal map and SDK/individual-tag origin distinction. Competition relocalization now uses this map; the separate legacy tag-local-to-opening transform remains unverified and unused by competition.

Edit `ShootingConfiguration` and the mechanism constants in `TurretSubsystem`/`TurretControl`:

1. The current planar model assumes an upright camera with negligible roll. Supply `ShooterMount` with separate robot-to-pivot, pivot-to-camera, and pivot-to-muzzle offsets, camera height/pitch/yaw, and turret zero direction. Mark `measured` only after measuring them; a measured zero offset is valid.
2. Competition openings come from `TeleOpFieldReference.target`: alliance X, nominal signed Y and the drawing-derived 1.51257-m aim height. Drivers manage cell state; no detector validation or tag visibility is needed for odometry-only aiming/firing. Periodic Limelight correction uses the nominal per-tag map and SDK capture timing, subject to per-frame rejection checks. Diagnostic target-relative geometry remains separate and unverified.
3. Replace `ShotProfile.example()` with measured, ordered distance/hood/RPM rows. The geometry ID must match the selected opening. Hood is an empirical mechanism command; RPM is not ball exit speed. Rows must have finite distance, hood travel 0–30 degrees, flywheel RPM 0–3700 (positive), strictly increasing range. Flight-time fields have been removed. Interpolation is linear; endpoints never extend coverage. The legacy example rows remain uncalibrated and cannot enable automatic shots.
4. Measure gate travel, feed-to-exit delay, feed window and recovery; enter the gate-command delay in `GateSubsystem.FEED_DELAY_AFTER_OPEN_COMMAND_SECONDS` and the feed/recovery constants in `ShootingConfiguration`, then mark `RELEASE_TIMING_MEASURED`. Tune the hood travel estimate and homing travel/stability times. The hood and gate have no physical position feedback.
5. Match the physical placement to the configured blue loading-corner competition start; autonomous strategies retain explicit independent starts and diagnostic target-relative mode preserves local provenance. The team assumes no odometry drift; uncertainty is informational and has no feeding cutoff. There are no chassis speed, acceleration or turret-rate limits on automatic feeding.

Homing states are unhomed, moving home, settling, ready, and fault. INIT commands home with trim disabled, waits for travel, then requires an encoder stability window and captures zero once. START does not bypass it. Timeout inhibits automatic firing. Rehoming invalidates the current shot; diagnostic zero and servo-center override require another home cycle. **A stationary stall cannot be distinguished from home using the relative encoder. READY means estimated home, not absolute verification.** The turret never resets or changes direction/mode of the intake's shared encoder channel.

The planner computes current range and bearing from odometry and the turret/muzzle offsets, then linearly interpolates hood and RPM. It updates these commands as position and heading change. Projectile interception, future-release prediction, inherited projectile velocity and flight-time data have been removed. Stale/unreferenced poses, unsupported profile range and unreachable yaw still reject automatic data; motion itself does not.

## Driver controls and readiness

Red/blue TeleOp selection, chassis, intake, depollinator, manual hood/RPM and feed inhibit remain available. The flywheel starts at 3500 output RPM; GP2 B is unassigned.

| Input | Behavior |
| --- | --- |
| GP1 RT | Hold current-pose aim and automatic feed when calibrated, localized, homed, within 3 degrees and at the requested hood/RPM. Invalid data reports a reason and inhibits automatic feed. |
| GP1 LT or GP2 RT | Explicit manual gate/feed override; wait for gate travel, then feed continuously while held. This bypasses shot readiness, including RPM readiness. |
| GP2 LT | Highest priority: close gate and stop both transfer stages. |
| GP2 B | Unassigned. |
| GP2 X | Begin non-blocking rehome; invalidates the current automatic shot. |
| GP2 Dpad left/right | Unassigned; odometry selects the nearest alliance cell automatically. |
| GP2 RS-X | Manual yaw at the Axon MAX+ 6.0 V rating (about 521.74 degrees/second at full stick, 1:1 gearing) when auto aim is released. |
| GP2 Dpad up/down, Y/A | Existing hood/RPM steps outside automatic aim. |
| Either driver's bumpers | Intake forward/reverse; simultaneous directions stop. Bumpers override the automatic full-queue stop. |

Automatic readiness requires valid homing/target/pose/profile/timing, yaw error <=3 degrees, estimated hood tracking, and flywheel speed over the existing 150-ms readiness window. Chassis velocity, chassis acceleration and turret rate do not gate feeding. Drivers choose whether to fire while moving. Entry/retention tolerances and timed recovery avoid gate chatter. Invalid solutions stop further feeding immediately. A ball already committed may still exit; telemetry reports that possibility and **timed feed attempts**, never invented ball counts. Leaving manual feed requires automatic readiness to qualify again. Full confirmation requires two continuous HIGH seconds. LOW resets pending confirmation, but confirmed full holds both transfers stopped until gate-open feeding begins. Competition TeleOp intake runs forward with space and stops while full; either driver's bumpers override and opposing bumpers stop. Feeding bypasses full; when the sensor clears during feeding, forward intake resumes. After shooting, stage 1 automatically resumes loading when space is available; stage 2 stays off.

## Pollen camera and collection

`PollenVision.CONFIG` uses validated `PollenConfiguration` data: camera mount, explicit intrinsics or SDK calibration/fallback FOV, HSV thresholds, pickup start/travel, and separate measured flags for mount, intrinsics, color thresholds, and pickup. The team-confirmed C270 lens is **195 mm above the floor, 248 mm forward of robot center, and on centerline**, looking straight forward with its optical axis **10 degrees below horizontal**. These values are configured and mount status is measured. Color and pickup calibration remain incomplete; explicit intrinsics are not supplied. Approximate intrinsics can only drive autonomous if explicitly allowed, and color/pickup status must still be measured. With the current approximate 60-degree horizontal FOV at 320×240, the nearest unclipped contact is about 550 mm from robot center; the current primitive example uses 650-mm visible confirmation followed by at most 450 mm blind travel / 0.9 s. Pickup remains unvalidated and low priority.

Use **Test: Pollen Camera Calibration** with only `webcam_1` configured. Record blob-center/contact pixels, estimated range, measured floor distance, active intrinsics/model and rejection reason. It initializes no drivetrain, intake, turret, or Pinpoint. Clipped contours, invalid floor rays, poor blob geometry, out-of-range and discontinuous targets are rejected. Frames retain capture timestamps and age checks. `PollenNavigator` confirms close targets over distinct frames before a bounded, timed/distance-limited pickup through the blind spot. `PollenAuto` remains the focused collection routine and now respects autonomous camera eligibility.

## Competition autonomous

All autonomous programs now use **Pedro Pathing v3 + Ivy through Zenith**. [Autonomous.md](Autonomous.md) describes the pinned libraries, named commands, project files and tuning procedure. The pollen navigation experiment is in the Test group and is separate from match routines.

Author `autos/red-match.auto.json` and `autos/blue-match.auto.json` in Zenith using measured start poses and reviewed routes. Run `npm run zenith:validate`, then `npm run zenith:deploy`. Existing match entry points load those names; additional routines get generated Ivy OpModes. **No competition routes have been supplied**; missing files fail during INIT before hardware initialization. `AutonomousStrategy`, `WaypointFollower` and `CompetitionAutonomousController` have been removed.

`BioBuzzRobot` registers intake, loading, aiming, timed shooting, bounded pollen collection, and shutdown commands. Zenith preserves supported curves, markers, parallel modes, branches and current-pose starts. Planned parking and fallback behavior belong in the file; there is no automatic replacement of the old controller's park policy or live obstacle avoidance. Pose loss, the 29-second deadline, completion, STOP and exceptions stop outputs; motors stop before camera shutdown. Pedro tuning remains inhibited by `PedroConfiguration.TUNING_VERIFIED = false` until physical verification. Initial dynamics and editor dimensions are explicit unmeasured examples.

The `aim` command enables continuous autonomous odometry aiming; `shoot(seconds)` alone requests feeding through the shared readiness and gate timing logic. There is no extra motion gate or manual-feed bypass. A failed shot solution does not change the driving route. Timed attempts never imply exact ammunition count.

## Verification scope

JUnit tests cover homing travel/stability/timeout/rehome, loop-rate independence, profiles and geometry, delayed/duplicate vision and rotating mounts, heading wrap, correction without velocity spikes, target loss/reacquisition, current-pose range/bearing interpolation, gate/feed timing, override priority, full-queue behavior and camera eligibility/clipping/continuity. Autonomous tests exercise Pedro unit conversions, wheel signs/power caps, holds/path speeds, pollen targets/replanning and limits, and Zenith curves, markers, parallel timeouts, branches, deferred starts, cancellation and invalid files.

Keep using isolated mechanism tests before combined operation. Current-pose aiming diagnostics in competition TeleOp require drivetrain/Pinpoint, turret/Limelight, gate and transfer hardware. Physical measurements and shot-table checks remain the separate dependencies listed in `SoftwareImplementationPlan.md`.

### Turret manual hold and external encoder backup

Competition GP2 RS-X requests linear yaw velocity up to the rated 6 V speed of 521.74 deg/s through 1:1 gearing. Two 10-ms velocity filters briefly ease acceleration changes; the shared 5-ms position-command smoothing softens target changes. These command filters do not guarantee physical acceleration or jerk. The positional servos use their own internal feedback. On stick or auto-aim release, competition control captures measured yaw once and discards queued target travel; subsequent encoder noise cannot move that hold target. Diagnostic tests retain their previous commanded-target hold.

Competition correction starts only above 3 degrees, after same-direction error qualifies for 250 ms at measured turret speed <=3 deg/s. While active it makes bounded position adjustments with another settled interval between them. It stops adjusting below 1.5 degrees and retains the successful offset; it does not restart until error exceeds 3 degrees. Total offset remains bounded to 25 degrees and mechanical travel. Target changes above 3 degrees reset the correction episode; small odometry target noise does not. These correction qualifications do not gate feeding. Automatic feed accepts <=3-degree yaw error without any turret-rate gate.

GP1 RT tracks the nearest alliance cell using odometry even without a shot profile. Hood/RPM interpolation and automatic firing await profile/release timing. There is no camera-centering fallback in competition TeleOp. The turret-only test and other diagnostic adapters remain separate.

### Turret and Limelight mounting

The team's corrected mounting description has the Limelight facing the same direction as the muzzle (relative yaw 0 degrees), pitched 30 degrees above horizontal, with its lens 335 mm above the floor. The turret pivot is centered left/right and 50.68184865 mm forward of robot center. With the muzzle pointing robot-forward, the camera is centered left/right and 99.53073974 mm behind the turret pivot. Its displacement rotates with the turret. The camera is therefore 48.84889109 mm behind robot center only when the muzzle points robot-forward. These confirmed values are recorded in ShootingConfiguration.MOUNT with measured=true. Target geometry, shot profiles and timing retain their separate calibration requirements.

The ball exit is centered on the turret rotation axis in plan view. Hood movement does not move the camera or significantly move the exit position; it changes the exit angle. Home (servo 0.511268) is the new +15-degree command reference, 2 hood degrees below the old home. Increasing PWM lowers elevation to -15 degrees; decreasing PWM toward home raises it toward +15 degrees. The internal hood command and exported hood travel remain downward travel from home, now 0–30 degrees. Launch elevation = 15 - hood command. With the existing 2:1 linkage and 355-degree programmed servo range, the endpoints are approximately 0.511268 and 0.680282 servo position (60 degrees of shaft travel away from the upper stop). Dpad Up lowers elevation, Down raises it; calibration telemetry shows both hood travel and launch elevation. This is commanded geometry, not a measurement of the actual ball trajectory. ShotProfile remains an empirical distance-to-hood/RPM table; the planar solver does not require muzzle height or assume a ballistic model. Exit height would be needed for a separate ballistic model. Revalidate shot tables against this corrected geometry.

Legacy diagnostic fallback aiming converts camera bearings to shooter bearings and wraps yaw while retaining the current travel-limit side inside the excluded left sector. Existing observation freshness requirements still apply. Manual shot calibration does not use the camera.



## October 23 hardware-review implementation

See [HardwareReviewImplementationStatus.md](HardwareReviewImplementationStatus.md) for the implementation/verification matrix and remaining measurements. `RobotLoop` owns MANUAL Lynx bulk caching and exactly one clear per INIT/active loop in all team OpModes. The SDK resets bulk mode to OFF between OpModes. Java loop period/work telemetry is separate from Pinpoint device-cycle telemetry. Turret, queue and motor samples are cached; intake/transfer current reads are throttled to 5 Hz. Shutdown attempts every remaining stop/close even if one device throws.

The former stationary qualification and motion-interception code has been removed. Accepted vision uncertainty still reports measurement quality, a 25-mm floor and uncorrected fusion residual. No drift growth, missing-camera penalty or uncertainty firing cutoff remains; pose-reference changes invalidate vision history.

In legacy diagnostic adapters, `GOAL_OBSERVATION_VALIDATED=false` keeps unvalidated camera settings and classifier tolerances out of automatic feeding. Competition TeleOp does not classify hive state and does not use that legacy flag to disable camera correction; it validates individual observations instead. The level-chassis assumption is explicit because no chassis pitch/roll feedback is supplied. `LOWERED_TAG_HEIGHT_METERS` and `LOWERED_TAG_PLANE_DEG` remain NaN: unsupported low poses report TRANSITIONING, not a fabricated calibrated LOWERED result. Opening-plane 60 degrees remains distinct from the underside tag plane's raised expectation of 30 degrees.

Calibration is display-only: BACK opens REVIEW for interpolated previews and live averages. No button saves files, and stopping discards the in-memory session. Record distance, flywheel-output RPM and labeled launch angle/hood travel from the Driver Station, then enter the agreed averages in the profile. Historical export/parser utilities remain unused by OpModes.
