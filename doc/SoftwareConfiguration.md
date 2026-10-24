# Robot software configuration and verification

The software includes motion-compensated aiming, shared firing, camera configuration, and pollen detection/navigation. The team's current autonomous scope is pollen detection and navigation only; match paths, preload/scoring sequences and parking routes are deferred. Existing match-strategy scaffolding is unconfigured and is not a requirement for completing pollen navigation. Passing desktop tests does not establish physical shooting accuracy.

## Servo maximum-speed configuration

The team identifies the supply as the [goBILDA 6V Servo Power Injector, 3125-0001-0001](https://www.gobilda.com/6v-servo-power-injector-6-channel-8-15v-input/). Its six channels share a regulated 6 V output rated up to 24 A total from an 8–15 V input; this is not 24 A per channel. The manufacturer specifies full 24 A output for 2 minutes 30 seconds at room temperature, with current/thermal limiting. The team's stated 6 V servo supply therefore matches the voltage used for the ratings below. Physical load/timing checks remain deferred.

All five servos remain in positional mode. The code sends final gate, depollinator, hood, home, and automatic yaw targets directly, without an artificial motion ramp. Full-stick manual yaw uses the MAX+ rated shaft speed through the 1:1 turret gearing; smaller stick inputs intentionally request proportionally slower movement. Hood button steps select a destination; they do not impose a servo speed limit. Mechanical angle limits remain enforced.

| Mechanism | Servo | Rated shaft speed at 6.0 V | Mechanism travel estimate |
|---|---|---|---|
| Turret rotation, ports 1 and 2 | Axon MAX+ | 0.115 s/60 degrees (521.74 deg/s) | 521.74 deg/s at 1:1 |
| Hood, port 0 | Axon MAX+ | 0.115 s/60 degrees | 260.87 deg/s through 2:1 linkage; 0.0575 s for 15 degrees of hood movement |
| Depollinator, port 4 | Axon MAX+ | 0.115 s/60 degrees | 0.25875 s for 135 degrees |
| Gate, port 3 | goBILDA 2000-0025-0004 Super Speed | 0.043 s/60 degrees (1395.35 deg/s) | 0.0645 s for 90 degrees |

Sources: [Axon MAX+ specifications](https://docs.axon-robotics.com/archive/max), [Axon programmer settings](https://docs.axon-robotics.com/archive/programmer), and [goBILDA Super Speed specifications](https://www.gobilda.com/2000-series-dual-mode-servo-25-4-super-speed/). These ratings assume the specified supply voltage; load, acceleration, and internal settings affect actual motion. Two rotation servos provide additional torque, not twice the speed. PWM pulse width selects position, not motor power. The goBILDA gate specifies a 50 Hz (20 ms) PWM frame, which matches the SDK default; increasing that frequency is not a documented speed upgrade.

The Axon programmer settings must be written to each MAX+ separately; this repository cannot read or change them through the FTC `Servo` API. For maximum available output on the robot's 6.0 V servo supply, set **PWM Power to 100%**. Axon permits 100% at 6.0 V, while recommending 85% as its standard setting; at 100%, monitor temperature because thermal protection may shut the servo down. If startup acceleration must also be unrestricted, disable **Soft Start** in the programmer. Keep the existing travel, neutral, and direction settings so home positions are preserved (the depollinator requires increasing PWM = clockwise viewed from the output shaft). Servo power comes from the confirmed injector; the Expansion Hub supplies the position commands.

Keep Axon's factory **Dampening Factor** and overload protection. The manufacturer advises against changing its internal D coefficient. FTC positional servos expose position commands, not `DcMotorEx` velocity/PIDF setters; adding motor PIDF calls cannot unlock servo speed. The turret already uses its external Through-Bore encoder for bounded PID position correction. Those gains require loaded robot measurements to tune; they are not a guaranteed speed boost. The other mechanisms have no external measured position feedback, so an added software PID loop would have no measured error to regulate. See the [FTC Servo API](https://javadoc.io/static/org.firstinspires.ftc/RobotCore/10.3.0/com/qualcomm/robotcore/hardware/Servo.html).

Gate feeding still waits 0.25 s for loaded settling, and turret homing still waits for encoder stability. These waits do not slow physical movement. Rated travel and hood position estimates do not prove arrival; replace timing allowances with measured loaded values before automatic shooting calibration is enabled.

## Build environment and baseline

The existing Gradle configuration uses Gradle **9.1.0**, Android Gradle Plugin **8.13.2**, FTC SDK **12.0.0**, Android compile SDK **30**, minimum SDK **24**, target SDK **28**, and Java **8** source/bytecode compatibility. The baseline and final build were run using Android Studio's bundled **Java 21.0.8** runtime. Configure `JAVA_HOME` to a compatible installed JDK (Java 21 is verified), and configure the Android SDK using `ANDROID_HOME` or the ignored `local.properties`:

```properties
sdk.dir=C:/your/Android/Sdk
```

From the repository root, run:

```powershell
.\gradlew.bat :TeamCode:testDebugUnitTest :TeamCode:assembleDebug
```

The hardware-review baseline passed **81 tests**. The October 23 implementation passed **118 tests** with zero failures/errors/skips and produced the debug APK. Android lint reports **0 errors and 14 existing warnings** (12 native-library alignment warnings and two unused resources). See [implementation status](HardwareReviewImplementationStatus.md). The new build retains the SDK's Java 8 compilation warnings and Gradle deprecation notices. Machine-specific SDK/JDK paths are not committed. APK output: `TeamCode/build/outputs/apk/debug/TeamCode-debug.apk`.

## Coordinates, samples, and vision

Pinpoint is connected to **Control Hub I2C port 0**, configured as `pinpoint`. The SDK hardware map selects the bus from the active Robot Configuration; no Java bus reassignment is needed.

Shot planning uses **meters, seconds, radians**. The team's field convention is center origin, +X toward blue, +Y toward the scoring table, +yaw counterclockwise. This is a team convention: tag surveys, routes, and starting poses must all use it. Robot axes are +X forward, +Y left. Drivetrain command lateral/yaw are right/clockwise positive and are converted at the navigation boundary. Turret encoder zero is servo home at position 0.5, pointing robot-right. Driver-facing turret headings increase clockwise: front 0, right/home 90, back 180, left 270 degrees; the left 260..280 degree zone is excluded. Internal encoder/solver yaw remains counterclockwise from home (right = -90 degrees in robot math). Camera angles are tx right-positive, ty up-positive. Runtime metric geometry uses each tag's independent `getTargetPoseCameraSpace()` pose: optical +X right, +Y down, +Z forward, rotated through the known camera pitch. Configured high-tag height is a classifier comparison, not an input used to infer range.

`PinpointOdometry` performs one bulk update and publishes a timestamped `MotionSample` with pose, fixed-frame X/Y velocity, angular velocity, linear/angular acceleration estimates, uncertainty, status, and field-reference validity. X/Y velocity uses the same fixed axes as Pinpoint position; it is not rotated by robot heading again. The [manufacturer's output and setPosition documentation](https://www.gobilda.com/content/user_manuals/3110-0002-0001%20User%20Guide.pdf) describes position/velocity outputs and establishing a field reference. Installed displacement/velocity signs and turning behavior remain physical verification dependencies. Constructor zero and diagnostic zero deliberately do not establish a field reference. Enter an explicit TeleOp starting pose in `ShootingConfiguration`, or use an approved autonomous strategy's starting pose, and let INIT calibration finish before START.

`PoseHistory` stores one second of pose and turret yaw, interpolating across heading wrap. Vision corrections modify a translation transform rather than resetting hardware pose or differentiating position jumps. Navigation and shooting use that same corrected estimate in competition autonomous. Current pose/velocity older than 100 ms is rejected. Observations older than 350 ms, missing history, duplicate/out-of-order frames, mismatched target maps, and position jumps over 0.5 m are rejected. A known opening can be acquired using odometry while a missing observation still inhibits firing. The pose history retains a 750-ms target limit, but the independent observed-state gate removes automatic firing permission immediately on a missing/conflicting frame, or after 150 ms without a fresh capture. Reacquisition must qualify again. These are configurable software limits, not measured accuracy guarantees.

SDK 12 bytecode was inspected locally: `LLResult.getControlHubTimeStamp()` uses `System.currentTimeMillis()` at parsing start; `getControlHubTimeStampNanos()` merely scales that wall-clock value. The result's Limelight-local timestamp identifies duplicate frames. `PoseHistory.captureTime` bridges wall-clock receipt age to `System.nanoTime()`, subtracting capture and pipeline latency once. JSON parse time is already included in receipt age. See the [SDK latency/timestamp API](https://javadoc.io/static/org.firstinspires.ftc/Hardware/10.1.0/com/qualcomm/hardware/limelightvision/LLResult.html) for the meanings of these latency fields. Enter any measured additional transport delay in `ADDITIONAL_VISION_TRANSPORT_SECONDS`; do not add staleness or parse latency again. `VISION_TIMING_MEASURED` remains false until that timing model is checked on the installed system.

The former MegaTag camera/robot-pose conflation and competing direct-centering loops were removed. Field relocalization uses metric per-tag observations and the selected target's surveyed, state-specific tag map plus the rotating camera mount. With a verified local tag-to-opening transform, target-relative aiming uses the local odometry frame without requiring an absolute map. It never assumes a Limelight `botpose` is a camera position.

## Entering shooting calibration

The team confirms the raised/high goal's **opening center at 1.50 m** above the floor, its **AprilTag center at 1.25 m**, and the **opening plane at 60 degrees from horizontal**, with the same dimensions on both alliance sides. These are recorded in `ShootingConfiguration`; target height now uses 1.50 m. Only raised/high goals are shooting targets; lowered goals remain ineligible. Individual tag X/Y positions and the tag-to-opening transform remain necessary for field localization; equal heights do not imply equal field coordinates. The opening-plane angle is calibration geometry, not a commanded hood or launch angle. The current empirical planar solver does not model the opening plane or infer a trajectory from its angle.

The supplied Section 9 TU06 PDF confirms the tag-ID groups, 36h11 family, 3.25-in. individual tags and cluster-center offsets of +/-6.5 and +/-2.75 in. See [FieldReference.md](FieldReference.md) for ordering, dimension references and derived local offsets. The remaining field-map dependency is the raised cell's pivot-to-opening/cluster transform, not sixteen unrelated tag measurements. Visibility does not identify raised state because the clusters move with the hive. The existing 17.73-in. opening Y value remains an unapproved example.

Edit `ShootingConfiguration` and the mechanism constants in `TurretSubsystem`/`TurretControl`:

1. The current planar model assumes an upright camera with negligible roll. Supply `ShooterMount` with separate robot-to-pivot, pivot-to-camera, and pivot-to-muzzle offsets, camera height/pitch/yaw, and turret zero direction. Mark `measured` only after measuring them; a measured zero offset is valid.
2. Replace `target(alliance, side)` with `GoalTarget` data for each selected opening. Supply actual opening X/Y/height, tag identities and each tag's surveyed X/Y/height, explicit `HiveState`, and a geometry ID. Configured raised geometry is only a map; automatic feeding additionally requires a fresh, stable observed RAISED state and validated detector/pipeline calibration. Alternatively supply verified `OPENING` local geometry instead of an absolute field map. Unknown/lowered state or mere tag visibility cannot enable firing. Selecting a side never changes because another tag is larger in an image.
3. Replace `ShotProfile.example()` with measured, ordered distance/hood/RPM/flight-time rows. The geometry ID must match the selected opening. Hood is an empirical mechanism command; RPM is not ball exit speed. Rows must have finite distance, hood travel 0–30 degrees, RPM 0–6000 (positive), strictly increasing range, and positive flight time up to 3 seconds. NaN flight time denotes a stationary-only profile. Interpolation is linear; endpoints never extend coverage. The legacy example rows remain uncalibrated and cannot enable automatic shots.
4. Measure gate travel, feed-to-exit delay, feed window and recovery; enter them in `firing()` and mark `RELEASE_TIMING_MEASURED`. Tune the hood travel estimate and homing travel/stability times. The hood and gate have no physical position feedback.
5. For field-localized mode set an explicit starting field pose; target-relative mode preserves local provenance. Validate `ENVELOPE`: maximum translation speed, turn rate, linear/angular acceleration, position uncertainty, and turret slew rate. Current limits are conservative software settings awaiting robot validation.

Homing states are unhomed, moving home, settling, ready, and fault. INIT commands home with trim disabled, waits for travel, then requires an encoder stability window and captures zero once. START does not bypass it. Timeout inhibits automatic firing. Rehoming invalidates the current shot; diagnostic zero and servo-center override require another home cycle. **A stationary stall cannot be distinguished from home using the relative encoder. READY means estimated home, not absolute verification.** The turret never resets or changes direction/mode of the intake's shared encoder channel.

The solver predicts the release pose using remaining gate/feed travel and measured constant field velocity. It includes tangential velocity at pivot/muzzle offsets, compensates lateral and radial motion, and iterates profile range/flight time at most 24 times. It rejects unsupported range, missing moving-flight-time data, nonconvergence, unsafe yaw/slew, stale samples, and motion outside the envelope. Desired yaw/rate describe release; the present yaw command accounts for angular travel before release. At zero velocity the same profile supplies the stationary shot. It does not model arbitrary acceleration or infer a projectile elevation from hood setting.

## Driver controls and readiness

Existing red/blue TeleOp selection, chassis, intake, depollinator, manual hood/RPM, feed inhibit, and flywheel-stop controls remain available.

| Input | Behavior |
| --- | --- |
| GP1 RT | Hold compensated aim and automatic feed when calibrated, localized, homed and stably tracking. Invalid data reports a reason and inhibits automatic feed. |
| GP1 LT or GP2 RT | Explicit manual gate/feed override; wait for gate travel, then feed continuously while held. This bypasses shot readiness, including flywheel-stop readiness. |
| GP2 LT | Highest priority: close gate and stop both transfer stages. |
| GP2 B | Hold flywheel stop; inhibits automatic feeding. Release restores manual RPM if no valid automatic profile is available. |
| GP2 X | Begin non-blocking rehome; invalidates the current automatic shot. |
| GP2 Dpad left/right | Select audience/far opening; invalidates observation/pose history. State/calibration still comes from configuration. |
| GP2 RS-X | Manual yaw at the Axon MAX+ 6.0 V rating (about 521.74 degrees/second at full stick, 1:1 gearing) when auto aim is released. |
| GP2 Dpad up/down, Y/A | Existing hood/RPM steps outside automatic aim. |
| Either driver's bumpers | Intake forward/reverse; simultaneous directions stop. Bumpers override full-queue reversal. |

Automatic readiness requires valid homing/target/pose/profile/timing/motion, compensated yaw and rate tracking, estimated hood tracking, and flywheel speed over a stable window. It never requires tx=0: a moving shot leads the opening. Entry/retention tolerances and timed recovery avoid gate chatter. Invalid solutions stop further feeding immediately. A ball already committed may still exit; telemetry reports that possibility and **timed feed attempts**, never invented ball counts. Leaving manual feed requires automatic readiness to qualify again. Full confirmation still requires two continuous HIGH seconds, LOW resets it, shot feed bypasses full, and intake reversal remains latched until GP1 RT release.

## Pollen camera and collection

`PollenVision.CONFIG` uses validated `PollenConfiguration` data: camera mount, explicit intrinsics or SDK calibration/fallback FOV, HSV thresholds, pickup start/travel, and separate measured flags for mount, intrinsics, color thresholds, and pickup. The team-confirmed C270 lens is **195 mm above the floor, 248 mm forward of robot center, and on centerline**, looking straight forward with its optical axis **10 degrees below horizontal**. These values are configured and mount status is measured. Color and pickup calibration remain incomplete; explicit intrinsics are not supplied. Approximate intrinsics can only drive autonomous if explicitly allowed, and color/pickup status must still be measured. With the current approximate 60-degree horizontal FOV at 320×240, the nearest unclipped contact is about 550 mm from robot center; the 350-mm pickup-start example is inside that blind spot and must be replaced during later pickup calibration.

Use **Test: Pollen Camera Calibration** with only `webcam_1` configured. Record blob-center/contact pixels, estimated range, measured floor distance, active intrinsics/model and rejection reason. It initializes no drivetrain, intake, turret, or Pinpoint. Clipped contours, invalid floor rays, poor blob geometry, out-of-range and discontinuous targets are rejected. Frames retain capture timestamps and age checks. `PollenNavigator` confirms close targets over distinct frames before a bounded, timed/distance-limited pickup through the blind spot. `PollenAuto` remains the focused collection routine and now respects autonomous camera eligibility.

## Competition autonomous

**Deferred by the team.** The current task requires pollen detection and navigation, not match paths. The following describes existing scaffolding only; null routes are intentional and are not an outstanding requirement for this scope.

Configure `AutonomousStrategy.RED_ROUTE` and `BLUE_ROUTE` using validated data: alliance/target, starting pose, collection/scoring/parking waypoints, robot clearance radius, keep-out boxes, route timeout, collection/score windows, optional preload/repeats, match deadline and parking reserve. Mark routes approved only after checking coordinates and clearance. **No default competition coordinates are invented**; red/blue match OpModes report the missing approved route and initialize no motion hardware in that case.

`CompetitionAutonomousController` transitions through preload, travel to collection, bounded collection, travel to scoring, scoring, optional repeats, and park/stop. Collection full, partial/empty, timeout and unavailable camera all return control to the strategy. Pose loss stops; navigation stall or timeout attempts the configured park fallback, whose failure stops. Current and planned segments must pass field/clearance checks. Collection checks a bounded forward clearance horizon; no general obstacle avoidance is provided. A global deadline stops all commands, with parking requested earlier by the configured reserve. Every state is interruptible by STOP, and the hardware adapter closes the gate and stops motors before camera shutdown.

Only scoring-route segments explicitly marked `movingShots` request moving fire. Other scoring waits for near-zero chassis velocity. A failed shot solution never changes the driving route. Scoring uses bounded feed/recovery windows without assuming exact ammunition count.

## Verification scope

JUnit tests cover homing travel/stability/timeout/rehome, loop-rate independence, profiles and geometry, delayed/duplicate vision and rotating mounts, heading wrap, correction without velocity spikes, target loss/reacquisition, synthetic moving projectile interception with separate trajectory integration, gate/feed timing, override priority, full-queue behavior, camera eligibility/clipping/continuity, waypoint signs/clearance/stall, autonomous collection outcomes, failed readiness, STOP and deadlines.

Keep using isolated mechanism tests before combined operation. Compensated aiming diagnostics in competition TeleOp require drivetrain/Pinpoint, turret/Limelight, gate and transfer hardware. Physical measurements and moving-shot accuracy checks remain the separate dependencies listed in `SoftwareImplementationPlan.md`.

### Turret manual hold and external encoder PID

Auto-aim uses the same shared `TurretControl.MAX_YAW_DEG_PER_SECOND` (521.74 deg/s at 6 V with 1:1 gearing) as manual control for its slew and tracking feasibility checks. Positional commands go directly to the target without an additional software speed cap; feeding still waits for measured yaw/rate readiness.

GP2 right-stick X sets proportional clockwise yaw speed, up to the Axon MAX+ 6 V rating of 60/0.115 = 521.74 deg/s at full stick with 1:1 gearing. Releasing the stick captures the measured encoder angle as the hold target, discarding queued manual travel. GP1 RT overrides GP2 yaw and rehome; releasing RT hands control back at the measured angle. Both rotation servos always receive the same position command.

The positional servo mapping supplies feedforward; external encoder PID adds bounded correction with measured-velocity damping and anti-windup at correction/travel limits. Gains in `TurretControl` are initial values requiring loaded-robot tuning; the speed rating is not a measured loaded speed. Without shot calibration, fresh selected-hive tags can still be tracked in yaw, once per camera frame; automatic firing remains inhibited. This fallback centers the selected tag, not a calibrated opening offset.

### Turret and Limelight mounting

The team's corrected mounting description has the Limelight facing the same direction as the muzzle (relative yaw 0 degrees), pitched 30 degrees above horizontal, with its lens 335 mm above the floor. The turret pivot is centered left/right and 50.68184865 mm forward of robot center. With the muzzle pointing robot-forward, the camera is centered left/right and 99.53073974 mm behind the turret pivot. Its displacement rotates with the turret. The camera is therefore 48.84889109 mm behind robot center only when the muzzle points robot-forward. These confirmed values are recorded in ShootingConfiguration.MOUNT with measured=true. Target geometry, shot profiles and timing retain their separate calibration requirements.

The ball exit is centered on the turret rotation axis in plan view. Hood movement does not move the camera or significantly move the exit position; it changes the exit angle. Home (servo 0.5) is the highest exit elevation: +15 degrees above horizontal. Counterclockwise servo travel lowers elevation to -15 degrees; clockwise travel raises it toward +15 degrees. The internal hood command and exported hood travel remain downward travel from home, now 0–30 degrees. Launch elevation = 15 - hood command. With the existing 2:1 linkage and 355-degree programmed servo range, the endpoints are 0.5 and approximately 0.330986 servo position (60 degrees CCW shaft travel). Dpad Up raises elevation, Down lowers it; calibration telemetry shows both hood travel and launch elevation. This is commanded geometry, not a measurement of the actual ball trajectory. ShotProfile remains an empirical distance-to-hood/RPM table; the planar solver does not require muzzle height or assume a ballistic model. Exit height would be needed for a separate ballistic model. Revalidate shot tables against this corrected geometry.

Fallback aiming converts camera bearings to shooter bearings and wraps yaw while retaining the current travel-limit side inside the excluded left sector. Existing observation freshness requirements still apply. Manual shot calibration does not use the camera.



## October 23 hardware-review implementation

See [HardwareReviewImplementationStatus.md](HardwareReviewImplementationStatus.md) for the implementation/verification matrix and remaining measurements. `RobotLoop` owns MANUAL Lynx bulk caching and exactly one clear per INIT/active loop in all team OpModes. The SDK resets bulk mode to OFF between OpModes. Java loop period/work telemetry is separate from Pinpoint device-cycle telemetry. Turret, queue and motor samples are cached; intake/transfer current reads are throttled to 5 Hz. Shutdown attempts every remaining stop/close even if one device throws.

`StationaryMotion` shares a filtered, 150-ms qualification at <=0.03 m/s, <=0.03 rad/s chassis and <=3 deg/s turret motion. Any raw excursion revokes qualification. Qualified bounded velocity/acceleration noise is treated as stationary; other motion still requires measured flight times. Accepted vision uncertainty includes measurement quality, a 25-mm floor, the uncorrected fusion residual and subsequent drift growth. Reference generations invalidate history and qualification on pose resets.

`GOAL_OBSERVATION_VALIDATED=false` keeps unvalidated camera settings and classifier tolerances out of automatic feeding. The level-chassis assumption is explicit because no chassis pitch/roll feedback is supplied. `LOWERED_TAG_HEIGHT_METERS` and `LOWERED_TAG_PLANE_DEG` remain NaN: unsupported low poses report TRANSITIONING, not a fabricated calibrated LOWERED result. Opening-plane 60 degrees remains distinct from the underside tag plane's raised expectation of 30 degrees.

Calibration exports: in `Calibrate: Hive Shot Table`, BACK opens REVIEW; right-stick click saves all hit/miss samples and averaged rows in the app's external-files `shot-calibration/shot-session-*.properties`. Telemetry reports the full saved path or failure; the notebook stays intact. Files include version, geometry, mounting identity, units and SESSION_UNVALIDATED_STATIONARY_ONLY status. The parser validates round trips; exports never automatically replace `ShootingConfiguration.PROFILE`.
