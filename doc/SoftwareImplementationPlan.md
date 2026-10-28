# Robot software completion and shooting while moving

> Historical review/plan: October 26 decisions supersede the moving-shot solver, flight-time data, motion-based firing gates, hive-state gating in competition TeleOp and calibration-file saving described below. These features have been removed from the competition workflow. See [current competition behavior](CompetitionTeleOp.md), [configuration](SoftwareConfiguration.md) and [tag map](FieldReference.md) for the implemented behavior.


Scope: address the six review findings and add motion-compensated shooting. This is an implementation plan, not a claim that the robot is calibrated or competition-ready. Physical checks and measurements are listed separately at the end. Existing local changes are the baseline and must be preserved.

Scope clarification, October 22, 2026: the team requests pollen detection and navigation only for autonomous work. Match paths, preload/repeat/scoring/parking strategies and their route data are deferred. Their appearance below records earlier planning, not a current requirement.

## 1. Establish a reproducible software baseline

- Locate/configure the Android SDK through the ignored `local.properties` file or `ANDROID_HOME`; document the required SDK and Java versions from the existing Gradle configuration. Do not commit machine-specific paths.
- Run `gradlew.bat :TeamCode:assembleDebug :TeamCode:testDebugUnitTest` and record the baseline. The review attempt stopped at missing SDK configuration before compilation.
- Keep hardware ownership in subsystems and match coordination in OpModes. Extract only hardware-independent homing, targeting, shot-solving, and firing logic that needs deterministic tests. Inject time into those controllers so tests do not sleep.

Acceptance: build and existing tests pass, or any independent baseline failure is explicitly identified before feature work.

## 2. Fix turret homing and control timing

- Replace immediate home-and-zero with a non-blocking state machine: unhomed, moving home, settling, ready, and fault. Command the home servo position with encoder trim disabled, allow a configured travel interval, require a stable encoder window, then capture the encoder zero once.
- Run homing updates during INIT in competition and turret-test OpModes. If START happens early, continue homing without allowing automatic firing. Rehoming invalidates the current shot solution.
- Add a timeout and explicit fault reason. With the current relative encoder and no independent home sensor, settling is an estimate: a stationary stalled turret cannot be distinguished conclusively from a turret at home. Do not describe this as absolute homing verification.
- Change manual yaw from degrees per loop to degrees per second. Make encoder trim time-based, bounded, and updated once per loop; command setters must not integrate correction a second time. Reset correction during homing and mode transitions.
- Cache encoder angle/velocity once per loop and ensure the shared intake motor/encoder channel retains its intended direction and mode.

Acceptance: tests cover starting away from zero, delayed movement, stability windows, timeout, early START, rehome, and equivalent behavior at different loop rates.

## 3. Correct goal geometry and make calibration data explicit

- Introduce a target model containing alliance, selected hive side/state, tag identities and geometry, and the actual opening position. Keep target selection stable across observations instead of switching to whichever tag has the largest area each frame. Tag visibility alone must not imply that a cell is in the scoring configuration.
- Convert camera observations to the opening using tag-to-opening transforms, camera-to-turret mounting, turret-to-robot mounting, and current turret yaw. Use each tag's own height for any angle-based range calculation; do not combine tag elevation with opening height.
- Define camera, robot, turret, and field coordinate conventions explicitly. Use one internal unit system for planning, with conversions at SDK and telemetry boundaries.
- Replace the implicit placeholder table with validated shot profiles containing distance, hood command, flywheel RPM, flight time, supported target geometry, and calibration status. Validate finite values, ordering, limits, and coverage. Flight time may be absent for a stationary-only profile, but must be valid for moving shots.
- Reject automatic shots outside the supported range instead of silently using an endpoint. Keep the existing example rows clearly marked as uncalibrated; software changes cannot manufacture measured replacements.
- Preserve manual controls and provide explicit reasons when an automatic profile is unavailable.

Acceptance: tests cover multiple tags referring to one opening, red/blue selection, both hive sides/states, frame transforms, invalid profiles, interpolation, and range rejection.

## 4. Connect odometry and vision through a consistent pose estimate

- Expose a timestamped cached Pinpoint sample containing pose, linear velocity, angular velocity, status, and freshness. Confirm the SDK velocity frame before converting to the planner's field frame. Continue one Pinpoint update per loop.
- Establish an explicit starting field pose; the constructor's local zero must not silently mean field center. Track whether the field reference is valid.
- Keep a short pose and turret-angle history so vision can be interpreted at image capture time. Verify the SDK timestamp/latency contract and avoid counting the same delay twice.
- Correct the existing relocalization path before wiring it in: distinguish camera pose from robot pose, account for the rotating camera mount, and only use a field map compatible with the target's state. Reject stale data, implausible jumps, incompatible frames, and repeated observations of the same frame.
- Apply accepted vision corrections without producing artificial velocity spikes. Maintain odometry goal tracking through brief occlusions, with explicit age/uncertainty limits that eventually inhibit firing.
- Make one coordinator own target and pose selection. Replace the currently unused/disconnected tracking paths rather than adding another competing aiming loop.

Acceptance: delayed and duplicate vision, turret rotation, heading wrap, pose corrections, invalid initialization, and target loss/reacquisition all have deterministic tests.

## 5. Implement the moving-shot solver

- Add a hardware-independent `ShotPlanner` producing desired turret yaw and yaw rate, hood command, RPM, predicted release time/position, flight time, and validity/rejection reason.
- Predict robot and shooter position at ball release using remaining gate travel plus feed-to-exit delay. Use a different remaining delay when the gate is already open or feeding is underway.
- Compute the shooter's inherited field velocity from chassis translation and rotation about the robot center. Include turret rotation at the launch point if the muzzle is offset from the turret axis. Keep camera, pivot, and muzzle offsets distinct.
- Solve for the launch direction and effective range after subtracting inherited velocity over ball flight time. Iterate with the calibrated stationary shot profile because effective range, hood/RPM, and flight time depend on one another. Bound iteration count and reject non-convergent solutions.
- Compensate both sideways motion and motion toward/away from the goal. Convert the field launch direction into turret yaw at the predicted robot heading at release. Account for angular tracking rate and turret travel limits.
- Treat hood command as an empirical mechanism setting, not automatically as the projectile's elevation angle. Flywheel RPM alone is not a measured ball exit speed.
- Use a declared motion envelope: reject excessive speed, turn rate, acceleration, uncertainty, or required turret slew. Begin with a short-horizon constant-velocity prediction and reject motion that violates it; do not promise accurate shots during arbitrary acceleration.
- At zero robot velocity, use the same stationary solution. Return an invalid result for stale velocity, missing flight-time data, or unsupported range.

Acceptance: simulated straight, lateral, diagonal, and rotating motion; correct lead signs; muzzle offsets; zero-speed equivalence; changing feed delay; angular wrap; unreachable yaw; and bounded solver execution. Verify projectile interception against an independent synthetic trajectory model, not just the solver's own helper functions.

## 6. Unify readiness, feeding, and TeleOp integration

- Add a non-blocking firing coordinator shared by TeleOp and autonomous: idle, preparing, gate opening, feeding, recovery, and inhibited/fault.
- Replace raw tag-centering readiness with tracking error relative to the compensated shot solution. A moving shot intentionally leads the visible target, so `tx == 0` must not remain a requirement.
- Require homing readiness, valid pose/target/profile, supported motion, turret position/rate tracking, hood tracking, and flywheel speed over a stable time window. Use hysteresis to avoid gate chatter.
- Adapt hood readiness for a changing command: estimate servo travel/lag rather than requiring an unchanged position for every moving shot. Retain the lack of physical position feedback in status reporting.
- Recompute during preparation and feeding. Stop further feeding on invalid solutions and model the fact that a ball already committed to the shooter cannot be recalled instantly.
- Preserve current manual feed, feed inhibit, flywheel stop, intake reversal, and alliance controls. GP1 auto-aim/fire uses compensation when moving within the supported envelope. Manual override remains explicit.
- Add telemetry for rejection reasons, target identity, observation age, field velocity, lead angle, predicted release, range, and shot profile status. Keep one telemetry flush at the end of the loop.
- Add controller tests and simulated integration tests; retain isolated subsystem test OpModes. Any combined aiming diagnostic must be clearly labeled as requiring multiple subsystems.

Acceptance: no automatic feed on stale/invalid data, deterministic override priority, no blocking waits, recovery after RPM drops, and no regression in intake/full-queue behavior. The current full sensor cannot count launched balls; report timed feed attempts rather than invented shot counts.

## 7. Finish pollen vision's software configuration

- Move camera mounting, intrinsics provenance, color thresholds, and pickup parameters into validated configuration. Keep measured/unknown status separate from numeric values; zero can be a real offset.
- Preserve frame timestamps and rejection reasons. Treat approximate intrinsics as a diagnostic mode, with autonomous eligibility explicitly configured instead of silently assuming calibrated range.
- Add a calibration-support mode that reports pixel/contact coordinates, estimated range, and the active model for later measurements. Keep this camera-only test independent of other hardware.
- Test invalid calibration, clipping, stale frames, target continuity, offset transforms, and the transition into the camera blind spot.

Acceptance: unsupported configuration is visible and cannot silently drive autonomous using supposedly measured distances. No physical constants are invented to remove warnings.

## 8. Build a complete autonomous coordinator

- Retain `PollenAuto` as the focused collection routine. Add a competition autonomous coordinator that reuses collection, waypoint navigation, and the shared firing coordinator.
- Implement configurable states for optional preload scoring, travel to collection, bounded collection, travel to scoring, scoring, optional repeat, and park/stop. Use a global match deadline and per-state timeout with guaranteed motor cleanup.
- Add non-blocking pose-based waypoint following with heading control, arrival tolerances, and stall/progress detection. Collection timeouts return control to the strategy so it can score or park instead of always ending the match routine.
- Represent red/blue routes, starting poses, scoring targets, robot-clearance constraints, and time budgets as validated strategy data. Test synthetic routes; do not label invented competition coordinates as approved field routes. General obstacle avoidance is outside this first implementation.
- Permit moving shots only on designated scoring route segments within the solver's envelope. Otherwise use stationary scoring. Never let a failed moving-shot solution force an unplanned drive maneuver.
- Use bounded feed windows and recovery timing without claiming an exact ball count. Select a bounded fallback when localization, collection, or shooting becomes unavailable.

Acceptance: simulated full autonomous runs cover full/partial/empty collection, no target, failed shot readiness, navigation stall, invalid configuration, STOP in every state, and deadline-driven parking/stop.

## 9. Verify and document the completed software

- Run meaningful unit and integration tests for the new controllers, geometry, latency handling, profiles, moving-shot trajectories, override priorities, and autonomous transitions.
- Run `gradlew.bat :TeamCode:testDebugUnitTest :TeamCode:assembleDebug` from the repository root.
- Update controls, configuration documentation, subsystem test instructions, and rejection-reason explanations. Distinguish software tests from robot validation in the completion report.
- Software completion means implemented paths, deterministic tests, successful packaging, and explicit handling of missing calibration. It does not mean physical shot accuracy has been established.

Implementation order: baseline → homing → geometry/profiles → pose/vision → shot solver → firing/TeleOp → pollen configuration → autonomous → final verification. Add tests with each phase rather than postponing them to the end.

## Separate physical checks and measurements — recorded, not part of software work

- [x] Turret physical checks: assembly/gear alignment, home alignment, servo directions/travel, encoder sign, mechanical limits, hood/flywheel operation and homing/settling behavior. Completed October 24, 2026 per operator confirmation after running `Test: Turret`; see [physical sign-off](HardwareReviewImplementationStatus.md#turret-physical-checks--complete-october-24-2026). Numerical timing calibration remains separate; absolute home confirmation would require independent feedback beyond the current relative encoder.
- Actual camera-to-pivot, pivot-to-robot, and muzzle offsets; turret zero direction; tag-to-opening geometry and hive scoring states.
- Measured stationary distance/hood/RPM profiles, flight times, launch consistency, gate travel, feed-to-exit delay, and flywheel recovery.
- Camera intrinsics, mounting angle/height, lens distortion, pollen colors under field lighting, and pickup geometry.
- Pinpoint displacement/velocity signs, frame conventions on the installed configuration, and turning behavior.
- Full-sensor threshold, transfer timing, and any future individual-shot sensing. The existing full sensor does not establish exact ammunition count.
- Robot clearance and approved autonomous route coordinates.
- Final moving-shot accuracy and the usable speed/turn/acceleration envelope.

These are configuration and validation dependencies. The software should expose them clearly and support later entry of measured values; physical checks are not implementation tasks in this plan.

## Software implementation record

Implemented paths, configuration instructions, verification scope, and remaining physical dependencies are documented in [SoftwareConfiguration.md](SoftwareConfiguration.md). The baseline build and 28 existing tests passed after configuring the ignored local Android SDK path. Automatic calibration and competition route defaults remain explicitly unavailable; no measured shot rows or approved coordinates were invented.
