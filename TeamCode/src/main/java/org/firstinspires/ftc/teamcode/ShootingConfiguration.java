package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.RobotMath.Vec;

/** Enter measured values here. Numeric examples remain explicitly ineligible for automatic shots. */
public final class ShootingConfiguration {
    private ShootingConfiguration() { }
    // Team-confirmed raised/high-goal geometry, identical dimensions on both alliance sides.
    public static final double HIGH_GOAL_CENTER_HEIGHT_METERS = 1.50;
    public static final double HIGH_GOAL_TAG_CENTER_HEIGHT_METERS = 1.25;
    // Opening plane relative to horizontal, not a launch/hood angle. Retained for shot calibration.
    public static final double HIGH_GOAL_OPENING_PLANE_DEG = 60;
    public static final double TURRET_PIVOT_FORWARD_MM = 50.68184865;
    public static final double TURRET_PIVOT_LEFT_MM = 0;
    public static final double CAMERA_FORWARD_FROM_PIVOT_MM = -99.53073974;
    public static final double CAMERA_LEFT_FROM_PIVOT_MM = 0;
    public static final double CAMERA_YAW_FROM_MUZZLE_DEG = 0;
    public static final double CAMERA_PITCH_UP_DEG = 30;
    // Lens height and camera/pivot planar geometry confirmed by the team.
    public static final double CAMERA_HEIGHT_MM = 335;
    // Camera/muzzle offsets use turret-local +X along the muzzle and +Y left.
    // Ball exit is centered on the pivot in plan view and does not move significantly with hood.
    public static final ShooterMount MOUNT = new ShooterMount(
            new Vec(TURRET_PIVOT_FORWARD_MM / 1000, TURRET_PIVOT_LEFT_MM / 1000),
            new Vec(CAMERA_FORWARD_FROM_PIVOT_MM / 1000, CAMERA_LEFT_FROM_PIVOT_MM / 1000),
            new Vec(0, 0), -Math.PI / 2, Math.toRadians(CAMERA_YAW_FROM_MUZZLE_DEG),
            Math.toRadians(CAMERA_PITCH_UP_DEG), CAMERA_HEIGHT_MM / 1000, true);
    // Cluster spacing is known; front/underside-to-aim-point references are not yet verified.
    // Fill tag-local +Y and +Z offsets AND verify axes/reference points before enabling.
    public static final OpeningGeometry OPENING = new OpeningGeometry("TU06-high-transform-unverified", false, 0, 0);
    public static final String CALIBRATION_GEOMETRY_ID = "TU06-high-1.50m-tag1.25m-plane60deg";
    // No measured low-state pose supplied: unsupported low observations remain TRANSITIONING.
    public static final double LOWERED_TAG_HEIGHT_METERS = Double.NaN, LOWERED_TAG_PLANE_DEG = Double.NaN;
    public static final boolean GOAL_OBSERVATION_VALIDATED = false; // Camera settings / pose quality / tolerances await validation.
    public static final boolean LEVEL_CHASSIS_ASSUMED = true; // No chassis pitch/roll sensor is supplied.
    public static final ShotProfile PROFILE = ShotProfile.example();
    // Chassis limits await validation; auto-aim uses the same rated 6 V turret speed as manual control.
    public static final ShotPlanner.Envelope ENVELOPE = new ShotPlanner.Envelope(.6, .6, 1.0, .15,
            Math.toRadians(TurretControl.MAX_YAW_DEG_PER_SECOND));
    public static final boolean RELEASE_TIMING_MEASURED = false;
    public static final boolean VISION_TIMING_MEASURED = false;
    public static final double ADDITIONAL_VISION_TRANSPORT_SECONDS = 0;
    public static final double GATE_TRAVEL_SECONDS = GateSubsystem.OPEN_SETTLE_TIME_SECONDS;
    public static final double FEED_TO_EXIT_SECONDS = .12;
    public static final double FEED_WINDOW_SECONDS = .18, RECOVERY_SECONDS = .35, READY_STABLE_SECONDS = .15;
    public static final boolean STARTING_FIELD_POSE_CONFIGURED = false;
    public static final double START_X_METERS = 0, START_Y_METERS = 0, START_HEADING_RAD = 0;
    public static GoalTarget target(TurretSubsystem.Alliance alliance, TurretSubsystem.HiveSide side) {
        // Heights/plane are known; opening/tag field X/Y and actual raised state remain unconfigured.
        // Use HIGH_GOAL_TAG_CENTER_HEIGHT_METERS for the high-goal tag when its X/Y is supplied.
        return GoalTarget.unknown(alliance, side);
    }
    public static FiringCoordinator firing() {
        // Gate travel is the existing estimate; feed timing examples are not measured calibration.
        return new FiringCoordinator(GATE_TRAVEL_SECONDS, FEED_TO_EXIT_SECONDS, FEED_WINDOW_SECONDS, RECOVERY_SECONDS, READY_STABLE_SECONDS);
    }
}
