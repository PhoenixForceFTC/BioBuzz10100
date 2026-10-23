package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.RobotMath.Vec;

/** Enter measured values here. Numeric examples remain explicitly ineligible for automatic shots. */
public final class ShootingConfiguration {
    private ShootingConfiguration() { }
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
        // Replace with surveyed, state-specific tag positions and actual opening geometry.
        return GoalTarget.unknown(alliance, side);
    }
    public static FiringCoordinator firing() {
        // Gate travel is the existing estimate; feed timing examples are not measured calibration.
        return new FiringCoordinator(GATE_TRAVEL_SECONDS, FEED_TO_EXIT_SECONDS, FEED_WINDOW_SECONDS, RECOVERY_SECONDS, READY_STABLE_SECONDS);
    }
}
