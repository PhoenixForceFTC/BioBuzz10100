package org.firstinspires.ftc.teamcode;

/** Team-approved nominal field targets and the shared blue-loading-corner TeleOp start. */
public final class TeleOpFieldReference {
    private TeleOpFieldReference() { }
    public static final double FIELD_HALF_WIDTH_METERS = FieldGeometry.FIELD_HALF_WIDTH_METERS;
    public static final double CENTER_FROM_REAR_METERS = .262481;
    public static final double CENTER_FROM_SIDE_METERS = .15835;
    public static final double START_X_METERS = FIELD_HALF_WIDTH_METERS - CENTER_FROM_REAR_METERS;
    public static final double START_Y_METERS = -FIELD_HALF_WIDTH_METERS + CENTER_FROM_SIDE_METERS;
    public static final double START_HEADING_DEG = 180;
    public static final double OPENING_Y_OFFSET_METERS = FieldGeometry.OPENING_Y_OFFSET_METERS;

    public static TurretSubsystem.HiveSide nearestSide(double fieldY, TurretSubsystem.HiveSide previous) {
        return fieldY < 0 ? TurretSubsystem.HiveSide.AUDIENCE
                : fieldY > 0 ? TurretSubsystem.HiveSide.FAR : previous;
    }
    public static GoalTarget target(TurretSubsystem.Alliance alliance, TurretSubsystem.HiveSide side) {
        // RAISED describes the aim geometry only. Drivers decide when this cell is shootable.
        // Nominal manual-derived tag map; camera fusion still requires explicit validation.
        return new GoalTarget(alliance, side, GoalTarget.HiveState.RAISED,
                TurretSubsystem.getHiveOpeningCenterXInches(alliance) * .0254,
                (side == TurretSubsystem.HiveSide.AUDIENCE ? -1 : 1) * OPENING_Y_OFFSET_METERS,
                ShootingConfiguration.HIGH_GOAL_CENTER_HEIGHT_METERS, true,
                ShootingConfiguration.CALIBRATION_GEOMETRY_ID, HiveTagGeometry.nominalRaisedTags(alliance, side));
    }
    /** Bearing from the turret pivot; independent of shot-profile calibration and camera visibility. */
    public static double yawRadians(MotionSample pose, GoalTarget target, ShooterMount mount) {
        RobotMath.Vec pivot = new RobotMath.Vec(pose.x, pose.y).plus(mount.pivot.rotate(pose.heading));
        return RobotMath.wrap(Math.atan2(target.y - pivot.y, target.x - pivot.x) - pose.heading - mount.zeroYaw);
    }
}
