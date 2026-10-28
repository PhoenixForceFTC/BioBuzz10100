package org.firstinspires.ftc.teamcode;

/** Motor encoder conversion for the external 2:3 flywheel speed reduction. */
public final class ShooterSpeed {
    private ShooterSpeed() { }
    public static final double MOTOR_TICKS_PER_REV = BareMotorSpecifications.ENCODER_TICKS_PER_REV;
    public static final double MOTOR_MAX_RPM = BareMotorSpecifications.CONFIGURED_FREE_SPEED_RPM;
    public static final double FLYWHEEL_REVS_PER_MOTOR_REV = 2.0 / 3.0;
    public static final double FLYWHEEL_MAX_RPM = MOTOR_MAX_RPM * FLYWHEEL_REVS_PER_MOTOR_REV;
    // Operator-requested full target range; no software recovery reserve.
    public static final double FLYWHEEL_MAX_TARGET_RPM = FLYWHEEL_MAX_RPM;

    public static double clampFlywheelRpm(double rpm) {
        return Double.isFinite(rpm) ? Math.max(-FLYWHEEL_MAX_TARGET_RPM, Math.min(FLYWHEEL_MAX_TARGET_RPM, rpm)) : 0;
    }
    public static double motorTicksPerSecond(double flywheelRpm) {
        return clampFlywheelRpm(flywheelRpm) / FLYWHEEL_REVS_PER_MOTOR_REV / 60 * MOTOR_TICKS_PER_REV;
    }
    public static double flywheelRpm(double motorTicksPerSecond) {
        return motorTicksPerSecond * 60 / MOTOR_TICKS_PER_REV * FLYWHEEL_REVS_PER_MOTOR_REV;
    }
}
