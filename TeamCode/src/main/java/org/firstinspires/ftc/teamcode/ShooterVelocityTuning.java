package org.firstinspires.ftc.teamcode;

/** Initial hub velocity PIDF gains and voltage-compensated feedforward; requires loaded tuning. */
public final class ShooterVelocityTuning {
    private ShooterVelocityTuning() { }
    // Stronger initial proportional recovery; validate overshoot during loaded trials.
    public static final double INITIAL_P = 40.0;
    public static final double INITIAL_I = 0.0;
    public static final double INITIAL_D = 0.0;
    public static final double MAX_I = 5.0;
    public static final double I_STEP = .05;
    public static final double MIN_F_CHANGE = .02;
    public static final double SAMPLE_INTERVAL_SECONDS = .2;
    // Lynx velocity feedforward output is scaled to +/-32767.
    public static final double FULL_SCALE_OUTPUT = 32767.0;
    public static final double BASE_F = FULL_SCALE_OUTPUT
            / (ShooterSpeed.MOTOR_MAX_RPM / 60 * ShooterSpeed.MOTOR_TICKS_PER_REV);

    /** Maximum recovery effort below target; coast above it, never actively brake.
     * No deadband delays recovery after a shot. Supports either commanded direction.
     */
    public static double bangBangPower(double targetRpm, double measuredRpm) {
        if (!Double.isFinite(targetRpm) || !Double.isFinite(measuredRpm) || targetRpm == 0) return 0;
        double direction = Math.signum(targetRpm);
        return measuredRpm * direction < Math.abs(targetRpm) ? direction : 0;
    }

    public static double feedforward(double batteryVolts, double scale) {
        double voltage = Double.isFinite(batteryVolts) && batteryVolts >= 9 && batteryVolts <= 16
                ? batteryVolts : BareMotorSpecifications.REFERENCE_VOLTAGE_VOLTS;
        double boundedScale = Double.isFinite(scale) ? Math.max(.5, Math.min(1.5, scale)) : 1;
        return BASE_F * BareMotorSpecifications.REFERENCE_VOLTAGE_VOLTS / voltage * boundedScale;
    }
    public static double estimatedFreeSpeedUsage(double outputRpm, double batteryVolts) {
        double volts = Double.isFinite(batteryVolts) && batteryVolts >= 9 && batteryVolts <= 16
                ? batteryVolts : BareMotorSpecifications.REFERENCE_VOLTAGE_VOLTS;
        return Math.abs(outputRpm) / ShooterSpeed.FLYWHEEL_MAX_RPM
                * BareMotorSpecifications.REFERENCE_VOLTAGE_VOLTS / volts;
    }
}
