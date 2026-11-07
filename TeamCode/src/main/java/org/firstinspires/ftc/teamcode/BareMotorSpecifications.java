package org.firstinspires.ftc.teamcode;

/** goBILDA 5000-0002-4008 shaft specs; shooter speed is configured separately in ShooterSpeed. */
public final class BareMotorSpecifications {
    private BareMotorSpecifications() { }
    public static final double NOMINAL_RPM = 6000.0;
    // Manufacturer-tested no-load speed at 12 V, not a measurement of the loaded robot.
    // https://www.gobilda.com/5000-series-12vdc-motor-with-8mm-rex-pinion-shaft/
    public static final double CONFIGURED_FREE_SPEED_RPM = 5800.0;
    public static final double ENCODER_TICKS_PER_REV = 28.0;
    public static final double REFERENCE_VOLTAGE_VOLTS = 12.0;
}
