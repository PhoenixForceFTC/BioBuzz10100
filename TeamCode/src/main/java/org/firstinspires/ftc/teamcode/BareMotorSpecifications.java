package org.firstinspires.ftc.teamcode;

/** Shared team baseline for the nominal 6000 RPM bare motors; verify each motor under load. */
public final class BareMotorSpecifications {
    private BareMotorSpecifications() { }
    public static final double NOMINAL_RPM = 6000.0;
    public static final double CONFIGURED_FREE_SPEED_RPM = 5550.0;
    public static final double ENCODER_TICKS_PER_REV = 28.0;
    public static final double REFERENCE_VOLTAGE_VOLTS = 12.0;
}
