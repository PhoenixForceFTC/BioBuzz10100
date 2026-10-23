package org.firstinspires.ftc.teamcode;

/** Shared Axon MAX+ specification for turret rotation, hood, and depollinator; excludes the gate. */
public final class AxonMaxPlus {
    private AxonMaxPlus() { }

    // https://docs.axon-robotics.com/archive/max — rated maximum speed, not measured loaded speed.
    // The servo programmer controls the internal PWM Power cap and Soft Start; FTC Servo cannot.
    // See doc/SoftwareConfiguration.md for the 6 V maximum-speed setup and factory PID settings.
    public static final double SECONDS_PER_60_DEG_AT_6V = 0.115;
    public static final double DEGREES_PER_SECOND_AT_6V = 60.0 / SECONDS_PER_60_DEG_AT_6V;
    // Robot programmer settings; verify these match each servo's stored configuration.
    public static final int MIN_PULSE_US = 500;
    public static final int MAX_PULSE_US = 2500;
    public static final double PROGRAMMED_TRAVEL_DEG = 355.0;
}
