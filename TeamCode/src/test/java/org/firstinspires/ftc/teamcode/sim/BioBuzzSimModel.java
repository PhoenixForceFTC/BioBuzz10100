package org.firstinspires.ftc.teamcode.sim;

import org.horizon36596.simloop.config.DrivetrainSimConfig;
import org.horizon36596.simloop.plant.MecanumDrivePlant;

/**
 * First-milestone model: independent wheel response and chassis motion only.
 * Geometry, effective ticks/in, strafe ratio and lag are synthetic estimates, NOT robot tuning.
 * Assertions check signs/relative motion, never measured speed, stopping distance, or shot success.
 */
final class BioBuzzSimModel implements DrivetrainSimConfig {
    static final double FREE_SPEED_TICKS_PER_SECOND = 28 * 5550.0 / 60;
    static final double RESPONSE_RATE_PER_SECOND = 10;
    static final String PARAMETERS = "seed=none; motor=28 ticks/rev,5550 RPM configured baseline; "
            + "SYNTHETIC: track=14 in,wheelbase=14 in,radius=2 in,effective ticks/in=64,"
            + "strafe ratio=0.85,lag=0.1 s; mounting signs=-1,+1,-1,+1; "
            + "no turret/flywheel/Pinpoint/vision/Lynx model";
    final MecanumDrivePlant drive;

    BioBuzzSimModel(BioBuzzSimHardware hardware) {
        drive = hardware.wheels[0] == null ? null : new MecanumDrivePlant(hardware.wheels, this);
    }

    void update(double seconds) {
        // OpModeRunner already steps every fake device once before calling this method.
        if (drive != null) drive.update(seconds);
    }

    @Override public double trackWidth() { return 14; }
    @Override public double wheelBase() { return 14; }
    @Override public double wheelRadius() { return 2; }
    @Override public double ticksPerInch() { return 64; }
    @Override public double maxVelocityTicksPerSecond() { return FREE_SPEED_TICKS_PER_SECOND; }
    @Override public double maxLateralVelocityTicksPerSecond() { return FREE_SPEED_TICKS_PER_SECOND * .85; }
    @Override public double maxAccel() { return RESPONSE_RATE_PER_SECOND; }
    // Installed left/right mounting convention, deliberately independent of motor.getDirection().
    @Override public double[] wheelMountingSigns() { return new double[] {-1, 1, -1, 1}; }
    @Override public double fieldHalfWidth() { return 72; }
    @Override public double fieldHalfHeight() { return 72; }
}
