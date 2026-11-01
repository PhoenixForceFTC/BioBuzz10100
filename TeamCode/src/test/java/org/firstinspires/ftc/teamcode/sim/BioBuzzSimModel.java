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
            + "turret=355 deg/servo span,8192 CPR*108/40,synthetic slew=180 deg/s; "
            + "Pinpoint=32,-74 mm,reversed pods,2000/(32pi) ticks/mm; "
            + "camera=synthetic FOV54x41,30Hz,35ms latency; no contact/ball/Lynx model";
    final MecanumDrivePlant drive;
    final BioBuzzSimHardware hardware;
    double yawDegrees, yawVelocity;
    boolean jamTurret;

    BioBuzzSimModel(BioBuzzSimHardware hardware) {
        this.hardware = hardware;
        drive = hardware.wheels[0] == null ? null : new MecanumDrivePlant(hardware.wheels, this);
    }

    void update(double seconds) {
        // OpModeRunner already steps every fake device once before calling this method.
        if (drive != null) drive.update(seconds);
        if (hardware.yaw1 != null) {
            double target = (.5 - hardware.yaw1.getScaledPosition()) * 355;
            double delta = jamTurret ? 0 : Math.max(-180 * seconds, Math.min(180 * seconds, target - yawDegrees));
            yawDegrees += delta; yawVelocity = delta / seconds;
            hardware.encoder.setEncoderMeasuringAnotherMechanism(yawDegrees * 61.44, yawVelocity * 61.44);
        }
        double x = drive == null ? 0 : drive.getX();
        double y = drive == null ? 0 : drive.getY();
        double heading = drive == null ? 0 : drive.getHeading();
        if (hardware.pinpoint != null) hardware.pinpoint.moveChassisTo(x * 25.4, y * 25.4, heading);
        if (hardware.camera != null) hardware.camera.setRobotPose(
                x + 50.68184865 / 25.4 * Math.cos(heading),
                y + 50.68184865 / 25.4 * Math.sin(heading), heading + Math.toRadians(yawDegrees) - Math.PI / 2);
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
