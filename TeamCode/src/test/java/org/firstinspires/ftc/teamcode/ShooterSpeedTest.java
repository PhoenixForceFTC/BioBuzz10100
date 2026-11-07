package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import static org.junit.Assert.*;

public class ShooterSpeedTest {
    @Test public void outputRpmConvertsToMotorEncoderVelocityAndBack() {
        assertEquals(6000, ShooterSpeed.FLYWHEEL_MAX_RPM, 1e-9);
        assertEquals(6000, ShooterSpeed.FLYWHEEL_MAX_TARGET_RPM, 1e-9);
        assertEquals(2800, ShooterSpeed.motorTicksPerSecond(6000), 1e-9);
        assertEquals(2800.0 / 60 * 28, ShooterSpeed.motorTicksPerSecond(2800), 1e-9);
        assertEquals(6000, ShooterSpeed.flywheelRpm(2800), 1e-9);
        assertEquals(2800, ShooterSpeed.flywheelRpm(2800.0 / 60 * 28), 1e-9);
        for (double rpm : new double[] {-3400, -1000, 0, 1000, 2800, 3400, 6000})
            assertEquals(rpm, ShooterSpeed.flywheelRpm(ShooterSpeed.motorTicksPerSecond(rpm)), 1e-9);
    }
    @Test public void impossibleTargetsAreClippedAndInvalidTargetsStop() {
        double maximumTicks = 6000.0 / 60 * 28;
        assertEquals(maximumTicks, ShooterSpeed.motorTicksPerSecond(7000), 1e-9);
        assertEquals(-maximumTicks, ShooterSpeed.motorTicksPerSecond(-7000), 1e-9);
        assertEquals(0, ShooterSpeed.motorTicksPerSecond(Double.NaN), 0);
        assertEquals(0, ShooterSpeed.motorTicksPerSecond(Double.POSITIVE_INFINITY), 0);
    }
    @Test public void calibrationRejectsUnreachableFlywheelRpm() {
        ShotCalibration notebook = new ShotCalibration();
        assertEquals(2133.3333333333335, notebook.rpm(), 1e-9);
        assertThrows(IllegalArgumentException.class, () -> notebook.configure(24, 6001, 0));
        assertThrows(IllegalArgumentException.class, () -> new ShotProfile("too-fast", true,
                new ShotProfile.Row(1, 0, 3000), new ShotProfile.Row(2, 0, 6001)));
        notebook.configure(24, 6000, 0);
        assertEquals(6000, notebook.rpm(), 0);
        ShotProfile fullSpeed = new ShotProfile("full-speed", true,
                new ShotProfile.Row(1, 0, 5000), new ShotProfile.Row(2, 0, 6000));
        assertEquals(6000, fullSpeed.sample(2).rpm, 0);
        assertEquals(100, TurretSubsystem.SHOOTER_READY_TOLERANCE_RPM, 0);
        assertEquals(2800, TurretSubsystem.SHOOTER_DEFAULT_TARGET_RPM, 1e-9);
        assertEquals(BareMotorSpecifications.CONFIGURED_FREE_SPEED_RPM, MecanumDrivetrain.BARE_MOTOR_MAX_RPM, 0);
        assertEquals(6000, TurretSubsystem.SHOOTER_MOTOR_MAX_RPM, 0);
    }
}
