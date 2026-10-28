package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import static org.junit.Assert.*;

public class ShooterSpeedTest {
    @Test public void outputRpmConvertsToMotorEncoderVelocityAndBack() {
        assertEquals(5550 * 2.0 / 3, ShooterSpeed.FLYWHEEL_MAX_RPM, 1e-9);
        assertEquals(3700, ShooterSpeed.FLYWHEEL_MAX_TARGET_RPM, 1e-9);
        assertEquals(2590, ShooterSpeed.motorTicksPerSecond(3700), 1e-9);
        assertEquals(1960, ShooterSpeed.motorTicksPerSecond(2800), 1e-9);
        assertEquals(4000, ShooterSpeed.flywheelRpm(2800), 1e-9);
        assertEquals(2800, ShooterSpeed.flywheelRpm(1960), 1e-9);
        for (double rpm : new double[] {-3400, -1000, 0, 1000, 2800, 3400})
            assertEquals(rpm, ShooterSpeed.flywheelRpm(ShooterSpeed.motorTicksPerSecond(rpm)), 1e-9);
    }
    @Test public void impossibleTargetsAreClippedAndInvalidTargetsStop() {
        double maximumTicks = 5550.0 / 60 * 28;
        assertEquals(maximumTicks, ShooterSpeed.motorTicksPerSecond(6000), 1e-9);
        assertEquals(-maximumTicks, ShooterSpeed.motorTicksPerSecond(-6000), 1e-9);
        assertEquals(0, ShooterSpeed.motorTicksPerSecond(Double.NaN), 0);
        assertEquals(0, ShooterSpeed.motorTicksPerSecond(Double.POSITIVE_INFINITY), 0);
    }
    @Test public void calibrationRejectsUnreachableFlywheelRpm() {
        ShotCalibration notebook = new ShotCalibration();
        assertEquals(3200 * 2.0 / 3, notebook.rpm(), 1e-9);
        assertThrows(IllegalArgumentException.class, () -> notebook.configure(24, 4001, 0));
        assertThrows(IllegalArgumentException.class, () -> new ShotProfile("too-fast", true,
                new ShotProfile.Row(1, 0, 3000), new ShotProfile.Row(2, 0, 4200)));
        assertEquals(2800, TurretSubsystem.SHOOTER_DEFAULT_TARGET_RPM, 1e-9);
        assertEquals(BareMotorSpecifications.CONFIGURED_FREE_SPEED_RPM, MecanumDrivetrain.BARE_MOTOR_MAX_RPM, 0);
        assertEquals(BareMotorSpecifications.CONFIGURED_FREE_SPEED_RPM, TurretSubsystem.SHOOTER_MOTOR_MAX_RPM, 0);
    }
}
