package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import static org.junit.Assert.*;

public class ShooterVelocityTuningTest {
    @Test public void bangBangRecoversAfterEveryShotAndCoastsWithoutBraking() {
        double target = 2800;
        double[] measured = {0, 2600, 2799, 2800, 2830, 2450, 2750, 2801};
        double[] expected = {1, 1, 1, 0, 0, 1, 1, 0};
        for (int i = 0; i < measured.length; i++) {
            assertEquals(expected[i], ShooterVelocityTuning.bangBangPower(target, measured[i]), 0);
            assertEquals(-expected[i], ShooterVelocityTuning.bangBangPower(-target, -measured[i]), 0);
        }
        assertEquals(1, ShooterVelocityTuning.bangBangPower(target, -100), 0);
    }
    @Test public void bangBangStopAndInvalidMeasurementsNeverPowerMotor() {
        assertEquals(0, ShooterVelocityTuning.bangBangPower(0, -100), 0);
        for (double invalid : new double[] {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY}) {
            assertEquals(0, ShooterVelocityTuning.bangBangPower(2800, invalid), 0);
            assertEquals(0, ShooterVelocityTuning.bangBangPower(invalid, 0), 0);
        }
    }
    @Test public void feedforwardMatchesConfiguredMotorSpeedAndCompensatesBatterySag() {
        double base = 32767.0 / (5550.0 / 60 * 28);
        assertEquals(base, ShooterVelocityTuning.feedforward(12, 1), 1e-9);
        assertEquals(base * 12 / 10, ShooterVelocityTuning.feedforward(10, 1), 1e-9);
        assertEquals(base * 12 / 14, ShooterVelocityTuning.feedforward(14, 1), 1e-9);
        assertEquals(base * 1.1, ShooterVelocityTuning.feedforward(12, 1.1), 1e-9);
    }
    @Test public void invalidVoltageFallsBackAndTuningAdjustmentsAreBounded() {
        double base = ShooterVelocityTuning.BASE_F;
        for (double v : new double[] {0, -1, Double.NaN, Double.POSITIVE_INFINITY})
            assertEquals(base, ShooterVelocityTuning.feedforward(v, 1), 1e-9);
        assertEquals(base * 1.5, ShooterVelocityTuning.feedforward(12, 10), 1e-9);
        assertEquals(base * .5, ShooterVelocityTuning.feedforward(12, -1), 1e-9);
        assertEquals(base, ShooterVelocityTuning.feedforward(12, Double.NaN), 1e-9);
    }
    @Test public void fullSpeedTargetUsesTheConfiguredMaximumWithoutReserve() {
        assertEquals(1, ShooterVelocityTuning.estimatedFreeSpeedUsage(ShooterSpeed.FLYWHEEL_MAX_TARGET_RPM, 12), 1e-9);
        assertTrue(ShooterVelocityTuning.estimatedFreeSpeedUsage(ShooterSpeed.FLYWHEEL_MAX_TARGET_RPM, 10) > 1);
        assertEquals(0, ShooterVelocityTuning.estimatedFreeSpeedUsage(0, 12), 0);
    }
}
