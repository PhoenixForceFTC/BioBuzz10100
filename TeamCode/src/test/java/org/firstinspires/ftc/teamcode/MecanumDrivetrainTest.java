package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import static org.junit.Assert.*;

public class MecanumDrivetrainTest {
    @Test public void reportedTransmissionHas364MotorRevolutionsPer30WheelRevolutions() {
        assertEquals(364.0 / 30, MecanumDrivetrain.MOTOR_REVS_PER_WHEEL_REV, 1e-12);
        assertEquals(339.7333333333333, MecanumDrivetrain.WHEEL_TICKS_PER_REV, 1e-9);
        assertEquals(28, MecanumDrivetrain.BARE_MOTOR_TICKS_PER_REV, 0);
        assertEquals(5800, MecanumDrivetrain.BARE_MOTOR_MAX_RPM, 0);
        assertEquals(478.02197802197804, MecanumDrivetrain.WHEEL_MAX_RPM, 1e-9);
    }

    @Test public void motorEncoderVelocityConvertsToWheelRpmInBothDirections() {
        // 364 motor RPM is 30 wheel RPM; encoder velocity remains 28 ticks/motor revolution.
        assertEquals(30, MecanumDrivetrain.motorTicksPerSecondToWheelRpm(364.0 * 28 / 60), 1e-9);
        assertEquals(-30, MecanumDrivetrain.motorTicksPerSecondToWheelRpm(-364.0 * 28 / 60), 1e-9);
        assertEquals(0, MecanumDrivetrain.motorTicksPerSecondToWheelRpm(0), 0);
        assertEquals(478.02197802197804,
                MecanumDrivetrain.motorTicksPerSecondToWheelRpm(5800.0 * 28 / 60), 1e-9);
    }

    @Test public void reported104mmWheelDefinesNominalEncoderDistanceAndNoLoadSpeed() {
        assertEquals(104, MecanumDrivetrain.WHEEL_DIAMETER_MM, 0);
        assertEquals(326.7256359733385,
                MecanumDrivetrain.WHEEL_TICKS_PER_REV / MecanumDrivetrain.MOTOR_TICKS_PER_MM, 1e-9);
        assertEquals(1.039812294867052, MecanumDrivetrain.MOTOR_TICKS_PER_MM, 1e-12);
        assertEquals(2603.0339129744, MecanumDrivetrain.WHEEL_NO_LOAD_SPEED_MM_PER_SEC, 1e-9);
    }
}
