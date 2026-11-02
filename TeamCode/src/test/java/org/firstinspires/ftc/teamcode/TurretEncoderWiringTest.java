package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.horizon36596.simloop.fakehardware.FakeDigitalChannel;
import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.horizon36596.simloop.fakehardware.FakeMotor;
import org.horizon36596.simloop.fakehardware.FakeServo;
import org.junit.Test;

import static org.junit.Assert.*;

/** Encoder-port routing and SDK direction regressions; no physical yaw calibration claim. */
public class TurretEncoderWiringTest {
    private final FakeHardwareMap map = new FakeHardwareMap();
    private final FakeMotor transfer = new FakeMotor(2590, 10, 28);

    private void configureHardware() {
        FakeMotor intake = new FakeMotor(2590, 10, 28);
        intake.setEncoderMeasuringAnotherMechanism(123, 456);
        map.register("intake_motor", intake);
        map.register("transfer_motor_1", transfer);
        map.register("transfer_motor_2", new FakeMotor(2590, 10, 28));
        map.register("turret_motor", new FakeMotor(2590, 10, 28));
        map.register("distance_sensor", new FakeDigitalChannel());
        for (String name : new String[]{"gate_servo", "turret_servo_1", "turret_servo_2", "turret_pitch"})
            map.register(name, new FakeServo());
        transfer.setEncoderMeasuringAnotherMechanism(6144, 3072);
    }

    @Test public void turretUsesTransferEncoderAndPreservesReverseMotorConfiguration() {
        configureHardware();
        // A legacy alias must not redirect feedback to the intake's newly connected encoder.
        map.register("turret_encoder", map.get(FakeMotor.class, "intake_motor"));
        new TransferSubsystem(map);
        transfer.setPower(.6);
        TurretSubsystem turret = new TurretSubsystem(map, false, () -> 0);
        turret.update(0);
        assertYawFeedback(turret);
        assertEquals(DcMotor.Direction.REVERSE, transfer.getDirection());
        assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER, transfer.getMode());
        assertEquals(.6, transfer.getPower(), 0);
        assertEquals(-6144, transfer.getCurrentPosition());
    }

    @Test public void laterTransferInitializationCannotFlipYawOrResetItsZero() {
        configureHardware();
        TurretSubsystem turret = new TurretSubsystem(map, false, () -> 0);
        turret.update(0);
        assertYawFeedback(turret);
        turret.zeroEncoderAtCurrentPosition();
        new TransferSubsystem(map);
        transfer.setEncoderMeasuringAnotherMechanism(12288, 3072);
        turret.update(.02);
        assertYawFeedback(turret);
    }

    @Test public void standaloneTurretEncoderAliasStillWorksWithoutTransferHardware() {
        configureHardware();
        map.remove(null, "transfer_motor_1", transfer);
        map.register("turret_encoder", transfer);
        TurretSubsystem turret = new TurretSubsystem(map, false, () -> 0);
        turret.update(0);
        assertYawFeedback(turret);
    }

    private void assertYawFeedback(TurretSubsystem turret) {
        assertEquals(6144, turret.getEncoderTicks());
        assertEquals(100, turret.getEncoderAngleDegrees(), 1e-9);
        assertEquals(50, turret.getEncoderVelocityDegPerSec(), 1e-9);
    }
}
