package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.horizon36596.simloop.fakehardware.FakeMotor;
import org.horizon36596.simloop.fakehardware.FakeServo;
import org.horizon36596.simloop.fakehardware.FakeVoltageSensor;
import org.junit.Test;
import static org.junit.Assert.*;

/** Hub command regressions only: fake hardware does not establish loaded flywheel tuning. */
public class ShooterLiveTuningTest {
    private static final class RecordingMotor extends FakeMotor {
        int writes;
        double p, i, d, f;
        RecordingMotor() { super(2590, 8, 28); }
        @Override public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
            writes++;
            this.p = p; this.i = i; this.d = d; this.f = f;
            super.setVelocityPIDFCoefficients(p, i, d, f);
        }
    }

    private TurretSubsystem turret(RecordingMotor motor) {
        FakeHardwareMap map = new FakeHardwareMap();
        map.register("turret_motor", motor);
        map.register("turret_encoder", new FakeMotor(2800, 8, 28));
        for (String name : new String[] {"turret_servo_1", "turret_servo_2", "turret_pitch"})
            map.register(name, new FakeServo());
        map.register("Control Hub", new FakeVoltageSensor(10));
        return new TurretSubsystem(map, false, () -> 0);
    }

    @Test public void liveGainsReachHubWithoutRestartingVelocityOrRewritingUnchangedSettings() {
        RecordingMotor motor = new RecordingMotor();
        TurretSubsystem turret = turret(motor);
        turret.setShooterRpm(2800);
        int initialWrites = motor.writes;
        turret.setShooterVelocityTuning(45, .05, 1, 1.1);
        assertEquals(initialWrites + 1, motor.writes);
        assertEquals(45, motor.p, 0);
        assertEquals(.05, motor.i, 0);
        assertEquals(1, motor.d, 0);
        assertEquals(ShooterVelocityTuning.BASE_F * 12 / 10 * 1.1, motor.f, 1e-9);
        assertEquals(2800, turret.getTargetShooterRpm(), 0);
        assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, motor.getMode());
        for (int n = 0; n < 100; n++) turret.setShooterVelocityTuning(45, .05, 1, 1.1);
        assertEquals(initialWrites + 1, motor.writes);
        turret.resetShooterVelocityTuning();
        assertEquals(ShooterVelocityTuning.INITIAL_P, motor.p, 0);
        assertEquals(ShooterVelocityTuning.INITIAL_I, motor.i, 0);
        assertEquals(ShooterVelocityTuning.INITIAL_D, motor.d, 0);
        assertEquals(ShooterVelocityTuning.feedforward(10, 1), motor.f, 1e-9);
        turret.stop();
        assertEquals(0, motor.getPower(), 0);
    }

    @Test public void invalidOrOutOfRangeLiveGainsCannotSendInvalidCoefficientsToHub() {
        RecordingMotor motor = new RecordingMotor();
        TurretSubsystem turret = turret(motor);
        turret.setShooterVelocityTuning(500, -1, 500, 10);
        assertEquals(100, motor.p, 0);
        assertEquals(0, motor.i, 0);
        assertEquals(100, motor.d, 0);
        assertEquals(ShooterVelocityTuning.feedforward(10, 1.5), motor.f, 1e-9);
        int writes = motor.writes;
        turret.setShooterVelocityTuning(Double.NaN, 1, 0, 1);
        turret.setShooterVelocityTuning(40, Double.POSITIVE_INFINITY, 0, 1);
        turret.setShooterVelocityTuning(40, 0, Double.NaN, 1);
        turret.setShooterVelocityTuning(40, 0, 0, Double.NEGATIVE_INFINITY);
        assertEquals(writes, motor.writes);
        turret.stop();
    }
}
