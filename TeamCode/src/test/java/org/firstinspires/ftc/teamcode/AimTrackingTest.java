package org.firstinspires.ftc.teamcode;

import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.horizon36596.simloop.fakehardware.FakeMotor;
import org.horizon36596.simloop.fakehardware.FakeServo;
import org.horizon36596.simloop.fakehardware.FakeVoltageSensor;
import org.junit.Test;
import static org.junit.Assert.*;
import static org.firstinspires.ftc.teamcode.ControllerFixtures.*;

/** Turret actuation timing, aim lead and the auto-aim turn cap. Synthetic plants; no robot timing is measured. */
public class AimTrackingTest {
    @Test public void ledPoseUsesFixedFrameVelocityAndHeadingRateWithoutChangingFreshness() {
        MotionSample sample = pose(1, .2, -.1, 3.1, 1.5, -.5, 2);
        MotionSample led = sample.led(.05);
        assertEquals(.2 + .075, led.x, 1e-12); assertEquals(-.1 - .025, led.y, 1e-12);
        assertEquals(RobotMath.wrap(3.1 + .1), led.heading, 1e-12); // Wraps through +/-pi.
        assertEquals(sample.time, led.time, 0); assertTrue(led.fresh(1, .1));
        assertSame(sample, sample.led(0)); assertSame(sample, sample.led(Double.NaN));
    }

    @Test public void plannerLeadsOnlyTheServoCommandAndKeepsReadinessOnTheBearingNow() {
        MotionSample spinning = pose(1, 0, 0, 0, 0, 0, 2); // 2 rad/s CCW, centered pivot.
        ShotPlanner.Solution none = new ShotPlanner(mount(), 0).solve(1, spinning, goal(2, .4), profile());
        ShotPlanner.Solution led = new ShotPlanner(mount(), .05).solve(1, spinning, goal(2, .4), profile());
        assertTrue(led.reason, led.valid);
        assertEquals(none.yaw, led.yaw, 0); assertEquals(none.yaw, none.commandYaw, 0);
        // A CCW chassis turn needs the turret to turn CW relative to it: the command leads by -omega * lead.
        assertEquals(none.yaw - .1, led.commandYaw, 1e-12);
        assertEquals(none.hoodDeg, led.hoodDeg, 0); assertEquals(none.rpm, led.rpm, 0);
        ShotPlanner.Solution trimmed = led.withOffsets(2, 0, 0);
        assertEquals(led.yaw + Math.toRadians(2), trimmed.yaw, 1e-12);
        assertEquals(led.commandYaw + Math.toRadians(2), trimmed.commandYaw, 1e-12);
        assertEquals(led.commandYaw, led.inhibit("test").commandYaw, 0);
        // A stationary chassis has no lead.
        assertEquals(none.yaw, new ShotPlanner(mount(), .05).solve(1, pose(1, 0, 0, 0, 0, 0, 0), goal(2, .4), profile()).commandYaw, 0);
    }

    @Test(expected = IllegalArgumentException.class) public void implausibleLeadIsRejected() {
        new ShotPlanner(mount(), TurretControl.MAX_AIM_LEAD_SECONDS + .01);
    }

    @Test public void turretWritesThisLoopsTargetOnlyInApplyYawCommand() {
        FakeHardwareMap map = new FakeHardwareMap();
        map.register("turret_motor", new FakeMotor(2800, 8, 28));
        map.register("turret_encoder", new FakeMotor(2800, 8, 28));
        FakeServo yaw = new FakeServo();
        map.register("turret_servo_1", yaw); map.register("turret_servo_2", new FakeServo());
        map.register("turret_pitch", new FakeServo());
        map.register("Control Hub", new FakeVoltageSensor(12));
        double[] clock = {0};
        TurretSubsystem turret = new TurretSubsystem(map, false, () -> clock[0]);
        for (int i = 0; i < 150 && !turret.isHomed(); i++) { clock[0] += .02; turret.update(clock[0]); turret.applyYawCommand(clock[0]); }
        assertTrue(turret.getHomingStatus(), turret.isHomed());
        double home = yaw.getPosition();
        clock[0] += .02;
        turret.update(clock[0]);
        turret.setTargetYawDegrees(30);
        assertEquals("sensor update must not write the previous target", home, yaw.getPosition(), 0);
        turret.applyYawCommand(clock[0]);
        assertEquals(TurretSubsystem.angleDegreesToRotationServoPos(30), yaw.getPosition(), .002);
    }

    @Test public void lagEstimatorRecoversConstantLagAndIgnoresHoldsAndUnwinds() {
        TurretLagEstimator lag = new TurretLagEstimator();
        double rate = 200, delay = .04;
        for (int i = 0; i < 100; i++) {
            double t = i * .02, required = -150 + rate * t;
            lag.update(t, required, required - rate * delay, rate, -rate);
        }
        assertEquals(delay, lag.residualLagSeconds(), 1e-9);
        assertEquals(.025 + delay, lag.suggestedLeadSeconds(.025), 1e-9);
        assertEquals(rate, lag.peakChassisRateDegPerSecond(), 0);
        int samples = lag.samples();
        lag.update(3, 10, 10, 5, 0);        // Holding: too slow to measure lag.
        lag.update(3.02, 170, -160, 400, 0); // Unwinding the long way (30 deg wrapped error): excluded.
        assertEquals(samples, lag.samples());
        assertTrue(Double.isNaN(new TurretLagEstimator().residualLagSeconds()));
    }

    /** First-order synthetic chassis: rate = gain * clockwise stick, 0.1 s response. */
    private static double[] spin(AutoAimSpinLimiter limiter, double fullStickDegPerSecond, double stick,
                                 boolean aiming, double seconds) {
        double rate = 0, peak = 0;
        for (double t = 0; t < seconds; t += .02) {
            double command = limiter.limit(t, stick, Math.toRadians(rate), aiming);
            rate += (-command * fullStickDegPerSecond - rate) * -Math.expm1(-.02 / .1);
            peak = Math.max(peak, Math.abs(rate));
        }
        return new double[] {Math.abs(rate), peak};
    }

    @Test public void spinCapHoldsTheTurretTrackableRateOnlyWhileAiming() {
        double cap = AutoAimSpinLimiter.MAX_CHASSIS_RATE_DEG_PER_SECOND;
        for (double stick : new double[] {1, -1}) {
            double[] aiming = spin(new AutoAimSpinLimiter(), 650, stick, true, 3);
            assertEquals(cap, aiming[0], cap * .03);
            assertTrue("overshoot " + aiming[1], aiming[1] <= cap * 1.1);
        }
        AutoAimSpinLimiter free = new AutoAimSpinLimiter();
        assertEquals(650, spin(free, 650, 1, false, 3)[0], 5);
        assertFalse(free.limiting());
        assertEquals(.3, new AutoAimSpinLimiter().limit(0, .3, Double.NaN, true), 0); // Below the limit: unchanged.
    }

    @Test public void slowerRobotLearnsItNeedsNoCapAndMissingOdometryKeepsTheEstimate() {
        AutoAimSpinLimiter slow = new AutoAimSpinLimiter();
        assertEquals(250, spin(slow, 250, 1, true, 4)[0], 5);
        assertEquals(1, slow.stickLimit(), 0);
        AutoAimSpinLimiter blind = new AutoAimSpinLimiter();
        double expected = AutoAimSpinLimiter.MAX_CHASSIS_RATE_DEG_PER_SECOND / AutoAimSpinLimiter.INITIAL_FULL_STICK_RATE_DEG_PER_SECOND;
        assertEquals(expected, blind.limit(0, 1, Double.NaN, true), 1e-12);
        assertEquals(-expected, blind.limit(.02, -1, Double.NaN, true), 1e-12);
        assertTrue(blind.limiting());
    }
}
