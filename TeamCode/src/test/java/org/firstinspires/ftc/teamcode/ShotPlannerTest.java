package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import org.firstinspires.ftc.teamcode.RobotMath.Vec;
import static org.junit.Assert.*;
import static org.firstinspires.ftc.teamcode.ControllerFixtures.*;

public class ShotPlannerTest {
    @Test public void competitionAutoAimAllowsRatedSpeedSlewsAndRejectsUnreachableOnes() {
        ShotPlanner p = new ShotPlanner(mount(), ShootingConfiguration.ENVELOPE);
        MotionSample stationary = pose(1, 0, 0, 0, 0, 0, 0);
        // 60 deg in 0.15 s requires 400 deg/s: above the former 100 deg/s limit, below the 6 V rating.
        ShotPlanner.Solution reachable = p.solve(1, stationary, goal(2, 0), profile(), Math.toRadians(-60), 0, .15);
        assertTrue(reachable.reason, reachable.valid);
        // 90 deg in the same interval needs 600 deg/s; aiming may acquire, but feeding must wait.
        ShotPlanner.Solution tooFar = p.solve(1, stationary, goal(2, 0), profile(), Math.toRadians(-90), 0, .15);
        assertFalse(tooFar.valid); assertTrue(tooFar.aimable);
        ShotPlanner.Solution moving = p.solve(1, stationary, goal(2, 0), profile(), 0, Math.toRadians(200), .15);
        assertTrue(moving.reason, moving.valid);
        ShotPlanner.Solution tooFast = p.solve(1, stationary, goal(2, 0), profile(), 0, Math.toRadians(530), .15);
        assertFalse(tooFast.valid); assertTrue(tooFast.reason.contains("too fast"));
    }
    private ShotPlanner.Solution shot(double vx, double vy, double omega, double delay) {
        return planner(mount()).solve(1, pose(1, 0, 0, 0, vx, vy, omega), goal(2, 0), profile(), 0, 0, delay);
    }
    @Test public void zeroVelocityMatchesStationaryProfile() {
        ShotPlanner.Solution s = shot(0, 0, 0, .3); assertTrue(s.reason, s.valid);
        assertEquals(0, s.yaw, 1e-8); assertEquals(2, s.effectiveRange, 1e-8); assertEquals(.5, s.flightSeconds, 1e-8);
        assertEquals(profile().sample(2).rpm, s.rpm, 1e-8); assertEquals(0, s.lead, 1e-8);
    }
    @Test public void correctLeadAndRangeForLateralTowardAwayAndDiagonalMotion() {
        assertTrue(shot(0, .4, 0, .3).yaw < 0); assertTrue(shot(0, -.4, 0, .3).yaw > 0);
        assertTrue(shot(.4, 0, 0, .3).effectiveRange < 2); assertTrue(shot(-.4, 0, 0, .3).effectiveRange > 2);
        assertTrue(shot(.3, .4, 0, .3).valid);
        assertTrue(Math.abs(shot(0, .4, 0, .4).yaw) > Math.abs(shot(0, .4, 0, .1).yaw));
    }
    @Test public void interceptsIndependentSyntheticProjectileForTranslationRotationAndOffsets() {
        ShooterMount m = new ShooterMount(new Vec(.12, -.08), new Vec(.06, .03), new Vec(.16, .02), 0, .1, .2, .3, true);
        double[][] cases = {{.4,0,0,0},{0,.4,0,0},{.3,.4,0,0},{.2,-.2,.4,-.1},{-.3,.2,-.5,.2}};
        for (double[] c : cases) {
            double delay = .3, h0 = .2;
            MotionSample p = pose(1, 0, 0, h0, c[0], c[1], c[2]);
            ShotPlanner.Solution s = planner(m).solve(1, p, goal(2, .3), profile(), 0, c[3], delay);
            assertTrue(s.reason, s.valid);
            // Independent physical model: construct mount kinematics with scalar trig, integrate ball flight.
            double heading = h0 + c[2] * delay, azimuth = heading + s.yaw;
            double px = .12 * Math.cos(heading) + .08 * Math.sin(heading);
            double py = .12 * Math.sin(heading) - .08 * Math.cos(heading);
            double mx = .16 * Math.cos(azimuth) - .02 * Math.sin(azimuth);
            double my = .16 * Math.sin(azimuth) + .02 * Math.cos(azimuth);
            double x = c[0] * delay + px + mx, y = c[1] * delay + py + my;
            double vx = 4 * Math.cos(azimuth) + c[0] - c[2] * py - (c[2] + c[3]) * my;
            double vy = 4 * Math.sin(azimuth) + c[1] + c[2] * px + (c[2] + c[3]) * mx;
            int steps = 1000;
            for (int i = 0; i < steps; i++) { x += vx * s.flightSeconds / steps; y += vy * s.flightSeconds / steps; }
            assertEquals(2, x, .0005); assertEquals(.3, y, .0005);
            assertTrue(s.iterations <= ShotPlanner.MAX_ITERATIONS);
        }
    }
    @Test public void staleVelocityUninitializedPoseAndMotionEnvelopeAreRejected() {
        ShotPlanner p = planner(mount()); GoalTarget g = goal(2,0);
        assertFalse(p.solve(2, pose(1,0,0,0,0,0,0),g,profile(),0,0,.3).valid);
        MotionSample unreferenced = new MotionSample(1,0,0,0,0,0,0,0,.02,true,false,"local");
        assertTrue(p.solve(1,unreferenced,g,profile(),0,0,.3).reason.contains("field pose"));
        assertFalse(shot(3,0,0,.3).valid); assertFalse(shot(0,0,4,.3).valid);
        MotionSample accelerating = new MotionSample(1,0,0,0,0,0,0,8,.02,true,true,"synthetic");
        assertFalse(p.solve(1,accelerating,g,profile(),0,0,.3).valid);
        MotionSample uncertain = new MotionSample(1,0,0,0,0,0,0,0,1,true,true,"synthetic");
        assertFalse(p.solve(1,uncertain,g,profile(),0,0,.3).valid);
        MotionSample changingTurn = new MotionSample(1,0,0,0,0,0,0,0,3,.02,true,true,"synthetic");
        assertFalse(p.solve(1,changingTurn,g,profile(),0,0,.3).valid);
    }
    @Test public void stationaryOnlyProfilesMissingTimeRangeAndTurretLimits() {
        ShotProfile stationary = new ShotProfile("synthetic",true,new ShotProfile.Row(.2,1,3000,Double.NaN),new ShotProfile.Row(5,10,4000,Double.NaN));
        ShotPlanner p = planner(mount());
        assertTrue(p.solve(1,pose(1,0,0,0,0,0,0),goal(2,0),stationary,0,0,.3).valid);
        assertFalse(p.solve(1,pose(1,0,0,0,.2,0,0),goal(2,0),stationary,0,0,.3).valid);
        assertFalse(p.solve(1,pose(1,0,0,0,0,0,0),goal(6,0),profile(),0,0,.3).valid);
        assertTrue(p.solve(1,pose(1,0,0,0,0,0,0),goal(-2,0),profile(),0,0,.3).reason.contains("unreachable"));
        assertFalse(p.solve(1,pose(1,0,0,0,0,0,0),goal(2,0),profile(),-3,0,.01).valid);
    }
    @Test public void headingWrapAndBoundedNonconvergence() {
        ShotPlanner p = planner(mount());
        ShotPlanner.Solution s = p.solve(1,pose(1,0,0,Math.PI-.01,0,0,.1),goal(-2,-.02),profile(),0,0,.3);
        assertTrue(s.reason,s.valid); assertTrue(Math.abs(s.yaw) < .1);
        ShotProfile oscillating = new ShotProfile("synthetic",true,new ShotProfile.Row(.2,1,3000,2.9),new ShotProfile.Row(5,10,4000,.01));
        s = p.solve(1,pose(1,0,0,0,-1.5,0,0),goal(2,0),oscillating,0,0,.1);
        assertFalse(s.valid); assertEquals("Shot solver did not converge",s.reason);
    }
    @Test public void turningTrackingCommandReachesPredictedReleaseYawAsGateDelayDecreases() {
        ShotPlanner p=planner(mount());
        for(double elapsed:new double[]{0,.1,.2,.3}) {
            double delay=.3-elapsed, heading=.4*elapsed;
            ShotPlanner.Solution s=p.solve(1,pose(1,0,0,heading,0,0,.4),goal(2,0),profile(),-heading,-.4,delay);
            assertTrue(s.reason,s.valid); assertEquals(-.4,s.yawRate,1e-7);
            assertEquals(-heading,s.currentYaw(delay),1e-7);
            assertEquals(s.yaw,s.currentYaw(delay)+s.yawRate*delay,1e-7);
        }
    }
    @Test public void acquiringVisionMayAimButCannotAuthorizeFeed() {
        ShotPlanner.Solution s=shot(.2,.2,0,.3); assertTrue(s.valid);
        ShotPlanner.Solution inhibited=s.inhibit("Target observation lost / stale");
        assertFalse(inhibited.valid); assertTrue(inhibited.aimable); assertEquals(s.yaw,inhibited.yaw,0);
        assertFalse(ShotPlanner.Solution.invalid("No geometry").inhibit("No target").aimable);
    }
}
