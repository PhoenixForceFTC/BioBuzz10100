package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import org.firstinspires.ftc.teamcode.RobotMath.Vec;
import static org.junit.Assert.*;
import static org.firstinspires.ftc.teamcode.ControllerFixtures.*;

public class ShotPlannerTest {
    @Test public void velocityAndAccelerationDoNotChangeOrInhibitSamePoseLookup() {
        ShotPlanner p = planner(mount());
        ShotPlanner.Solution baseline = p.solve(1, pose(1,0,0,0,0,0,0), goal(2,.4), profile());
        for (double velocity : new double[] {-3, 0, 3}) {
            MotionSample sample = new MotionSample(1,0,0,0,velocity,2,4,20,30,.02,true,true,"motion");
            ShotPlanner.Solution s = p.solve(1,sample,goal(2,.4),profile());
            assertTrue(s.reason,s.valid);
            assertEquals(baseline.yaw,s.yaw,0); assertEquals(baseline.rpm,s.rpm,0);
            assertEquals(baseline.hoodDeg,s.hoodDeg,0); assertEquals(baseline.effectiveRange,s.effectiveRange,0);
        }
    }
    @Test public void rangeBearingHoodAndRpmUpdateAsPositionAndHeadingChange() {
        ShotPlanner p = planner(mount());
        ShotPlanner.Solution before = p.solve(1,pose(1,0,0,0,.4,0,.3),goal(2,0),profile());
        ShotPlanner.Solution after = p.solve(1.02,pose(1.02,.4,.2,.3,.4,0,.3),goal(2,0),profile());
        assertTrue(before.valid); assertTrue(after.valid);
        double distance = Math.hypot(1.6,-.2);
        assertEquals(Math.atan2(-.2,1.6)-.3,after.yaw,1e-9);
        assertEquals(distance,after.effectiveRange,1e-9);
        assertEquals(profile().sample(distance).rpm,after.rpm,1e-9);
        assertEquals(profile().sample(distance).hoodDeg,after.hoodDeg,1e-9);
        assertTrue(after.rpm < before.rpm);
    }
    @Test public void exactMountGeometryPointsMuzzleAtTarget() {
        ShooterMount m = new ShooterMount(new Vec(.12,-.08),new Vec(.06,.03),new Vec(.16,.02),0,.1,.2,.3,true);
        double heading=.2;
        ShotPlanner.Solution s=planner(m).solve(1,pose(1,.2,-.1,heading,.4,.3,.5),goal(2,.3),profile());
        assertTrue(s.reason,s.valid);
        double azimuth=heading+s.yaw;
        double muzzleX=.2+.12*Math.cos(heading)+.08*Math.sin(heading)+.16*Math.cos(azimuth)-.02*Math.sin(azimuth);
        double muzzleY=-.1+.12*Math.sin(heading)-.08*Math.cos(heading)+.16*Math.sin(azimuth)+.02*Math.cos(azimuth);
        assertEquals(2,muzzleX+s.effectiveRange*Math.cos(azimuth),1e-9);
        assertEquals(.3,muzzleY+s.effectiveRange*Math.sin(azimuth),1e-9);
    }
    @Test public void staleAndUnreferencedPosesRejectButUncertaintyIsInformational() {
        ShotPlanner p=planner(mount());
        assertFalse(p.solve(2,pose(1,0,0,0,0,0,0),goal(2,0),profile()).valid);
        MotionSample local=new MotionSample(1,0,0,0,0,0,0,0,.02,true,false,"local");
        assertFalse(p.solve(1,local,goal(2,0),profile()).valid);
        MotionSample uncertain=new MotionSample(1,0,0,0,0,0,0,0,1,true,true,"uncertain");
        assertTrue(p.solve(1,uncertain,goal(2,0),profile()).valid);
    }
    @Test public void uncalibratedOutOfRangeAndUnreachableTargetsAreRejected() {
        ShotPlanner p=planner(mount()); MotionSample sample=pose(1,0,0,0,0,0,0);
        assertFalse(p.solve(1,sample,goal(2,0),ShotProfile.example()).valid);
        assertFalse(p.solve(1,sample,goal(6,0),profile()).valid);
        assertFalse(p.solve(1,sample,goal(-2,0),profile()).valid);
    }
    @Test public void turningRecomputesCurrentBearingWithoutFuturePrediction() {
        ShotPlanner p=planner(mount());
        for (double heading : new double[] {-.4,0,.4,Math.PI-.01}) {
            GoalTarget target=Math.abs(heading)>3?goal(-2,0):goal(2,0);
            ShotPlanner.Solution s=p.solve(1,pose(1,0,0,heading,0,0,2),target,profile());
            assertTrue(s.reason,s.valid);
            assertEquals(RobotMath.wrap(Math.atan2(target.y,target.x)-heading),s.yaw,1e-9);
        }
    }
    @Test public void feedInhibitDoesNotFreezeAimSettings() {
        ShotPlanner.Solution s=planner(mount()).solve(1,pose(1,0,0,0,1,2,3),goal(2,0),profile());
        ShotPlanner.Solution inhibited=s.inhibit("Release timing unmeasured");
        assertFalse(inhibited.valid); assertTrue(inhibited.aimable);
        assertEquals(s.yaw,inhibited.yaw,0); assertEquals(s.rpm,inhibited.rpm,0);
        assertFalse(ShotPlanner.Solution.invalid("No profile").inhibit("No timing").aimable);
    }
    @Test public void shootingToleranceIncludesThreeDegreesWithoutVelocityRequirement() {
        assertTrue(ShotReadiness.angleReady(3)); assertTrue(ShotReadiness.angleReady(-3));
        assertFalse(ShotReadiness.angleReady(3.001)); assertFalse(ShotReadiness.angleReady(-3.001));
        assertFalse(ShotReadiness.angleReady(Double.NaN)); assertFalse(ShotReadiness.angleReady(Double.POSITIVE_INFINITY));
    }
}
