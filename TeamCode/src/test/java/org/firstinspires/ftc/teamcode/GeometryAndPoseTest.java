package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import org.firstinspires.ftc.teamcode.RobotMath.Vec;
import static org.junit.Assert.*;
import static org.firstinspires.ftc.teamcode.ControllerFixtures.*;

public class GeometryAndPoseTest {
    @Test public void installedCameraOffsetRotatesWithTurretAroundForwardPivot() {
        ShooterMount m = ShootingConfiguration.MOUNT;
        MotionSample p = pose(1, 0, 0, 0, 0, 0, 0);
        Vec forward = m.cameraPosition(p, Math.PI / 2); // Home is right; +90 CCW faces robot-forward.
        assertEquals(.05068184865, m.pivot.x, 1e-12);
        assertEquals(0, m.pivot.y, 0);
        assertEquals(-.04884889109, forward.x, 1e-12);
        assertEquals(0, forward.y, 1e-12);
        Vec right = m.cameraPosition(p, 0);
        assertEquals(.05068184865, right.x, 1e-12);
        assertEquals(.09953073974, right.y, 1e-12);
        assertEquals(0, m.muzzle.norm(), 0);
        assertEquals(0, m.cameraYaw, 0);
        assertEquals(Math.toRadians(30), m.cameraPitch, 0);
    }
    @Test public void bothAlliancesSidesAndStatesStayExplicit() {
        for (TurretSubsystem.Alliance a : TurretSubsystem.Alliance.values()) {
            for (TurretSubsystem.HiveSide side : TurretSubsystem.HiveSide.values()) {
                int id = a == TurretSubsystem.Alliance.RED ? side == TurretSubsystem.HiveSide.FAR ? 30 : 34
                        : side == TurretSubsystem.HiveSide.FAR ? 42 : 38;
                for (GoalTarget.HiveState state : GoalTarget.HiveState.values()) {
                    GoalTarget g = new GoalTarget(a,side,state,1,2,1.5,true,"map",new GoalTarget.Tag(id,1,1,1));
                    assertEquals(state == GoalTarget.HiveState.RAISED,g.scoreable());
                    assertEquals(a,g.alliance); assertEquals(side,g.side); assertTrue(g.tags.containsKey(id));
                }
                assertFalse(GoalTarget.unknown(a,side).scoreable());
            }
        }
    }
    @Test public void tagHeightAndCameraPivotOffsetsUseSeparateFrames() {
        ShooterMount m = new ShooterMount(new Vec(.2,.1),new Vec(.15,-.05),new Vec(.3,0),-.5,.1,.2,.3,true);
        MotionSample p = pose(1,1,2,Math.PI/2,0,0,0);
        Vec camera = m.cameraPosition(p,.5);
        assertEquals(.95,camera.x,1e-9); assertEquals(2.35,camera.y,1e-9);
        GoalTarget.Tag tag = new GoalTarget.Tag(30,0,0,.9);
        double cameraForward = 2 * Math.cos(.1), cameraLeft = -2 * Math.sin(.1);
        double opticalForward = cameraForward * Math.cos(.2) + .6 * Math.sin(.2);
        double opticalUp = -cameraForward * Math.sin(.2) + .6 * Math.cos(.2);
        Vec ray = m.cameraToTag(tag, Math.atan2(-cameraLeft,opticalForward), Math.atan2(opticalUp,opticalForward), .1);
        assertEquals(2,ray.x,1e-9); assertEquals(0,ray.y,1e-9);
        assertNull(m.cameraToTag(tag,0,-.2,.1));
    }
    @Test public void multipleTagsLocateOneOpeningWithoutChangingSelection() {
        GoalTarget g = goal(2,.2); PoseHistory h = new PoseHistory(); ShooterMount m = mount();
        for (int i=0;i<=20;i++) h.add(pose(1+i*.01,0,0,0,.2,0,0),0);
        for (int id : new int[]{30,31}) {
            GoalTarget.Tag tag = g.tags.get(id);
            double dz=tag.height-.3, opticalForward=tag.x*Math.cos(.2)+dz*Math.sin(.2);
            double opticalUp=-tag.x*Math.sin(.2)+dz*Math.cos(.2);
            assertTrue(h.reason(),h.observe(1.2,id,1.1,id,Math.atan2(-tag.y,opticalForward),Math.atan2(opticalUp,opticalForward),g,m));
        }
        assertEquals(2,g.x,0); assertEquals(.2,g.y,0); assertEquals(.2,h.current(1.2).vx,0);
        assertFalse(h.observe(1.2,40,1.1,34,0,.3,g,m));
    }
    @Test public void delayedRotatingCameraDuplicateAndStaleFrames() {
        PoseHistory h = new PoseHistory(); ShooterMount m=mount(); GoalTarget g=goal(2,0);
        for(int i=0;i<=30;i++) h.add(pose(1+i*.01,0,0,0,0,0,0),i*.01);
        double ty=Math.atan2(.7,2)-.2;
        double forward=2*Math.cos(.1), left=-2*Math.sin(.1);
        double optical=forward*Math.cos(.2)+.7*Math.sin(.2);
        assertTrue(h.reason(),h.observe(1.3,1,1.1,30,Math.atan2(-left,optical),Math.atan2(-forward*Math.sin(.2)+.7*Math.cos(.2),optical),g,m));
        assertEquals(0,h.current(1.3).x,1e-8); assertEquals(0,h.current(1.3).y,1e-8);
        assertFalse(h.observe(1.3,1,1.1,30,.1,ty,g,m));
        assertFalse(h.observe(2,2,1.1,30,.1,ty,g,m));
        assertFalse(h.targetFresh(2));
        for(int i=31;i<=100;i++) h.add(pose(1+i*.01,0,0,0,0,0,0),0);
        assertTrue(h.reason(),h.observe(2,3,1.95,30,0,ty,g,m)); assertTrue(h.targetFresh(2));
    }
    @Test public void correctionDoesNotSpikeVelocityAndRejectsBadReferenceOrJump() {
        PoseHistory h=new PoseHistory(); GoalTarget g=goal(2,0); ShooterMount m=mount();
        for(int i=0;i<=20;i++) h.add(pose(1+i*.01,.1,0,0,.3,0,0),0);
        assertTrue(h.observe(1.2,1,1.1,30,0,Math.atan2(.7,2)-.2,g,m));
        assertTrue(h.current(1.2).x < .1); assertEquals(.3,h.current(1.2).vx,0);
        double correctedX=h.current(1.2).x;
        h.invalidateObservation(false); assertFalse(h.targetFresh(1.2)); assertEquals(correctedX,h.current(1.2).x,1e-8);
        assertFalse(h.observe(1.2,2,1.1,30,0,Math.atan2(.7,4)-.2,g,m));
        h.reset();
        for(int i=0;i<=20;i++) h.add(new MotionSample(1+i*.01,0,0,0,0,0,0,0,.02,true,false,"local"),0);
        assertFalse(h.observe(1.2,3,1.1,30,0,.2,g,m));
    }
    @Test public void historyInterpolatesHeadingWrapAndLatencyIsCountedOnce() {
        PoseHistory h=new PoseHistory(); h.add(pose(1,0,0,Math.toRadians(179),0,0,0),Math.toRadians(179));
        h.add(pose(1.02,0,0,Math.toRadians(-179),0,0,0),Math.toRadians(-179));
        assertEquals(Math.PI,h.at(1.01).pose.heading,1e-8); assertEquals(Math.PI,h.at(1.01).turretYaw,1e-8);
        assertEquals(9.925,PoseHistory.captureTime(10,1000,950,10,15,0),1e-8);
        assertTrue(Double.isNaN(PoseHistory.captureTime(10,1000,1100,0,0,0)));
        assertNull(h.at(.9)); assertNull(h.at(1.03));
    }
    @Test public void profilesValidateCoverageLimitsAndInterpolation() {
        ShotProfile p=profile(); ShotProfile.Row r=p.sample(2);
        assertEquals(.5,r.flightSeconds,1e-9); assertNull(p.sample(.1)); assertNull(p.sample(5.01)); assertNull(p.sample(Double.NaN));
        assertThrows(IllegalArgumentException.class,()->new ShotProfile("x",true,new ShotProfile.Row(2,1,3000,.5),new ShotProfile.Row(1,2,4000,.6)));
        assertThrows(IllegalArgumentException.class,()->new ShotProfile("x",true,new ShotProfile.Row(1,31,3000,.5),new ShotProfile.Row(2,2,4000,.6)));
        assertThrows(IllegalArgumentException.class,()->new ShotProfile("x",true,new ShotProfile.Row(1,1,Double.NaN,.5),new ShotProfile.Row(2,2,4000,.6)));
        assertThrows(IllegalArgumentException.class,()->new ShotProfile("x",true,new ShotProfile.Row(1,1,3000,0),new ShotProfile.Row(2,2,4000,.6)));
        assertFalse(ShotProfile.example().calibrated);
    }
}
