package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import java.util.*;
import static org.junit.Assert.*;
import static org.firstinspires.ftc.teamcode.ControllerFixtures.*;

public class HardwareReviewRegressionTest {
    @Test public void missingNamedServoNeverSubstitutesAnInventoryDeviceOnAnotherHub() {
        Object wrongHub=new Object(), intended=new Object(); Map<String,Object> names=new HashMap<>();
        names.put("unrelated_servo_port_1",wrongHub);
        IllegalArgumentException error=assertThrows(IllegalArgumentException.class,()->NamedHardware.require(names::get,
                ()->"unrelated_servo_port_1 [Control Hub USB 1, port 1]","turret_servo_1","turretServo1"));
        assertTrue(error.getMessage().contains("turret_servo_1")); assertTrue(error.getMessage().contains("Control Hub"));
        names.put("turretServo1",intended); assertSame(intended,NamedHardware.require(names::get,()->"unused","turret_servo_1","turretServo1"));
    }
    @Test public void relativeZeroAndFaultCannotRequestHomeWithoutAnExplicitRetry() {
        TurretHomingController h=new TurretHomingController(1,.3,3,.2);
        assertFalse(h.commandsHome()); h.start(1); assertTrue(h.commandsHome());
        h.update(2,10,0); assertTrue(h.commandsHome()); h.update(2.31,10,0); assertTrue(h.ready()); assertFalse(h.commandsHome());
        h.invalidate(); h.update(2.5,100,30); assertFalse(h.ready()); assertFalse(h.commandsHome());
        h.start(3); h.update(6.1,100,30); assertEquals(TurretHomingController.State.FAULT,h.state()); assertFalse(h.commandsHome());
        h.start(7); assertTrue(h.commandsHome()); h.update(8,100,30); h.update(8.31,100,30); assertTrue(h.ready());
    }
    @Test public void delayedTrackingUsesCapturedTurretAndChassisThenCurrentChassis() {
        PoseHistory h=new PoseHistory(); ShooterMount m=new ShooterMount(new RobotMath.Vec(0,0),new RobotMath.Vec(0,0),new RobotMath.Vec(0,0),0,0,0,.3,true);
        h.add(pose(1,0,0,.1,0,0,0),0);
        for(int i=1;i<=10;i++) h.add(pose(1+i*.01,0,0,.1+i*.01,0,0,0),i*.01);
        // At capture: chassis .1 + turret 0 + target error +.1. Current chassis .2 => target yaw 0.
        assertEquals(m.cameraYaw,h.delayedYaw(1.1,1,-.1,m),1e-9);
        assertEquals(.2+m.cameraYaw,h.delayedYaw(1.1,1.1,-.1,m),1e-9);
        assertTrue(Double.isNaN(h.delayedYaw(2,1,-.1,m)));
        assertTrue(Double.isNaN(h.delayedYaw(1.1,.8,-.1,m)));
    }
    @Test public void acceptedVisionReducesUncertaintyForFullMatchAndThenDrifts() {
        PoseHistory h=new PoseHistory(); GoalTarget g=goal(2,0); ShooterMount m=mount();
        double ty=Math.atan2(.7,2)-m.cameraPitch;
        for(int i=0;i<=18000;i++) {
            double now=1+i*.01;
            MotionSample p=new MotionSample(now,0,0,0,0,0,0,0,.02+i*.00001,true,true,"field");
            h.add(p,0);
            if(i>10 && i%5==0) {
                assertTrue(h.reason(),h.observe(now,i,now-.05,30,0,ty,g,m));
                assertTrue(h.current(now).uncertainty<.15);
            }
        }
        double before=h.current(181).uncertainty;
        assertTrue(h.current(183).uncertainty>before); assertFalse(h.targetFresh(183));
    }
    @Test public void metricFusionRespectsJumpLimitAndDoesNotRewriteMeasuredVelocity() {
        PoseHistory h=new PoseHistory(); ShooterMount m=mount(); GoalTarget g=goal(2,0);
        for(int i=0;i<20;i++) h.add(pose(1+i*.01,.1,0,0,.2,0,0),0);
        double up=1-m.cameraHeight;
        TagObservation t=new TagObservation(30,0,2*Math.sin(m.cameraPitch)-up*Math.cos(m.cameraPitch),
                2*Math.cos(m.cameraPitch)+up*Math.sin(m.cameraPitch),Math.PI,0,0,1);
        assertTrue(h.reason(),h.observe3d(1.19,1,1.1,t,g,m));
        assertTrue(h.current(1.19).x<.1); assertEquals(.2,h.current(1.19).vx,0);
        assertTrue(h.current(1.19).uncertainty>=.025);
        TagObservation jump=new TagObservation(30,0,t.y,t.z+2,Math.PI,0,0,1);
        assertFalse(h.observe3d(1.19,2,1.1,jump,g,m));
    }
    @Test public void stationaryQualificationToleratesNoiseButRevokesActualMotionImmediately() {
        StationaryMotion q=new StationaryMotion();
        for(int i=0;i<=30;i++) {
            double t=1+i*.01;
            q.update(t,pose(t,0,0,0,i%2==0?.004:-.004,.002,.005),.01);
        }
        assertTrue(q.qualified());
        assertEquals(0,q.stationarySample(pose(1.3,0,0,0,.002,0,0)).vx,0);
        assertFalse(q.update(1.31,pose(1.31,0,0,0,.1,0,0),0));
        assertFalse(q.update(1.32,pose(1.32,0,0,0,0,0,0),0));
        assertFalse(q.update(2,pose(2,0,0,0,0,0,0),0));
    }
    @Test public void stationaryOnlyProfileNeedsQualificationAndMovingProfileStillNeedsFlightTimes() {
        ShotPlanner planner=planner(mount()); ShotProfile stationary=new ShotProfile("synthetic",true,
                new ShotProfile.Row(.2,1,3000,Double.NaN),new ShotProfile.Row(5,10,4000,Double.NaN));
        for(int i=0;i<30;i++) {
            double t=1+i*.01; ShotPlanner.Solution solution=planner.solve(t,pose(t,0,0,0,.001,0,.002),goal(2,0),stationary,0,.01,.3);
            if(i>20) assertTrue(solution.reason,solution.valid); else if(i==0) assertFalse(solution.valid);
        }
        ShotPlanner.Solution moving=planner.solve(1.3,pose(1.3,0,0,0,.2,0,0),goal(2,0),stationary,0,0,.3);
        assertFalse(moving.valid); assertTrue(moving.reason.contains("flight time"));
    }
    @Test public void localTargetCanAimWithoutInventingFieldReference() {
        MotionSample local=new MotionSample(1,0,0,0,0,0,0,0,.03,true,false,"local startup");
        GoalTarget field=goal(2,0), relative=field.inLocalFrame();
        assertFalse(planner(mount()).solve(1,local,field,profile(),0,0,.3).valid);
        ShotPlanner.Solution s=planner(mount()).solve(1,local,relative,profile(),0,0,.3);
        assertTrue(s.reason,s.valid); assertFalse(local.fieldReferenced); assertEquals(GoalTarget.Frame.LOCAL_ODOMETRY,relative.frame);
    }
    @Test public void transferTimingComesFromConfigurationAndLowImmediatelyClearsFull() {
        TransferController c=new TransferController(.6,.4);
        c.update(1,true,false,false); assertEquals(.6,c.stage1(),0); assertEquals(0,c.stage2(),0);
        c.update(1.39,true,false,false); assertFalse(c.full());
        c.update(1.41,true,false,false); assertTrue(c.full()); assertEquals(0,c.stage1(),0);
        c.update(1.42,false,false,false); assertFalse(c.full()); assertEquals(.6,c.stage1(),0);
        c.update(1.43,true,false,true); assertEquals(0,c.stage1(),0);
        c.update(1.5,true,true,true); assertEquals(.6,c.stage1(),0); assertEquals(.6,c.stage2(),0);
        c.stop(); assertEquals(0,c.stage1(),0); assertEquals(0,c.stage2(),0); assertFalse(c.full());
    }
    @Test public void traceNeverFeedsUntilGateDelayAndLossManualInhibitAndShutdownRemainDistinct() {
        FiringCoordinator f=ShootingConfiguration.firing(); TransferController c=new TransferController(1,2);
        c.stop(); assertEquals(0,c.stage1(),0);
        double opened=Double.NaN; boolean sawFeed=false;
        for(int i=0;i<=100;i++) {
            double t=i*.01;
            boolean manual=t>=.7, inhibit=t>=.95, raised=t<.55;
            f.update(t,true,manual,inhibit,false,raised,true,true,"Goal UNKNOWN / moving");
            if(f.gateOpen() && Double.isNaN(opened)) opened=t;
            if(!f.gateOpen()) opened=Double.NaN;
            boolean actualFeed=f.feeding() && !Double.isNaN(opened) && t-opened>=GateSubsystem.OPEN_SETTLE_TIME_SECONDS;
            c.update(t,i%3!=0,actualFeed,true);
            if(actualFeed) { sawFeed=true; assertTrue(f.gateOpen()); assertTrue(t-opened>=.25); assertEquals(1,c.stage1(),0); assertEquals(1,c.stage2(),0); }
            if(t>=.55 && t<.7 || inhibit) { assertFalse(f.gateOpen()); assertEquals(0,c.stage1(),0); assertEquals(0,c.stage2(),0); }
        }
        assertTrue(sawFeed);
        try { throw new IllegalStateException("simulated loop exception"); }
        catch(IllegalStateException expected) { /* adapter finally cleanup */ }
        finally { f.stop(1.1); c.stop(); }
        assertFalse(f.gateOpen()); assertFalse(f.feeding()); assertEquals(0,c.stage1(),0); assertEquals(0,c.stage2(),0);
    }
    @Test public void referenceResetInvalidatesCorrectionsHistoryAndStationaryQualification() {
        PoseHistory history=new PoseHistory();
        for(int i=0;i<30;i++) history.add(pose(1+i*.01,0,0,0,0,0,0).withReference(1),0);
        history.acceptRelative(1.2,.03); assertTrue(history.targetFresh(1.29));
        history.add(pose(1.3,2,3,0,0,0,0).withReference(2),0);
        assertNull(history.at(1.2)); assertFalse(history.targetFresh(1.3)); assertEquals(2,history.current(1.3).referenceId);
        StationaryMotion q=new StationaryMotion();
        for(int i=0;i<30;i++) q.update(1+i*.01,pose(1+i*.01,0,0,0,0,0,0).withReference(1),0);
        assertTrue(q.qualified()); assertFalse(q.update(1.3,pose(1.3,0,0,0,0,0,0).withReference(2),0));
    }
    @Test public void shutdownAttemptsAllDevicesAndPreservesFirstFailure() {
        java.util.List<String> called=new java.util.ArrayList<>();
        IllegalStateException failure=new IllegalStateException("hub disconnected");
        RuntimeException thrown=assertThrows(RuntimeException.class,()->RobotShutdown.stopAll(
                ()->{called.add("drive"); throw failure;},()->called.add("intake"),
                ()->{called.add("transfer"); throw new IllegalArgumentException("second device");},()->called.add("gate")));
        assertSame(failure,thrown); assertEquals(java.util.Arrays.asList("drive","intake","transfer","gate"),called);
        assertEquals(1,thrown.getSuppressed().length);
    }
    @Test public void metricDelayedTrackingCompensatesChassisTranslationAndAngleOnlyTrackingRejectsParallax() {
        ShooterMount m=new ShooterMount(new RobotMath.Vec(0,0),new RobotMath.Vec(0,0),new RobotMath.Vec(0,0),0,0,0,.335,true);
        PoseHistory h=new PoseHistory(); h.add(pose(1,0,0,0,0,0,0),0);
        for(int i=1;i<=10;i++) h.add(pose(1+i*.01,0,i*.01,0,0,0,0),i*.01);
        TagObservation tag=new TagObservation(30,0,-.915,2,Math.PI,0,0,1);
        assertEquals(Math.atan2(-.1,2),h.delayedYaw3d(1.1,1,tag,m),1e-9);
        assertTrue(Double.isNaN(h.delayedYaw(1.1,1,0,m)));
    }

    @Test public void qualifiedBoundedNoiseDoesNotBecomeMovingShotOrFalseAccelerationRejection() {
        ShotPlanner p=planner(mount()); ShotProfile table=new ShotProfile("synthetic",true,
                new ShotProfile.Row(.2,1,3000,Double.NaN),new ShotProfile.Row(5,10,4000,Double.NaN));
        ShotPlanner.Solution result=null;
        for(int i=0;i<40;i++) {
            double t=1+i*.01;
            MotionSample noisy=new MotionSample(t,0,0,0,i%2==0?.015:-.015,0,.01,3,2,.03,true,true,"bounded velocity noise");
            result=p.solve(t,noisy,goal(2,0),table,0,.01,.3);
        }
        assertNotNull(result); assertTrue(result.reason,result.valid);
        MotionSample moving=new MotionSample(1.4,0,0,0,.1,0,.1,3,2,.03,true,true,"actual motion");
        assertFalse(p.solve(1.4,moving,goal(2,0),table,0,0,.3).valid);
    }
    @Test public void acceptedFusionKeepsUncorrectedResidualInUncertaintyBudget() {
        PoseHistory h=new PoseHistory(); ShooterMount m=mount();
        for(int i=0;i<20;i++) h.add(pose(1+i*.01,.4,0,0,0,0,0),0);
        assertTrue(h.observe(1.19,1,1.1,30,0,Math.atan2(.7,2)-m.cameraPitch,goal(2,0),m));
        assertTrue(h.current(1.19).uncertainty>.25); // .35 gain leaves .26 m of the .4 m innovation.
        assertFalse(new ShotPlanner(m,ShootingConfiguration.ENVELOPE).solve(1.19,h.current(1.19),goal(2,0),profile(),0,0,.3).valid);
    }

}
