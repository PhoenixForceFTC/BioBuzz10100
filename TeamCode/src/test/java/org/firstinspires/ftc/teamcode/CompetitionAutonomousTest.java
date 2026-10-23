package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import java.util.EnumSet;
import static org.junit.Assert.*;
import static org.firstinspires.ftc.teamcode.ControllerFixtures.*;

public class CompetitionAutonomousTest {
    private CompetitionAutonomousController controller(boolean preload,int repeat) {
        CompetitionAutonomousController c=new CompetitionAutonomousController(strategy(preload,repeat),PollenConfiguration.diagnostic()); c.start(1); return c;
    }
    @Test public void fullPartialAndEmptyCollectionReturnToScoreThenPark() {
        for(int loadState:new int[]{0,1,2}) { // Empty, partial pickup, and full sensor confirmation.
            CompetitionAutonomousController c=controller(true,1);
            EnumSet<CompetitionAutonomousController.State> seen=EnumSet.noneOf(CompetitionAutonomousController.State.class);
            double x=0,y=0,h=0; int attempts=0;
            FiringCoordinator firing=new FiringCoordinator(.2,.1,.15,.3,.1);
            for(int i=0;i<2900 && !c.finished();i++) {
                double now=1+i*.01; seen.add(c.state());
                MotionSample pose=pose(now,x,y,h,0,0,0);
                PollenTarget pollen=loadState==1 ? PollenTarget.fromFloorContact((long)(now*1e9),160,180,160,200,
                        1000,200,200,160,120,180,10,0,0) : null;
                c.update(now,pose,pollen,true,loadState==2,false);
                // Simulated hardware: scaled robot-centric motion -> field frame.
                double forward=c.axial(), left=-c.lateral();
                x+=(forward*Math.cos(h)-left*Math.sin(h))*.01;
                y+=(forward*Math.sin(h)+left*Math.cos(h))*.01; h-=c.yaw()*.01;
                firing.update(now,c.fireRequested(),false,false,false,true,true,true,"");
                attempts=firing.feedAttempts();
            }
            assertEquals(c.reason(),CompetitionAutonomousController.State.COMPLETE,c.state());
            assertTrue(seen.contains(CompetitionAutonomousController.State.COLLECTING));
            assertTrue(seen.contains(CompetitionAutonomousController.State.SCORING));
            assertTrue(seen.contains(CompetitionAutonomousController.State.PARKING)); assertTrue(attempts>0);
        }
    }
    @Test public void noTargetOrFailedReadinessHasBoundedScoreWindows() {
        CompetitionAutonomousController c=controller(true,0); FiringCoordinator f=new FiringCoordinator(.2,.1,.15,.3,.1);
        for(int i=0;i<1000;i++) {
            double now=1+i*.01; c.update(now,pose(now,.3,0,0,0,0,0),null,false,false,false);
            f.update(now,c.fireRequested(),false,false,false,false,false,false,"No target / readiness"); assertFalse(f.feeding());
        }
        assertNotEquals(CompetitionAutonomousController.State.COLLECTING,c.state());
    }
    @Test public void navigationStallFallsBackToParkingAndLocalizationFailureStops() {
        CompetitionAutonomousController c=controller(false,0);
        c.update(1.1,pose(1.1,0,0,0,0,0,0),null,true,false,false);
        c.update(2.7,pose(2.7,0,0,0,0,0,0),null,true,false,false);
        assertEquals(CompetitionAutonomousController.State.PARKING,c.state()); assertEquals(0,c.axial(),0);
        c.update(2.8,null,null,true,false,false); assertEquals(CompetitionAutonomousController.State.FAULT,c.state());
        assertFalse(c.fireRequested()); assertEquals(0,c.intake(),0);
    }
    @Test public void deadlineTriggersParkingThenStopsAndStopWorksInEveryReachableState() {
        for(CompetitionAutonomousController.State desired:CompetitionAutonomousController.State.values()) {
            if(desired==CompetitionAutonomousController.State.IDLE || desired==CompetitionAutonomousController.State.FAULT) continue;
            CompetitionAutonomousController c=controller(true,1); double x=0,y=0,h=0; boolean reached=false;
            for(int i=0;i<3000&&!reached;i++) {
                double now=1+i*.01;
                if(c.state()==desired) { c.update(now,pose(now,x,y,h,0,0,0),null,true,false,true); reached=true; break; }
                c.update(now,pose(now,x,y,h,0,0,0),null,true,false,false);
                x+=c.axial()*.01; y-=c.lateral()*.01; h-=c.yaw()*.01;
            }
            assertTrue("Reached "+desired,reached); assertTrue(c.finished()); assertEquals(0,c.axial(),0); assertFalse(c.fireRequested());
        }
        CompetitionAutonomousController c=controller(true,0);
        c.update(26.1,pose(26.1,0,0,0,0,0,0),null,true,false,false);
        assertEquals(CompetitionAutonomousController.State.PARKING,c.state());
        c.update(30.1,pose(30.1,0,0,0,0,0,0),null,true,false,false); assertTrue(c.finished()); assertEquals(0,c.axial(),0);
    }
    @Test public void unapprovedInvalidAndClearanceViolatingRoutesAreRejected() {
        CompetitionAutonomousController c=new CompetitionAutonomousController(null,PollenConfiguration.diagnostic()); c.start(1);
        assertEquals(CompetitionAutonomousController.State.FAULT,c.state());
        AutonomousStrategy s=strategy(false,0); assertFalse(s.segmentClear(0,0,2,0));
        assertThrows(IllegalArgumentException.class,()->new AutonomousStrategy("bad",true,TurretSubsystem.Alliance.RED,s.start,s.target,
                new AutonomousStrategy.Waypoint[]{new AutonomousStrategy.Waypoint(2,0,0,false)},s.scoring(),s.parking(),
                new AutonomousStrategy.Obstacle[0],.2,29,4,5,2,1,false,0));
        assertThrows(IllegalArgumentException.class,()->new AutonomousStrategy("blocked",true,TurretSubsystem.Alliance.RED,s.start,s.target,
                s.collection(),s.scoring(),s.parking(),new AutonomousStrategy.Obstacle[]{new AutonomousStrategy.Obstacle(.1,-.1,.2,.1)},
                .05,29,4,5,2,1,false,0));
    }
    @Test public void waypointHeadingWrapAndClockwiseDriveConvention() {
        AutonomousStrategy s=strategy(false,0); WaypointFollower f=new WaypointFollower();
        f.start(1,s,new AutonomousStrategy.Waypoint[]{new AutonomousStrategy.Waypoint(0,.3,Math.toRadians(-179),false)});
        f.update(1.1,pose(1.1,0,0,Math.toRadians(179),0,0,0));
        assertTrue(f.yaw()<0); assertTrue(Math.abs(f.yaw())<.1); assertTrue(f.lateral()>0);
    }
    @Test public void movingShotsOnlyOnDesignatedSegmentsAndStationaryScoringWaitsForStop() {
        CompetitionAutonomousController c=controller(false,0);
        c.update(1.1,pose(1.1,.3,0,0,0,0,0),null,true,false,false);
        assertFalse(c.fireRequested()); c.update(1.2,pose(1.2,.3,0,0,0,0,0),null,true,true,false);
        c.update(1.3,pose(1.3,.3,0,0,0,0,0),null,true,true,false); assertTrue(c.fireRequested());
        c.update(1.4,pose(1.4,.5,0,0,0,0,0),null,true,true,false);
        c.update(1.5,pose(1.5,.5,0,0,.2,0,0),null,true,true,false); assertFalse(c.fireRequested());
    }
}
