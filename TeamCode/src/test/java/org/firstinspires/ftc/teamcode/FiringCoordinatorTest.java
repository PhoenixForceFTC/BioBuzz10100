package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import static org.junit.Assert.*;
import static org.firstinspires.ftc.teamcode.ControllerFixtures.*;

public class FiringCoordinatorTest {
    private FiringCoordinator f() { return new FiringCoordinator(.2,.1,.15,.3,.1); }
    private void update(FiringCoordinator f,double now,boolean valid,boolean tight,boolean loose) {
        f.update(now,true,false,false,false,valid,tight,loose,"Invalid synthetic solution");
    }
    @Test public void stableReadinessGateTravelFeedingAndRpmRecovery() {
        FiringCoordinator f=f(); update(f,0,true,true,true); assertFalse(f.gateOpen());
        update(f,.11,true,true,true); assertTrue(f.gateOpen()); assertFalse(f.feeding());
        assertEquals(.2,f.releaseDelay(.21),1e-8);
        update(f,.32,true,true,true); assertTrue(f.feeding()); assertEquals(1,f.feedAttempts());
        assertEquals(.06,f.releaseDelay(.36),1e-8);
        update(f,.36,true,false,false); assertFalse(f.feeding()); assertFalse(f.gateOpen());
        assertEquals(FiringCoordinator.State.RECOVERY,f.state()); assertTrue(f.ballMayBeCommitted());
        update(f,.7,true,true,true); update(f,.71,true,true,true); update(f,.82,true,true,true);
        assertEquals(FiringCoordinator.State.GATE_OPENING,f.state());
    }
    @Test public void staleOrInvalidImmediatelyStopsFurtherFeedingAndInhibitWins() {
        FiringCoordinator f=f(); update(f,0,true,true,true); update(f,.11,true,true,true); update(f,.32,true,true,true);
        update(f,.33,false,true,true); assertFalse(f.feeding()); assertEquals(FiringCoordinator.State.INHIBITED,f.state());
        f.update(.4,true,true,true,false,true,true,true,""); assertFalse(f.gateOpen());
        assertEquals("Operator feed inhibit",f.reason());
    }
    @Test public void overridePriorityAndHysteresisAreDeterministic() {
        FiringCoordinator f=f();
        f.update(0,true,false,false,true,true,true,true,""); assertFalse(f.gateOpen());
        f.update(.1,true,true,false,true,false,false,false,"invalid"); assertTrue(f.gateOpen());
        f.update(.31,true,true,false,true,false,false,false,"invalid"); assertTrue(f.feeding());
        f.stop(.32); assertFalse(f.feeding()); assertFalse(f.gateOpen());
        f=f(); update(f,0,true,true,true); update(f,.11,true,true,true); update(f,.15,true,false,true);
        assertTrue(f.gateOpen()); // Entry tolerance differs from retention tolerance.
    }
    @Test public void movingSolutionFeedsWithLeadAndStopsOnTargetLoss() {
        FiringCoordinator f=f(); boolean sawFeed=false;
        for(int i=0;i<100;i++) {
            double now=1+i*.01;
            ShotPlanner.Solution s=planner(mount()).solve(now,pose(now,0,0,0,0,.4,0),goal(2,0),profile(),-.1,0,f.releaseDelay(now));
            assertTrue(s.reason,s.valid); assertTrue(s.lead<0);
            update(f,now,i<60,true,true); sawFeed|=f.feeding();
            if(i>=60) assertFalse(f.feeding());
        }
        assertTrue(sawFeed); assertTrue(f.feedAttempts()>0);
    }
    @Test public void manualFeedStaysContinuousAndAutoMustRequalifyAfterOverride() {
        FiringCoordinator f=f(); f.update(0,true,true,false,false,true,true,true,"");
        f.update(.21,true,true,false,false,true,true,true,""); assertTrue(f.feeding());
        f.update(1,true,true,false,false,true,true,true,""); assertTrue(f.feeding());
        f.update(1.01,true,false,false,false,true,true,true,""); assertFalse(f.feeding()); assertFalse(f.gateOpen());
        f.update(1.12,true,false,false,false,true,true,true,""); assertTrue(f.gateOpen());
    }
}
