package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import static org.junit.Assert.*;

public class TransferControllerTest {
    @Test public void continuousHighConfirmsFullAndAnyLowResetsIt() {
        TransferController c=new TransferController(); c.update(0,true,false,false); c.update(1.99,true,false,false);
        assertFalse(c.full()); assertEquals(1,c.stage1(),0); assertEquals(0,c.stage2(),0);
        c.update(2,true,false,false); assertTrue(c.full()); assertEquals(0,c.stage1(),0);
        c.update(2.01,false,false,false); assertFalse(c.full()); assertEquals(1,c.stage1(),0);
        c.update(3,true,false,false); c.update(4,true,false,false); assertFalse(c.full());
    }
    @Test public void feedingBypassesFullAndPauseButOtherwisePauseStopsBothStages() {
        TransferController c=new TransferController(); c.update(0,true,false,false); c.update(3,true,true,true);
        assertTrue(c.full()); assertEquals(1,c.stage1(),0); assertEquals(1,c.stage2(),0);
        c.update(3.1,true,false,true); assertEquals(0,c.stage1(),0); assertEquals(0,c.stage2(),0);
        c.stop(); assertFalse(c.full()); assertEquals(0,c.stage1(),0);
    }
    @Test public void fullIntakeLatchAndBumperPrioritiesArePreserved() {
        IntakeQueueController c=new IntakeQueueController();
        assertEquals(-1,c.update(true,false,false,false),0);
        assertEquals(1,c.update(true,true,true,false),0);
        assertEquals(0,c.update(true,true,true,true),0);
        assertEquals(-1,c.update(true,true,false,false),0);
        assertEquals(0,c.update(true,false,false,false),0);
        assertEquals(0,c.update(true,false,false,false),0);
        c.update(false,false,false,false); assertEquals(-1,c.update(true,false,false,false),0);
    }
}
