package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import static org.junit.Assert.*;

public class TransferControllerTest {
    @Test public void teleOpStopsAtLatchedFullAndResumesAfterFeedClearsQueue() {
        IntakeQueueController intake = new IntakeQueueController(0);
        TransferController transfer = new TransferController();
        transfer.update(0, true, false, false);
        assertEquals(1, intake.update(transfer.full(), false, false), 0);
        transfer.update(2, true, false, false);
        assertEquals(0, intake.update(transfer.full(), false, false), 0);
        transfer.update(2.1, false, false, false);
        assertEquals(0, intake.update(transfer.full(), false, false), 0);
        assertEquals(1, intake.update(transfer.full(), true, false), 0);
        assertEquals(-1, intake.update(transfer.full(), false, true), 0);
        assertEquals(0, intake.update(transfer.full(), true, true), 0);
        transfer.update(2.5, false, true, true);
        assertEquals(1, intake.update(transfer.full(), false, false), 0);
    }
    @Test public void loadingKeepsStage2OffAndFeedingRunsBothForward() {
        TransferController c=new TransferController(.6,.4);
        c.update(0,false,false,false);
        assertEquals(.6,c.stage1(),0); assertEquals(0,c.stage2(),0);
        // Gate travel pauses both stages before the caller permits feeding.
        c.update(.1,false,false,true);
        assertEquals(0,c.stage1(),0); assertEquals(0,c.stage2(),0);
        c.update(.4,false,true,true);
        assertEquals(.6,c.stage1(),0); assertEquals(.6,c.stage2(),0);
        c.update(.6,false,false,false);
        assertEquals(.6,c.stage1(),0); assertEquals(0,c.stage2(),0);
        c.stop(); assertEquals(0,c.stage1(),0); assertEquals(0,c.stage2(),0);
    }
    @Test public void confirmedFullHoldsTransfersUntilFeedingEvenIfSensorClears() {
        TransferController c=new TransferController(); c.update(0,true,false,false); c.update(1.99,true,false,false);
        assertFalse(c.full()); assertEquals(1,c.stage1(),0); assertEquals(0,c.stage2(),0);
        c.update(2,true,false,false); assertTrue(c.full()); assertEquals(0,c.stage1(),0); assertEquals(0,c.stage2(),0);
        c.update(2.01,false,false,false); assertTrue(c.full()); assertEquals(0,c.stage1(),0); assertEquals(0,c.stage2(),0);
        c.update(3,false,true,true); assertFalse(c.full()); assertEquals(1,c.stage1(),0); assertEquals(1,c.stage2(),0);
        c.update(4,false,false,false); assertEquals(1,c.stage1(),0); assertEquals(0,c.stage2(),0);
    }
    @Test public void feedingBypassesFullAndPauseButOtherwisePauseStopsBothStages() {
        TransferController c=new TransferController(); c.update(0,true,false,false); c.update(3,true,true,true);
        assertTrue(c.full()); assertEquals(1,c.stage1(),0); assertEquals(1,c.stage2(),0);
        c.update(3.1,true,false,true); assertEquals(0,c.stage1(),0); assertEquals(0,c.stage2(),0);
        c.stop(); assertFalse(c.full()); assertEquals(0,c.stage1(),0);
    }
    @Test public void intakeAlwaysFollowsQueueAndBumpersAlwaysOverride() {
        IntakeQueueController c=new IntakeQueueController();
        assertEquals(1,c.update(false,false,false),0);
        assertEquals(-1,c.update(true,false,false),0);
        assertEquals(1,c.update(true,true,false),0);
        assertEquals(0,c.update(true,true,true),0);
        assertEquals(-1,c.update(false,false,true),0);
        assertEquals(0,c.update(false,true,true),0);
        assertEquals(-1,c.update(true,false,false),0);
        assertEquals(1,c.update(false,false,false),0);
    }
    @Test public void fullQueueShotAndReloadCycleResumesWithoutBumpers() {
        TransferController transfer=new TransferController();
        IntakeQueueController intake=new IntakeQueueController();
        transfer.update(0,true,false,false);
        transfer.update(2,true,false,false);
        assertEquals(-1,intake.update(transfer.full(),false,false),0);
        transfer.update(2.1,false,false,false); // Fourth-ball sensor clears, gate still closed.
        assertTrue(transfer.full()); assertEquals(0,transfer.stage1(),0); assertEquals(0,transfer.stage2(),0);
        assertEquals(-1,intake.update(transfer.full(),false,false),0);
        transfer.update(2.2,false,false,true); // Gate opening: keep both stopped.
        assertEquals(0,transfer.stage1(),0); assertEquals(0,transfer.stage2(),0);
        transfer.update(2.5,false,true,true); // Settled gate permits forward feeding.
        assertEquals(1,transfer.stage1(),0); assertEquals(1,transfer.stage2(),0);
        transfer.update(3,false,false,false); // Shot request released: resume collection.
        assertEquals(1,intake.update(transfer.full(),false,false),0);
        assertEquals(1,transfer.stage1(),0); assertEquals(0,transfer.stage2(),0);
    }
}
