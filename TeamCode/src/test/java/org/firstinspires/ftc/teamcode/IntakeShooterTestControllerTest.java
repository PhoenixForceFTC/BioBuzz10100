package org.firstinspires.ftc.teamcode;

import org.junit.Test;

import static org.junit.Assert.*;

public class IntakeShooterTestControllerTest {
    private static final double GATE_OPEN_AT = 1 + ShootingConfiguration.READY_STABLE_SECONDS + .01;
    private static final double FEED_AT = GATE_OPEN_AT + ShootingConfiguration.GATE_TO_FEED_DELAY_SECONDS + .01;

    @Test public void heldTriggersRepeatTimedFeedsWithRecoveryAndReadinessBetweenShots() {
        IntakeShooterTestController c = feeding();
        TransferController transfer = new TransferController();
        double feedEnds = FEED_AT + ShootingConfiguration.FEED_WINDOW_SECONDS;
        requestShot(c, feedEnds - .01, true);
        assertTrue(c.feedTransfer(true));
        double recoveryAt = feedEnds + .01;
        requestShot(c, recoveryAt, true);
        assertFalse(c.firing().gateOpen());
        assertFalse(c.feedTransfer(true));
        transfer.update(recoveryAt, true, c.feedTransfer(true), !c.loadIndex());
        assertEquals(0, transfer.stage1(), 0);
        assertEquals(0, transfer.stage2(), 0);
        requestShot(c, recoveryAt + ShootingConfiguration.RECOVERY_SECONDS - .01, true);
        assertEquals(FiringCoordinator.State.RECOVERY, c.firing().state());
        double recoveredAt = recoveryAt + ShootingConfiguration.RECOVERY_SECONDS + .01;
        requestShot(c, recoveredAt, true);
        requestShot(c, recoveredAt + .01, false);
        requestShot(c, recoveredAt + .2, false);
        assertEquals(1, c.firing().feedAttempts());
        assertFalse(c.firing().gateOpen());
        double readyAt = recoveredAt + .3;
        requestShot(c, readyAt, true);
        double reopenedAt = readyAt + ShootingConfiguration.READY_STABLE_SECONDS + .01;
        requestShot(c, reopenedAt, true);
        assertTrue(c.firing().gateOpen());
        assertFalse(c.feedTransfer(true));
        requestShot(c, reopenedAt + ShootingConfiguration.GATE_TO_FEED_DELAY_SECONDS + .01, true);
        assertTrue(c.feedTransfer(true));
        assertEquals(2, c.firing().feedAttempts());
    }

    @Test public void acceptedServoErrorCanFeedWithoutNeedingAnUnrequestedCorrection() {
        IntakeShooterTestController c = armed();
        // The servo controller accepts this 2-degree error without correcting it.
        boolean settled = IntakeShooterTestController.isYawSettled(2, 0);
        assertTrue(settled);
        requestShot(c, 1, settled);
        requestShot(c, GATE_OPEN_AT, settled);
        requestShot(c, FEED_AT, settled);
        assertTrue(c.firing().gateOpen());
        assertTrue(c.feedTransfer(true));
        assertFalse(c.feedTransfer(false));
    }

    @Test public void benchYawCheckRejectsExcessErrorMotionAndInvalidReadings() {
        assertTrue(IntakeShooterTestController.isYawSettled(4, -3));
        assertTrue(IntakeShooterTestController.isYawSettled(-4, 3));
        assertFalse(IntakeShooterTestController.isYawSettled(4.01, 0));
        assertFalse(IntakeShooterTestController.isYawSettled(-4.01, 0));
        assertFalse(IntakeShooterTestController.isYawSettled(0, 3.01));
        assertFalse(IntakeShooterTestController.isYawSettled(0, -3.01));
        assertFalse(IntakeShooterTestController.isYawSettled(Double.NaN, 0));
        assertFalse(IntakeShooterTestController.isYawSettled(0, Double.POSITIVE_INFINITY));
        IntakeShooterTestController c = armed();
        requestShot(c, 1, IntakeShooterTestController.isYawSettled(5, 0));
        requestShot(c, 2, IntakeShooterTestController.isYawSettled(5, 0));
        assertFalse(c.firing().gateOpen());
        assertFalse(c.feedTransfer(true));
    }

    private IntakeShooterTestController armed() {
        IntakeShooterTestController c = new IntakeShooterTestController();
        c.update(0, false, false, false, false, false, true, true);
        assertTrue(c.armed());
        return c;
    }

    private void requestShot(IntakeShooterTestController c, double now, boolean ready) {
        c.update(now, false, false, true, true, false, true, ready);
    }

    private IntakeShooterTestController feeding() {
        IntakeShooterTestController c = armed();
        requestShot(c, 1, true);
        requestShot(c, GATE_OPEN_AT, true);
        assertTrue(c.firing().gateOpen());
        requestShot(c, FEED_AT, true);
        assertTrue(c.feedTransfer(true));
        return c;
    }

    @Test public void heldControlsAtStartCannotMoveMotorsUntilReleased() {
        IntakeShooterTestController c = new IntakeShooterTestController();
        c.update(0, true, false, true, true, false, true, true);
        c.update(1, true, false, true, true, false, true, true);
        assertFalse(c.armed());
        assertFalse(c.spinFlywheel());
        assertFalse(c.loadIndex());
        assertEquals(0, c.intakePower(false), 0);
        assertFalse(c.firing().gateOpen());
        assertFalse(c.feedTransfer(true));
        c.update(2, false, false, false, false, false, true, true);
        c.update(3, true, false, false, false, false, true, false);
        assertTrue(c.loadIndex());
        assertEquals(1, c.intakePower(false), 0);
        assertFalse(c.firing().gateOpen());
    }

    @Test public void loadingHonorsFullAndReverseOnlyRunsIntake() {
        IntakeShooterTestController c = armed();
        TransferController transfer = new TransferController();
        c.update(1, true, false, false, false, false, true, false);
        transfer.update(1, true, false, !c.loadIndex());
        assertEquals(1, transfer.stage1(), 0);
        assertEquals(0, transfer.stage2(), 0);
        assertEquals(1, c.intakePower(transfer.full()), 0);
        transfer.update(3.01, true, false, !c.loadIndex());
        assertTrue(transfer.full());
        assertEquals(-1, c.intakePower(transfer.full()), 0);
        assertEquals(0, transfer.stage1(), 0);
        assertEquals(0, transfer.stage2(), 0);

        c.update(3.1, false, true, false, false, false, true, false);
        transfer.update(3.1, false, false, !c.loadIndex());
        assertFalse(c.loadIndex());
        assertEquals(-1, c.intakePower(true), 0);
        assertEquals(0, transfer.stage1(), 0);
        assertEquals(0, transfer.stage2(), 0);
        c.update(3.2, true, true, false, false, false, true, false);
        assertEquals(0, c.intakePower(false), 0);
        assertFalse(c.loadIndex());
    }

    @Test public void feedCannotBypassSpinHomeOrReadiness() {
        IntakeShooterTestController c = armed();
        c.update(1, true, false, false, true, false, true, true);
        assertFalse(c.firing().gateOpen());
        assertFalse(c.loadIndex());
        assertEquals(0, c.intakePower(false), 0);
        c.update(2, false, false, true, true, false, false, true);
        assertTrue(c.spinFlywheel());
        assertFalse(c.firing().gateOpen());
        requestShot(c, 3, false);
        requestShot(c, 4, false);
        assertFalse(c.firing().gateOpen());
        assertFalse(c.feedTransfer(true));
    }

    @Test public void readinessMustBeContinuousAndGateDelayMustElapseBeforeBothStagesRun() {
        IntakeShooterTestController c = armed();
        requestShot(c, 1, true);
        requestShot(c, 1.1, false);
        requestShot(c, 1.2, true);
        requestShot(c, 1.3, true);
        assertFalse(c.firing().gateOpen());
        requestShot(c, 1.36, true);
        assertTrue(c.firing().gateOpen());
        assertFalse(c.feedTransfer(true));
        double settledAt = 1.36 + ShootingConfiguration.GATE_TO_FEED_DELAY_SECONDS;
        requestShot(c, settledAt - .01, true);
        assertFalse(c.feedTransfer(true));
        requestShot(c, settledAt + .01, true);
        assertFalse(c.feedTransfer(false));
        assertTrue(c.feedTransfer(true));
        TransferController transfer = new TransferController();
        transfer.update(settledAt + .01, true, c.feedTransfer(true), !c.loadIndex());
        assertEquals(1, transfer.stage1(), 0);
        assertEquals(1, transfer.stage2(), 0);
        assertEquals(0, c.intakePower(false), 0);
    }

    @Test public void lossOfSpeedDuringTimedFeedClosesGateAndRequiresRecovery() {
        IntakeShooterTestController c = feeding();
        requestShot(c, FEED_AT + .01, false);
        assertFalse(c.firing().gateOpen());
        assertFalse(c.feedTransfer(true));
        assertEquals(FiringCoordinator.State.RECOVERY, c.firing().state());
        requestShot(c, 3, false);
        assertFalse(c.firing().gateOpen());
        assertFalse(c.feedTransfer(true));
        assertTrue(c.spinFlywheel());
        assertEquals(FiringCoordinator.State.PREPARING, c.firing().state());
    }

    @Test public void releasingThenReholdingFeedRequiresInitialReadinessAgain() {
        IntakeShooterTestController c = feeding();
        c.update(FEED_AT + .01, false, false, true, false, false, true, true);
        requestShot(c, 2, false);
        assertFalse(c.firing().gateOpen());
        assertFalse(c.feedTransfer(true));
        requestShot(c, 3, true);
        requestShot(c, 3.16, true);
        assertTrue(c.firing().gateOpen());
        assertFalse(c.feedTransfer(true));
        requestShot(c, 3.16 + ShootingConfiguration.GATE_TO_FEED_DELAY_SECONDS + .01, true);
        assertTrue(c.feedTransfer(true));
    }

    @Test public void lostHomeStillStopsTimedFeed() {
        IntakeShooterTestController c = feeding();
        c.update(FEED_AT + .01, false, false, true, true, false, false, true);
        assertFalse(c.firing().gateOpen());
        assertFalse(c.feedTransfer(true));
    }

    @Test public void releasingEitherTriggerImmediatelyClosesGateAndStopsFeeding() {
        IntakeShooterTestController c = feeding();
        c.update(FEED_AT + .01, false, false, true, false, false, true, true);
        assertFalse(c.firing().gateOpen());
        assertFalse(c.feedTransfer(true));
        assertTrue(c.spinFlywheel());
        c = feeding();
        c.update(FEED_AT + .01, false, false, false, true, false, true, true);
        assertFalse(c.firing().gateOpen());
        assertFalse(c.feedTransfer(true));
        assertFalse(c.spinFlywheel());
    }

    @Test public void stopWinsOverEveryHoldAndRequiresReleaseBeforeResuming() {
        IntakeShooterTestController c = feeding();
        c.update(FEED_AT + .01, true, true, true, true, true, true, true);
        assertFalse(c.armed());
        assertFalse(c.spinFlywheel());
        assertFalse(c.loadIndex());
        assertEquals(0, c.intakePower(false), 0);
        assertFalse(c.firing().gateOpen());
        c.update(2, true, false, true, true, false, true, true);
        assertFalse(c.armed());
        assertFalse(c.spinFlywheel());
        assertFalse(c.feedTransfer(true));
        c.update(3, false, false, false, false, false, true, true);
        assertTrue(c.armed());
    }

    @Test public void reverseCancelsAnActiveFeedEvenWhenTriggersStayHeld() {
        IntakeShooterTestController c = feeding();
        c.update(FEED_AT + .01, false, true, true, true, false, true, true);
        assertFalse(c.firing().gateOpen());
        assertFalse(c.feedTransfer(true));
        assertFalse(c.loadIndex());
        assertEquals(-1, c.intakePower(false), 0);
    }
}
