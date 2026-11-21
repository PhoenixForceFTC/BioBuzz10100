package org.firstinspires.ftc.teamcode;

import org.junit.Test;

import static org.junit.Assert.*;

/**
 * Bench controls running the competition TeleOp shooting policy (continuous only, ported 2026-11-19; firing
 * coordinator and yaw readiness matched to competition 2026-11-21): the gate and transfer stage 1 start in the
 * same loop. The bench flywheel has no idle.
 */
public class IntakeShooterTestControllerTest {
    private static final double READY = IntakeShooterTestController.READY_STABLE_SECONDS;

    @Test public void benchUsesTheCompetitionContinuousFeedPolicy() {
        IntakeShooterTestController c = armed();
        assertTrue(c.firing().continuous());
        assertEquals(CompetitionFeedController.READY_STABLE_SECONDS, c.firing().stableSeconds, 0);
        assertEquals(ShootingConfiguration.READY_STABLE_SECONDS, READY, 0);
        assertEquals(CompetitionFeedController.GATE_TO_FEED_DELAY_SECONDS, c.firing().gateTravelSeconds, 0);
        // Same coordinator setup as competition: a manual feed opens the gate and feeds in one loop.
        c.update(1, false, false, false, false, true, false, false, false, false, false);
        assertTrue(c.firing().gateOpen());
        assertTrue(c.feedTransfer(false));
    }

    @Test public void readinessMustHoldForStableDelayThenGateAndStage1StartTogether() {
        IntakeShooterTestController c = armed();
        autoShot(c, 1, true, true);
        assertEquals(FiringCoordinator.State.PREPARING, c.firing().state());
        assertFalse(c.firing().gateOpen());
        autoShot(c, 1.1, true, true);
        assertFalse(c.firing().gateOpen());
        autoShot(c, 1.12, false, true); // Losing tight readiness restarts the stable interval.
        autoShot(c, 1.2, true, true);
        autoShot(c, 1.2 + READY - .01, true, true);
        assertFalse(c.firing().gateOpen());
        double gateCommand = 1.2 + READY + .01;
        autoShot(c, gateCommand, true, true);
        // Competition: the gate command and stage 1 start in the same loop, with collection paused.
        assertTrue(c.firing().gateOpen());
        assertTrue(c.pauseLoading());
        assertEquals(0, c.intakePower(false), 0);
        assertEquals(FiringCoordinator.State.FEEDING, c.firing().state());
        assertTrue(c.feedTransfer(false));
        assertTrue(c.feedStage1());
        assertFalse(c.closedGateStage1());
        assertEquals(1, c.firing().feedAttempts());
    }

    @Test public void competitionCoordinatorStillFeedsWithTheGateInOneLoop() {
        FiringCoordinator competition = CompetitionFeedController.firing();
        new CompetitionFeedController().updateSelection(0, competition);
        competition.update(1, false, true, false, false, false, false, false, "");
        assertTrue(competition.gateOpen());
        assertTrue(competition.feeding());
    }

    @Test public void continuousAutoFeedHasNoTimedWindowWhileFeedReadinessHolds() {
        IntakeShooterTestController c = feeding();
        autoShot(c, 10, false, true); // Tight readiness may drop during the feed; the loose check keeps it.
        assertTrue(c.firing().gateOpen());
        assertTrue(c.feedTransfer(false));
        assertEquals(1, c.firing().feedAttempts());
        assertEquals(0, c.intakePower(false), 0); // Automatic collection pauses while shooting.
        assertTrue(c.pauseLoading());
    }

    @Test public void lossOfFeedReadinessClosesGateAndRequiresRequalification() {
        IntakeShooterTestController c = feeding();
        autoShot(c, 2, true, false);
        assertEquals(FiringCoordinator.State.RECOVERY, c.firing().state());
        assertFalse(c.firing().gateOpen());
        assertFalse(c.feedTransfer(true));
        assertTrue(c.shootingBusy());
        autoShot(c, 2.01, true, true); // Zero recovery pause: back to PREPARING.
        autoShot(c, 2.02, true, true);
        autoShot(c, 2.02 + READY - .01, true, true);
        assertFalse(c.firing().gateOpen());
        autoShot(c, 2.02 + READY + .01, true, true);
        assertTrue(c.firing().gateOpen());
        assertTrue(c.feedTransfer(false)); // Requalified: gate and transfer restart together.
        assertEquals(2, c.firing().feedAttempts());
    }

    @Test public void feedCannotBypassSpinHomeOrReadiness() {
        IntakeShooterTestController c = armed();
        c.update(1, false, false, false, true, false, false, false, true, true, true); // LT without RT.
        assertFalse(c.firing().gateOpen());
        assertEquals(FiringCoordinator.State.IDLE, c.firing().state());
        assertFalse(c.pauseLoading()); // LT alone is not a shot request.
        c.update(2, false, false, true, true, false, false, false, false, true, true);
        assertEquals(FiringCoordinator.State.INHIBITED, c.firing().state());
        assertFalse(c.firing().gateOpen());
        for (double t = 3; t < 5; t += .02) autoShot(c, t, false, true);
        assertFalse(c.firing().gateOpen());
        assertFalse(c.feedTransfer(true));
        assertFalse(c.autoShotReady());
    }

    @Test public void manualFeedBypassesReadinessHomeAndFlywheelUntilReleased() {
        IntakeShooterTestController c = armed();
        c.update(1, false, false, false, false, true, false, false, false, false, false);
        assertTrue(c.firing().gateOpen());
        assertTrue(c.feedTransfer(false)); // Manual feed: gate and transfer in the same loop.
        assertTrue(c.feedStage1());
        assertTrue(c.manualFeedRequested());
        assertFalse(c.spinFlywheel());
        assertEquals(0, c.intakePower(false), 0);
        c.update(5, false, false, false, false, true, false, false, false, false, false);
        assertTrue(c.feedTransfer(false)); // No feed window in continuous mode.
        assertEquals(1, c.firing().feedAttempts());
        c.update(6, false, false, false, false, false, false, false, false, false, false);
        assertEquals(FiringCoordinator.State.IDLE, c.firing().state());
        assertFalse(c.firing().gateOpen());
        assertFalse(c.pauseLoading());
        assertEquals(1, c.intakePower(false), 0);
    }

    @Test public void inhibitWinsOverAutomaticAndManualFeedWithoutDisarming() {
        IntakeShooterTestController c = feeding();
        c.update(2, false, false, true, true, false, true, false, true, true, true);
        assertEquals(FiringCoordinator.State.INHIBITED, c.firing().state());
        assertFalse(c.firing().gateOpen());
        assertFalse(c.feedTransfer(true));
        assertTrue(c.pauseLoading());
        assertEquals(0, c.intakePower(false), 0);
        assertTrue(c.spinFlywheel()); // Inhibit blocks feeding only.
        c.update(3, false, false, false, false, true, true, false, true, true, true);
        assertFalse(c.firing().gateOpen());
        c.update(4, false, false, false, false, true, false, false, true, true, true);
        assertTrue(c.armed());
        assertTrue(c.firing().gateOpen());
        assertTrue(c.feedTransfer(false));
    }

    @Test public void acceptedServoErrorCanFeedWithoutNeedingAnUnrequestedCorrection() {
        IntakeShooterTestController c = armed();
        boolean settled = IntakeShooterTestController.isYawReady(2);
        assertTrue(settled);
        autoShot(c, 1, settled, true);
        autoShot(c, 1 + READY + .01, settled, true);
        assertTrue(c.firing().gateOpen());
        assertTrue(c.feedTransfer(false));
    }

    @Test public void benchYawCheckIsTheCompetitionAngleCheckWithNoRateLimit() {
        double tolerance = ShotReadiness.YAW_TOLERANCE_DEG;
        assertTrue(IntakeShooterTestController.isYawReady(tolerance));
        assertTrue(IntakeShooterTestController.isYawReady(-tolerance));
        assertFalse(IntakeShooterTestController.isYawReady(tolerance + .01));
        assertFalse(IntakeShooterTestController.isYawReady(-tolerance - .01));
        assertFalse(IntakeShooterTestController.isYawReady(Double.NaN));
        IntakeShooterTestController c = armed();
        autoShot(c, 1, IntakeShooterTestController.isYawReady(tolerance + 1), true);
        autoShot(c, 2, IntakeShooterTestController.isYawReady(tolerance + 1), true);
        assertFalse(c.firing().gateOpen());
    }

    @Test public void feedContinuationUsesTheCompetitionLooseRpmBand() {
        double band = TurretSubsystem.SHOOTER_LOOSE_READY_TOLERANCE_RPM;
        assertTrue(IntakeShooterTestController.isLooseReady(0, true, 3500, 3500 - band + .01));
        assertTrue(IntakeShooterTestController.isLooseReady(-ShotReadiness.YAW_TOLERANCE_DEG, true, 3500, 3500 + band - .01));
        assertFalse(IntakeShooterTestController.isLooseReady(0, true, 3500, 3500 - band - .01));
        assertFalse(IntakeShooterTestController.isLooseReady(ShotReadiness.YAW_TOLERANCE_DEG + .01, true, 3500, 3500));
        assertFalse(IntakeShooterTestController.isLooseReady(0, false, 3500, 3500));
        assertFalse(IntakeShooterTestController.isLooseReady(0, true, 0, 0));
        assertFalse(IntakeShooterTestController.isLooseReady(Double.NaN, true, 3500, 3500));
        assertFalse(IntakeShooterTestController.isLooseReady(0, true, 3500, Double.NaN));
        assertTrue(band > TurretSubsystem.SHOOTER_READY_TOLERANCE_RPM);
    }

    @Test public void rpmStepsUseCoarseHundredAndFineTenAndClampToFlywheelRange() {
        assertEquals(3600, IntakeShooterTestController.stepRpm(3500, true, false, false, false), 0);
        assertEquals(3400, IntakeShooterTestController.stepRpm(3500, false, true, false, false), 0);
        assertEquals(3510, IntakeShooterTestController.stepRpm(3500, false, false, true, false), 0);
        assertEquals(3490, IntakeShooterTestController.stepRpm(3500, false, false, false, true), 0);
        assertEquals(3590, IntakeShooterTestController.stepRpm(3500, true, false, false, true), 0);
        assertEquals(3500, IntakeShooterTestController.stepRpm(3500, true, true, true, true), 0);
        double rpm = 3500;
        for (int i = 0; i < 7; i++) rpm = IntakeShooterTestController.stepRpm(rpm, false, false, true, false);
        assertEquals(3570, rpm, 0); // Repeated fine steps do not drift.
        assertEquals(0, IntakeShooterTestController.stepRpm(5, false, false, false, true), 0);
        assertEquals(TurretSubsystem.SHOOTER_MAX_RPM,
                IntakeShooterTestController.stepRpm(TurretSubsystem.SHOOTER_MAX_RPM - 5, false, false, true, false), 0);
        assertEquals(TurretSubsystem.SHOOTER_MAX_RPM,
                IntakeShooterTestController.stepRpm(1e9, false, false, false, false), 0);
        assertEquals(10, IntakeShooterTestController.stepRpm(Double.NaN, false, false, true, false), 0);
    }

    @Test public void flywheelRunsOnlyWhileRtIsHeldWithNoBenchIdle() {
        IntakeShooterTestController c = new IntakeShooterTestController();
        c.update(0, false, false, true, false, false, false, false, true, true, true); // RT held at START.
        assertFalse(c.spinFlywheel());
        c.update(1, false, false, false, false, false, false, false, true, true, true);
        assertTrue(c.armed());
        assertFalse(c.spinFlywheel()); // Armed with RT released: off (the competition idle is not used).
        c.update(2, false, false, true, false, false, false, false, true, true, true);
        assertTrue(c.spinFlywheel());
        c.update(3, false, false, true, false, false, true, false, true, true, true);
        assertTrue(c.spinFlywheel()); // Inhibit blocks feeding only.
        c.update(4, false, false, false, false, false, false, false, true, true, true);
        assertFalse(c.spinFlywheel()); // Releasing RT turns it off, not down to an idle.
        c.update(5, false, false, true, false, false, false, true, true, true, true);
        assertFalse(c.spinFlywheel()); // Stop wins over RT.
    }

    @Test public void heldControlsAtStartCannotMoveMotorsUntilReleased() {
        IntakeShooterTestController c = new IntakeShooterTestController();
        c.update(0, true, false, true, true, true, false, false, true, true, true);
        c.update(1, true, false, true, true, true, false, false, true, true, true);
        assertFalse(c.armed());
        assertFalse(c.spinFlywheel());
        assertTrue(c.pauseLoading());
        assertFalse(c.forceStage1Loading());
        assertEquals(0, c.intakePower(false), 0);
        assertFalse(c.firing().gateOpen());
        assertFalse(c.feedTransfer(true));
        c.update(2, false, false, false, false, false, false, false, true, true, true);
        c.update(3, true, false, false, false, false, false, false, true, false, false);
        assertFalse(c.pauseLoading());
        assertTrue(c.forceStage1Loading());
        assertEquals(1, c.intakePower(false), 0);
        assertFalse(c.firing().gateOpen());
    }

    @Test public void loadingHonorsFullAndReverseOnlyRunsIntake() {
        IntakeShooterTestController c = armed();
        TransferController transfer = new TransferController();
        c.update(1, true, false, false, false, false, false, false, true, false, false);
        load(transfer, c, 1, true);
        assertEquals(1, transfer.stage1(), 0);
        assertEquals(1, c.intakePower(transfer.full()), 0);
        load(transfer, c, 1 + TransferSubsystem.FULL_CONFIRM_TIME_SECONDS + .01, true);
        assertTrue(transfer.full());
        assertEquals(1, c.intakePower(transfer.full()), 0); // RB overrides confirmed HIGH.
        assertEquals(1, transfer.stage1(), 0); // Competition RB also forces stage 1 past full.

        c.update(1.6, false, false, false, false, false, false, false, true, false, false);
        assertEquals(0, c.intakePower(transfer.full()), 0); // Release RB: automatic stop, not reverse.
        load(transfer, c, 1.61, false);
        assertFalse(transfer.full()); assertEquals(1, transfer.stage1(), 0);
        assertEquals(1, c.intakePower(transfer.full()), 0); // LOW restores automatic forward.

        c.update(1.7, false, true, false, false, false, false, false, true, false, false);
        load(transfer, c, 1.7, false);
        assertTrue(c.pauseLoading());
        assertEquals(-1, c.intakePower(true), 0);
        assertEquals(0, transfer.stage1(), 0);
        c.update(1.8, true, true, false, false, false, false, false, true, false, false);
        assertEquals(0, c.intakePower(false), 0);
        assertTrue(c.pauseLoading());
        assertFalse(c.forceStage1Loading());
    }

    @Test public void shootingPausesAutomaticCollectionButBumpersStillOverride() {
        IntakeShooterTestController c = armed();
        c.update(1, false, false, true, false, false, false, false, true, false, false); // RT alone.
        assertTrue(c.spinFlywheel());
        assertFalse(c.pauseLoading());
        assertEquals(1, c.intakePower(false), 0);
        autoShot(c, 2, false, false); // RT + LT, not yet ready.
        assertTrue(c.shootingBusy());
        assertTrue(c.pauseLoading());
        assertEquals(0, c.intakePower(false), 0);
        c.update(3, true, false, true, true, false, false, false, true, false, false);
        assertEquals(1, c.intakePower(false), 0);
    }

    @Test public void reverseDuringAFeedReversesIntakeButKeepsFeedingLikeCompetition() {
        IntakeShooterTestController c = feeding();
        c.update(2, false, true, true, true, false, false, false, true, true, true);
        assertTrue(c.firing().gateOpen());
        assertTrue(c.feedTransfer(false));
        assertEquals(-1, c.intakePower(false), 0);
    }

    @Test public void releasingThenReholdingFeedRequiresInitialReadinessAgain() {
        IntakeShooterTestController c = feeding();
        c.update(2, false, false, true, false, false, false, false, true, true, true);
        assertFalse(c.firing().gateOpen());
        autoShot(c, 2.1, true, true);
        autoShot(c, 2.1 + READY - .01, true, true);
        assertFalse(c.firing().gateOpen());
        autoShot(c, 2.1 + READY + .01, true, true);
        assertTrue(c.feedTransfer(false));
        assertEquals(2, c.firing().feedAttempts());
    }

    @Test public void lostHomeStopsAutomaticFeed() {
        IntakeShooterTestController c = feeding();
        c.update(2, false, false, true, true, false, false, false, false, true, true);
        assertEquals(FiringCoordinator.State.INHIBITED, c.firing().state());
        assertFalse(c.firing().gateOpen());
        assertFalse(c.feedTransfer(true));
    }

    @Test public void releasingEitherTriggerImmediatelyClosesGateAndStopsFeeding() {
        IntakeShooterTestController c = feeding();
        c.update(2, false, false, true, false, false, false, false, true, true, true);
        assertFalse(c.firing().gateOpen());
        assertFalse(c.feedTransfer(true));
        assertFalse(c.pauseLoading());
        c = feeding();
        c.update(2, false, false, false, true, false, false, false, true, true, true);
        assertFalse(c.spinFlywheel());
        assertFalse(c.firing().gateOpen());
        assertFalse(c.feedTransfer(true));
    }

    @Test public void stopWinsOverEveryHoldAndRequiresReleaseBeforeResuming() {
        IntakeShooterTestController c = feeding();
        c.update(2, true, true, true, true, true, false, true, true, true, true);
        assertFalse(c.armed());
        assertFalse(c.spinFlywheel());
        assertFalse(c.firing().gateOpen());
        assertFalse(c.feedTransfer(true));
        assertEquals(0, c.intakePower(false), 0);
        assertTrue(c.pauseLoading());
        c.update(3, false, false, false, false, true, false, false, true, true, true); // Manual still held.
        assertFalse(c.armed());
        assertFalse(c.firing().gateOpen());
        c.update(4, false, false, false, false, false, false, false, true, true, true);
        assertTrue(c.armed());
        c.update(5, false, false, false, false, true, false, false, true, true, true);
        assertTrue(c.firing().gateOpen());
        assertTrue(c.feedTransfer(false));
    }

    private IntakeShooterTestController armed() {
        IntakeShooterTestController c = new IntakeShooterTestController();
        c.update(0, false, false, false, false, false, false, false, true, true, true);
        assertTrue(c.armed());
        return c;
    }

    /** GP1 RT + LT held (automatic shot), homed, with the given tight/loose readiness. */
    private void autoShot(IntakeShooterTestController c, double now, boolean tight, boolean loose) {
        c.update(now, false, false, true, true, false, false, false, true, tight, loose);
    }

    private IntakeShooterTestController feeding() {
        IntakeShooterTestController c = armed();
        autoShot(c, 1, true, true);
        autoShot(c, 1 + READY + .01, true, true);
        assertTrue(c.firing().gateOpen());
        assertTrue(c.feedTransfer(false));
        return c;
    }

    private static void load(TransferController transfer, IntakeShooterTestController c, double now, boolean high) {
        transfer.update(now, high, c.feedTransfer(false), c.pauseLoading(), c.feedStage1(), c.forceStage1Loading());
    }
}
