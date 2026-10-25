package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import static org.junit.Assert.*;

public class TurretControllerTest {
    @Test public void directYawCommandsAndReversalsHaveNoArtificialRamp() {
        for (int hz : new int[] { 10, 50, 100 }) {
            TurretControl c = new TurretControl();
            double dt = 1.0 / hz;
            for (int i = 1; i <= hz * 5; i++) {
                double target = ((i - 1) / (hz / 2)) % 2 == 0 ? 160 : -160;
                assertEquals(target, c.updateYawCommand(i * dt, target, 0, 100, true, 0), 0);
                assertFalse(c.isBackupCorrectionActive());
            }
        }
    }
    @Test public void normalHoldNoiseAndSmallPositionErrorDoNotEngageBackup() {
        TurretControl c = new TurretControl();
        for (int i = 0; i < 500; i++) {
            assertEquals(90, c.updateYawCommand(i * .02, 90, 90 + (i % 2 == 0 ? .8 : -.8), 2, true, 0), 0);
            assertFalse(c.isBackupCorrectionActive());
        }
    }
    @Test public void backupRequiresPersistentSameDirectionErrorAtAStationaryTarget() {
        TurretControl c = new TurretControl();
        assertEquals(90, c.updateYawCommand(0, 90, 88, 0, true, 0), 0);
        assertEquals(90, c.updateYawCommand(.2, 90, 88, 0, true, 0), 0);
        // An error reversal and then a moving turret each restart qualification.
        assertEquals(90, c.updateYawCommand(.26, 90, 92, 0, true, 0), 0);
        assertEquals(90, c.updateYawCommand(.4, 90, 92, -10, true, 0), 0);
        assertEquals(90, c.updateYawCommand(.5, 90, 88, 0, true, 0), 0);
        assertEquals(90, c.updateYawCommand(.74, 90, 88, 0, true, 0), 0);
        assertTrue(c.updateYawCommand(.76, 90, 88, 0, true, 0) > 90);
        assertTrue(c.isBackupCorrectionActive());
    }
    @Test public void changingTargetsCannotQualifyBackupEvenWithSlowEncoderMotion() {
        TurretControl c = new TurretControl();
        for (int i = 0; i < 100; i++) {
            double target = i;
            assertEquals(target, c.updateYawCommand(i * .02, target, target - 2, 0, true, 0), 0);
            assertFalse(c.isBackupCorrectionActive());
        }
    }
    @Test public void backupPausesDuringRecoveryMotionThenHoldsTheCorrectedCommand() {
        TurretControl c = new TurretControl();
        c.updateYawCommand(0, 90, 88, 0, true, 0);
        double corrected = c.updateYawCommand(.26, 90, 88, 0, true, 0);
        assertTrue(corrected > 90);
        assertEquals(corrected, c.updateYawCommand(.28, 90, 89, 10, true, 0), 0);
        assertEquals(corrected, c.updateYawCommand(.3, 90, 90, 0, true, 0), 0);
        assertFalse(c.isBackupCorrectionActive());
        assertEquals(corrected, c.updateYawCommand(.6, 90, 90.2, 1, true, 0), 0);
        // Preserve the learned offset on a new move, without further feedback during that move.
        assertEquals(corrected - 90, c.updateYawCommand(.62, 0, 80, -100, true, 0), 1e-8);
        assertEquals(0, c.updateYawCommand(.64, 0, 80, -100, false, 0), 0);
        assertEquals(0, c.getYawTrimDegrees(), 0);
        assertFalse(c.isBackupCorrectionActive());
        assertEquals(0, c.updateYawCommand(.66, 0, 0, 0, true, 0), 0);
    }
    @Test public void largeOrFastMovesCannotWindUpTheHoldIntegrator() {
        for (int hz : new int[] { 50, 100 }) {
            TurretControl c = new TurretControl();
            for (int i = 0; i <= hz * 5; i++) {
                assertEquals(90, c.updateYawCommand(i / (double) hz, 90, 0, 50, true, 0), 0);
            }
            assertEquals(0, c.getYawTrimDegrees(), 0);
            c.reset();
            for (int i = 0; i <= hz * 5; i++) {
                c.update(5 + i / (double) hz, 2, 50, true, 0, -25, 25);
            }
            assertEquals(-TurretControl.YAW_KD_SECONDS * 10,
                    c.update(10.02, 0, 10, true, 0, -25, 25), 1e-8);
        }
    }
    @Test public void servoResponseWithPositionBiasSettlesWithoutCommandHunting() {
        // Synthetic internal servo response: 120 ms time constant, 300 deg/s limit, signed bias.
        // This verifies feedback behavior, not the loaded robot's physical settling time.
        for (int hz : new int[] { 50, 100 }) {
            for (double bias : new double[] { -8, -1.5, 1.5, 8 }) {
                TurretControl c = new TurretControl();
                double position = 0, velocity = 0, command = 0, heldCommand = Double.NaN;
                double dt = 1.0 / hz;
                for (int i = 0; i <= hz * 25; i++) {
                    command = c.updateYawCommand(i * dt, 90, position, velocity, true, 0);
                    velocity = RobotMath.clip((command - bias - position) / .12, 300);
                    position += velocity * dt;
                    if (i == hz * 20) heldCommand = command;
                    if (i > hz * 20) assertEquals(heldCommand, command, 1e-8);
                }
                assertEquals(90, position, TurretControl.TRIM_DEADBAND_DEG);
                assertEquals(0, velocity, 1e-8);
            }
        }
    }
    @Test public void settledEncoderNoiseKeepsTheSuccessfulServoCommand() {
        TurretControl c = new TurretControl();
        double trim = c.update(0, 1, true, 0);
        assertTrue(trim > 0);
        for (int i = 1; i <= 500; i++) {
            double noise = i % 2 == 0 ? .3 : -.3;
            assertEquals(trim, c.update(i * .02, noise, 2, true, 0, -25, 25), 0);
        }
        assertTrue(c.update(10.02, -2, true, 0) < trim);
        assertEquals(0, c.update(10.04, 0, false, 0), 0);
        assertEquals(0, c.update(10.06, 0, true, 0), 0);
    }
    @Test public void directYawAndTrimRespectBothTravelLimitsAndReset() {
        TurretControl c = new TurretControl();
        assertEquals(175, c.updateYawCommand(0, 1000, 0, 0, true, 0), 0);
        assertEquals(-175, c.updateYawCommand(.02, -1000, 0, 0, true, 0), 0);
        c.reset();
        assertEquals(0, c.getYawTrimDegrees(), 0);
        assertEquals(90, c.updateYawCommand(10, 90, 90, 0, true, 0), 0);
    }
    @Test public void delayedUpdatesCannotAccumulateMoreThanOneBoundedIntegralStep() {
        TurretControl delayed = new TurretControl(), regular = new TurretControl();
        delayed.update(0, 2, true, 0); regular.update(0, 2, true, 0);
        assertEquals(regular.update(TurretControl.MAX_UPDATE_SECONDS, 2, true, 0),
                delayed.update(10, 2, true, 0), 1e-8);
    }
    @Test public void zeroIsCapturedOnlyAfterTravelAndStableWindowEvenIfStartIsEarly() {
        TurretHomingController h = new TurretHomingController(1, .3, 4, .2);
        h.start(0); h.update(.1, 5000, 80); assertFalse(h.ready());
        h.update(.8, 3000, 48); assertFalse(h.ready());
        h.update(1.01, 100, 1); h.update(1.2, 50, .5); assertFalse(h.ready());
        h.update(1.4, 10, .1); h.update(1.71, 11, .11);
        assertTrue(h.ready()); assertEquals(11, h.zeroTicks());
        h.update(2, 1000, 10); assertEquals(11, h.zeroTicks());
    }
    @Test public void delayedMovementResetsStableWindowAndRehomeInvalidates() {
        TurretHomingController h = new TurretHomingController(1, .3, 4, .2);
        h.start(0); h.update(1, 500, 8); h.update(1.25, 300, 5); h.update(1.5, 20, .2);
        assertFalse(h.ready()); h.update(1.81, 20, .2); assertTrue(h.ready());
        h.start(2); assertFalse(h.ready()); h.update(3, 40, .4); h.update(3.31, 40, .4);
        assertTrue(h.ready()); assertEquals(40, h.zeroTicks()); h.invalidate(); assertFalse(h.ready());
    }
    @Test public void unstableEncoderTimesOutAndStationaryStallIsOnlyAnEstimate() {
        TurretHomingController h = new TurretHomingController(1, .3, 2, .2); h.start(0);
        for (double t = 0; t < 2.2; t += .1) h.update(t, (int)(t * 1000), t * 20);
        assertEquals(TurretHomingController.State.FAULT, h.state()); assertTrue(h.reason().contains("timeout"));
        h.start(3); h.update(4, 8000, 130); h.update(4.31, 8000, 130);
        assertTrue(h.ready()); assertTrue(h.reason().contains("no absolute"));
    }
    @Test public void trimManualYawAndHoodHaveEquivalentRatesAtDifferentLoopFrequencies() {
        double[] trim = new double[2], yaw = new double[2];
        for (int k = 0; k < 2; k++) {
            int hz = k == 0 ? 50 : 100; TurretControl c = new TurretControl(); c.update(0, 2, true, 0);
            for (int i = 1; i <= hz; i++) { trim[k] = c.update(i / (double)hz, 2, true, 10); yaw[k] += TurretControl.manualDelta(.5, 1.0 / hz); }
            assertEquals(0, c.hoodError(10), 1e-8); c.reset(); assertEquals(0, c.update(1, 0, false, 10), 1e-9);
        }
        assertEquals(trim[0], trim[1], 1e-8); assertEquals(30.0 / .115, yaw[0], 1e-8); assertEquals(yaw[0], yaw[1], 1e-8);
    }
    @Test public void hoodTravelEstimateAccountsForMaxPlusSpeedAndTwoToOneLinkage() {
        TurretControl c = new TurretControl();
        c.update(0, 0, false, 15);
        c.update(.03, 0, false, 15);
        assertEquals(15 - (60.0 / .115 / 2) * .03, c.hoodError(15), 1e-8);
        c.update(.06, 0, false, 15);
        assertEquals(0, c.hoodError(15), 1e-8);
    }
    @Test public void trimIsBoundedAndChangingHoodCanTrackWithoutAnUnchangedCommand() {
        TurretControl c = new TurretControl(); c.update(0, 100, true, 0);
        for (int i = 1; i <= 100; i++) assertTrue(c.update(i * .02, 100, true, i * .05) <= 25);
        assertTrue(c.hoodError(5) < .1);
    }
    @Test public void robotHeadingConventionWrapsAtFrontAndExcludesLeft() {
        assertEquals(90, TurretControl.yawToRobotHeadingDegrees(0), 1e-8);
        assertEquals(0, TurretControl.yawToRobotHeadingDegrees(90), 1e-8);
        assertEquals(180, TurretControl.yawToRobotHeadingDegrees(-90), 1e-8);
        assertEquals(270, TurretControl.yawToRobotHeadingDegrees(180), 1e-8);
        assertEquals(275, TurretControl.yawToRobotHeadingDegrees(175), 1e-8);
        assertEquals(265, TurretControl.yawToRobotHeadingDegrees(-175), 1e-8);
        for (int heading = 0; heading < 360; heading++) {
            double yaw = TurretControl.robotHeadingToYawDegrees(heading);
            assertEquals(heading, TurretControl.yawToRobotHeadingDegrees(yaw), 1e-8);
            assertEquals(heading > 265 && heading < 275, Math.abs(yaw) > 175 + 1e-8);
        }
        assertEquals(2, TurretControl.robotHeadingToYawDegrees(359)
                - TurretControl.robotHeadingToYawDegrees(1), 1e-8);
    }
    @Test public void manualReleaseCapturesEncoderInsteadOfFinishingQueuedMovement() {
        TurretControl c = new TurretControl();
        double commanded = c.manualYawTarget(0, 0, 1, .1, false);
        assertEquals(-60 / .115 * .1, commanded, 1e-8);
        double held = c.manualYawTarget(-10, commanded, 0, .02, false);
        assertEquals(-10, held, 1e-8);
        // Subsequent encoder drift must not move the held setpoint.
        assertEquals(-10, c.manualYawTarget(-12, held, .04, .02, false), 1e-8);
        TurretControl half = new TurretControl();
        assertEquals(commanded / 2, half.manualYawTarget(0, 0, .5, .1, false), 1e-8);
    }
    @Test public void autoAimOverridesStickAndHandsOffAtActualPosition() {
        TurretControl c = new TurretControl();
        c.manualYawTarget(0, 0, 1, .02, false);
        assertEquals(40, c.manualYawTarget(10, 40, -1, .02, true), 1e-8);
        assertEquals(80, c.manualYawTarget(20, 80, 1, .02, true), 1e-8);
        assertEquals(25, c.manualYawTarget(25, 80, 0, .02, false), 1e-8);
        c.manualYawTarget(30, 90, 1, .02, true);
        assertEquals(30 - TurretControl.manualDelta(.5, .02),
                c.manualYawTarget(30, 90, .5, .02, false), 1e-8);
    }
    @Test public void pidRespondsImmediatelyDampsMotionAndDoesNotWindUpAtLimits() {
        TurretControl c = new TurretControl();
        double proportional = TurretControl.YAW_KP * (10 - TurretControl.TRIM_DEADBAND_DEG);
        assertEquals(proportional, c.update(0, 10, 0, true, 0, -25, 25), 1e-8);
        assertTrue(c.update(.02, 10, 50, true, 0, -25, 25) < proportional);
        c.reset();
        for (int i = 2; i < 500; i++) {
            // Settled error qualifies for integration, but the travel limit must block accumulation.
            assertEquals(0, c.update(i * .02, 2, 0, true, 0, -25, 0), 1e-8);
        }
        assertTrue(c.update(10, -1, 0, true, 0, -25, 0) < 0);
        assertEquals(0, c.update(10.02, 100, 0, false, 0, -25, 25), 1e-8);
        assertEquals(0, c.update(10.04, 0, 0, true, 0, -25, 25), 1e-8);
        c.reset();
        for (int i = 0; i < 500; i++) {
            assertEquals(0, c.update(11 + i * .02, -2, 0, true, 0, 0, 25), 1e-8);
        }
        assertTrue(c.update(21, 1, 0, true, 0, 0, 25) > 0);
    }
    @Test public void hoodHomeIsHighestAndCounterclockwiseTravelLowersLaunchAngle() {
        assertEquals(15, TurretSubsystem.hoodTravelToLaunchAngleDegrees(0), 0);
        assertEquals(0, TurretSubsystem.hoodTravelToLaunchAngleDegrees(15), 0);
        assertEquals(-15, TurretSubsystem.hoodTravelToLaunchAngleDegrees(30), 0);
        assertEquals(.5, TurretSubsystem.pitchDegreesToServoPos(0), 1e-9);
        assertEquals(.5 - 30.0 / 355, TurretSubsystem.pitchDegreesToServoPos(15), 1e-9);
        assertEquals(.5 - 60.0 / 355, TurretSubsystem.pitchDegreesToServoPos(30), 1e-9);
        assertEquals(TurretSubsystem.pitchDegreesToServoPos(30), TurretSubsystem.pitchDegreesToServoPos(50), 0);
        assertEquals(.5, TurretSubsystem.pitchDegreesToServoPos(-10), 1e-9);
    }
    @Test public void cameraBearingWrapsIntoShooterYawFrame() {
        assertEquals(0, ShootingConfiguration.MOUNT.cameraYaw, 0);
        assertTrue(ShootingConfiguration.MOUNT.measured);
        assertEquals(-90, ShotCoordinator.visualTrackingYawDegrees(90, 0, 180), 1e-8);
        assertEquals(90, ShotCoordinator.visualTrackingYawDegrees(-90, 0, 180), 1e-8);
        assertEquals(-100, ShotCoordinator.visualTrackingYawDegrees(90, 10, 180), 1e-8);
        assertEquals(-175, ShotCoordinator.visualTrackingYawDegrees(-170, 12, 0), 1e-8);
    }
    @Test public void visualHiveTrackingUsesClockwiseCameraErrorWithoutWrappingThroughLeft() {
        assertEquals(-10, ShotCoordinator.visualTrackingYawDegrees(0, 10, 0), 1e-8);
        assertEquals(15, ShotCoordinator.visualTrackingYawDegrees(10, 0, 5), 1e-8);
        double desired = ShotCoordinator.visualTrackingYawDegrees(170, -12, 0);
        assertEquals(175, TurretSubsystem.clampYawAngle(desired), 1e-8);
    }
}
