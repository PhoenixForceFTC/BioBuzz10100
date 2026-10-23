package org.firstinspires.ftc.teamcode;

import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.*;

public class PollenNavigatorTest {
    private static final long START = 1_000_000_000L;
    private PollenNavigator navigator;

    private static long at(double seconds) { return START + (long) (seconds * 1e9); }

    /** Synthesizes the image ray for a floor point, independently of the projection implementation. */
    private static PollenTarget target(long timestamp, double forwardMm, double rightMm) {
        double pitch = Math.toRadians(10.0);
        double opticalZ = forwardMm * Math.cos(pitch) + 180.0 * Math.sin(pitch);
        double imageDown = -forwardMm * Math.sin(pitch) + 180.0 * Math.cos(pitch);
        double x = 160.0 + 280.0 * rightMm / opticalZ;
        double y = 120.0 + 280.0 * imageDown / opticalZ;
        return PollenTarget.fromFloorContact(timestamp, x, y - 10.0, x, y, 150.0,
                280, 280, 160, 120, 180, 10, 0, 0);
    }

    private void update(double seconds, PollenTarget target, double x, double y, double heading,
                        boolean ready, boolean full) {
        navigator.update(at(seconds), target, x, y, heading, ready, full);
    }

    private void assertStopped() {
        assertEquals(0.0, navigator.getAxialPower(), 0.0);
        assertEquals(0.0, navigator.getYawPower(), 0.0);
        assertEquals(0.0, navigator.getIntakePower(), 0.0);
    }

    @Before public void start() {
        navigator = new PollenNavigator();
        navigator.start(START, 0, 0);
    }

    @Test public void noMotionBeforeStartOrHardwareReadiness() {
        navigator = new PollenNavigator();
        update(0, target(START, 1000, 0), 0, 0, 0, true, false);
        assertStopped();
        navigator.start(START, 0, 0);
        update(0, null, 0, 0, 0, false, false);
        assertStopped();
        assertEquals(PollenNavigator.State.WAITING_FOR_HARDWARE, navigator.getState());
        update(4.1, null, 0, 0, 0, false, false);
        assertEquals(PollenNavigator.State.FAULT, navigator.getState());
    }

    @Test public void boundedSearchRotatesWithoutTranslation() {
        update(0, null, 0, 0, 0, true, false);
        assertEquals(PollenNavigator.State.SEARCHING, navigator.getState());
        assertEquals(0.0, navigator.getAxialPower(), 0.0);
        assertTrue(navigator.getYawPower() > 0.0);
        update(0.2, null, 0, 0, -91, true, false);
        assertEquals(PollenNavigator.State.TIMED_OUT, navigator.getState());
        assertStopped();
    }

    @Test public void searchTimesOutEvenIfRobotCannotRotate() {
        update(0, null, 0, 0, 0, true, false);
        update(4.1, null, 0, 0, 0, true, false);
        assertEquals(PollenNavigator.State.TIMED_OUT, navigator.getState());
        assertStopped();
    }

    @Test public void turnsTowardOffAxisTargetBeforeDriving() {
        update(0, target(START, 700, 350), 0, 0, 0, true, false);
        assertEquals(0.0, navigator.getAxialPower(), 0.0);
        assertTrue(navigator.getYawPower() > 0.0);
        assertTrue(navigator.getYawPower() <= PollenNavigator.MAX_YAW_POWER);
        update(0.1, target(at(0.1), 700, -350), 0, 0, 0, true, false);
        assertTrue(navigator.getYawPower() < 0.0);
    }

    @Test public void alignedApproachSlowsNearPollen() {
        update(0, target(START, 1200, 0), 0, 0, 0, true, false);
        double farPower = navigator.getAxialPower();
        assertEquals(PollenNavigator.MAX_DRIVE_POWER, farPower, 1e-9);
        update(0.1, target(at(0.1), 500, 0), 50, 0, 0, true, false);
        assertTrue(navigator.getAxialPower() > 0.0);
        assertTrue(navigator.getAxialPower() < farPower);
    }

    @Test public void missingOrStaleTargetImmediatelyStopsThenSearches() {
        update(0, target(START, 1000, 0), 0, 0, 0, true, false);
        update(0.21, target(START, 1000, 0), 20, 0, 0, true, false);
        assertStopped();
        update(0.6, null, 20, 0, 0, true, false);
        assertEquals(PollenNavigator.State.SEARCHING, navigator.getState());
        assertStopped();
    }

    @Test public void targetReacquisitionResumesApproach() {
        update(0, target(START, 900, 0), 0, 0, 0, true, false);
        update(0.1, null, 20, 0, 0, true, false);
        update(0.2, target(at(0.2), 800, 0), 20, 0, 0, true, false);
        assertEquals(PollenNavigator.State.APPROACHING, navigator.getState());
        assertTrue(navigator.getAxialPower() > 0.0);
    }

    @Test public void pickupRequiresSustainedCloseAlignedDetection() {
        update(0, target(START, 340, 0), 0, 0, 0, true, false);
        update(0.05, null, 0, 0, 0, true, false);
        update(0.15, target(at(0.15), 340, 0), 0, 0, 0, true, false);
        assertEquals(PollenNavigator.State.APPROACHING, navigator.getState());
        update(0.23, target(at(0.23), 340, 0), 0, 0, 0, true, false);
        update(0.3, target(at(0.3), 340, 0), 0, 0, 0, true, false);
        assertEquals(PollenNavigator.State.COLLECTING, navigator.getState());
    }

    private void beginPickup() {
        update(0, target(START, 340, 0), 0, 0, 0, true, false);
        update(0.07, target(at(0.07), 340, 0), 0, 0, 0, true, false);
        update(0.15, target(at(0.15), 340, 0), 0, 0, 0, true, false);
        assertEquals(PollenNavigator.State.COLLECTING, navigator.getState());
    }

    @Test public void pickupSurvivesOcclusionAndStopsAtDistanceLimit() {
        beginPickup();
        update(0.2, null, 30, 0, 5, true, false);
        assertEquals(PollenNavigator.PICKUP_POWER, navigator.getAxialPower(), 0.0);
        assertTrue(navigator.getYawPower() > 0.0); // Correct CCW heading error with CW power.
        update(0.4, null, 201, 0, 0, true, false);
        assertEquals(PollenNavigator.State.SETTLING, navigator.getState());
        assertEquals(0.0, navigator.getAxialPower(), 0.0);
        assertEquals(1, navigator.getPickupAttempts());
        update(0.81, null, 201, 0, 0, true, false);
        assertEquals(PollenNavigator.State.SEARCHING, navigator.getState());
        assertStopped();
    }

    @Test public void pickupTimesOutWithStalledDrive() {
        beginPickup();
        update(1.06, null, 0, 0, 0, true, false);
        assertEquals(PollenNavigator.State.SETTLING, navigator.getState());
        assertEquals(0.0, navigator.getAxialPower(), 0.0);
    }

    @Test public void headingHoldHandlesPinpointWrap() {
        update(0, target(START, 340, 0), 0, 0, 179, true, false);
        update(0.07, target(at(0.07), 340, 0), 0, 0, 179, true, false);
        update(0.15, target(at(0.15), 340, 0), 0, 0, 179, true, false);
        update(0.2, null, 0, 0, -179, true, false);
        assertEquals(2 * PollenNavigator.YAW_KP_PER_DEG, navigator.getYawPower(), 1e-9);
    }

    @Test public void fullQueueStopsAndCannotRestart() {
        beginPickup();
        update(0.2, null, 20, 0, 0, true, true);
        assertEquals(PollenNavigator.State.COMPLETE, navigator.getState());
        assertStopped();
        update(0.3, target(at(0.3), 800, 0), 20, 0, 0, true, false);
        assertStopped();
    }

    @Test public void cameraOrOdometryFailureStopsAllCommands() {
        beginPickup();
        update(0.2, null, 0, 0, 0, false, false);
        assertEquals(PollenNavigator.State.FAULT, navigator.getState());
        assertStopped();
    }

    @Test public void invalidPoseStopsAllCommands() {
        update(0, target(START, 1000, 0), Double.NaN, 0, 0, true, false);
        assertEquals(PollenNavigator.State.FAULT, navigator.getState());
        assertStopped();
    }

    @Test public void totalPathLengthAndAutoClockBoundTheRoutine() {
        update(0, target(START, 1000, 0), 0, 0, 0, true, false);
        update(0.1, target(at(0.1), 1000, 0), 1300, 0, 0, true, false);
        update(0.2, target(at(0.2), 1000, 0), 0, 0, 0, true, false);
        assertEquals(PollenNavigator.State.COMPLETE, navigator.getState());
        assertStopped();
        navigator.start(START, 0, 0);
        update(28.1, target(at(28.1), 1000, 0), 0, 0, 0, true, false);
        assertEquals(PollenNavigator.State.TIMED_OUT, navigator.getState());
        assertStopped();
    }

    @Test public void stalledApproachTimesOut() {
        update(0, target(START, 800, 0), 0, 0, 0, true, false);
        update(7.1, target(at(7.1), 800, 0), 0, 0, 0, true, false);
        assertEquals(PollenNavigator.State.TIMED_OUT, navigator.getState());
        assertStopped();
    }

    @Test public void rereadingOneCameraFrameCannotConfirmPickup() {
        PollenTarget singleFrame = target(START, 340, 0);
        update(0, singleFrame, 0, 0, 0, true, false);
        update(0.07, singleFrame, 0, 0, 0, true, false);
        update(0.15, singleFrame, 0, 0, 0, true, false);
        assertEquals(PollenNavigator.State.APPROACHING, navigator.getState());
        assertEquals(0.0, navigator.getAxialPower(), 0.0);
        update(0.16, target(at(0.16), 340, 0), 0, 0, 0, true, false);
        update(0.18, target(at(0.18), 340, 0), 0, 0, 0, true, false);
        assertEquals(PollenNavigator.State.COLLECTING, navigator.getState());
    }

    @Test public void startupCalibrationPoseDoesNotPoisonTravelLimit() {
        navigator.start(START, Double.NaN, Double.NaN);
        update(0, null, Double.NaN, Double.NaN, Double.NaN, false, false);
        assertEquals(PollenNavigator.State.WAITING_FOR_HARDWARE, navigator.getState());
        assertStopped();
        update(1, target(at(1), 1000, 0), 0, 0, 0, true, false);
        assertEquals(0.0, navigator.getTotalTravelMm(), 0.0);
        update(1.1, target(at(1.1), 1000, 0), 2401, 0, 0, true, false);
        assertEquals(PollenNavigator.State.COMPLETE, navigator.getState());
        assertStopped();
    }
}
