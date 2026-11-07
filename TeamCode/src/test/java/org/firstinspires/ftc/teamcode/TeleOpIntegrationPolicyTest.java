package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import static org.junit.Assert.*;
import static org.firstinspires.ftc.teamcode.ControllerFixtures.*;
import static org.firstinspires.ftc.teamcode.RobotStatusLights.Color.*;

public class TeleOpIntegrationPolicyTest {
    @Test public void firingPauseStopsAutomaticIntakeButRetainsExplicitOverrides() {
        IntakeQueueController c = new IntakeQueueController();
        for (boolean full : new boolean[]{false, true}) {
            assertEquals(0, c.update(full, false, false, true), 0);
            assertEquals(1, c.update(full, true, false, true), 0);
            assertEquals(-1, c.update(full, false, true, true), 0);
            assertEquals(0, c.update(full, true, true, true), 0);
        }
        assertEquals(1, c.update(false, false, false, false), 0);
        assertEquals(-1, c.update(true, false, false, false), 0);
    }
    @Test public void trimsRemainBoundedAndResetWithoutChangingCalibrationValidity() {
        AutoAimAdjustments a = new AutoAimAdjustments();
        for (int i = 0; i < 100; i++) a.adjust(1, .1, 1, 1);
        assertEquals(-10, a.yawDeg(), 0); assertEquals(5, a.hoodDeg(), 0); assertEquals(500, a.rpm(), 0);
        ShotPlanner.Solution base = planner(mount()).solve(1, pose(1,0,0,0,0,0,0), goal(2,0), profile());
        ShotPlanner.Solution adjusted = base.withOffsets(a.yawDeg(), a.hoodDeg(), a.rpm());
        assertTrue(adjusted.valid);
        assertEquals(base.yaw - Math.toRadians(10), adjusted.yaw, 1e-9);
        assertEquals(base.hoodDeg + 5, adjusted.hoodDeg, 1e-9);
        assertEquals(base.rpm + 500, adjusted.rpm, 1e-9);
        assertFalse(base.inhibit("Unmeasured").withOffsets(1, 1, 50).valid);
        assertFalse(ShotPlanner.Solution.invalid("Uncalibrated").withOffsets(0,0,0).aimable);
        for (double[] trim : new double[][]{{180,0,0},{0,40,0},{0,0,-6000},{0,0,6000},{Double.NaN,0,0}})
            assertFalse(base.withOffsets(trim[0],trim[1],trim[2]).valid);
        a.reset(); assertEquals(0, a.yawDeg(), 0); assertEquals(0, a.hoodDeg(), 0); assertEquals(0, a.rpm(), 0);
    }
    @Test public void lightsDistinguishManualBypassInhibitAndAutomaticReadiness() {
        assertEquals(GREEN, RobotStatusLights.shooterColor(0, FiringCoordinator.State.IDLE, false, true, true));
        assertEquals(YELLOW, RobotStatusLights.shooterColor(0, FiringCoordinator.State.PREPARING, false, false, true));
        assertEquals(RED, RobotStatusLights.shooterColor(0, FiringCoordinator.State.INHIBITED, true, true, true));
        assertEquals(VIOLET, RobotStatusLights.shooterColor(.1, FiringCoordinator.State.FEEDING, true, false, true));
        assertEquals(OFF, RobotStatusLights.shooterColor(.3, FiringCoordinator.State.FEEDING, true, false, true));
        assertEquals(VIOLET, RobotStatusLights.shooterColor(.3, FiringCoordinator.State.FEEDING, false, true, true));
        assertEquals(GREEN, RobotStatusLights.intakeColor(0, true, false, false, -1, false));
        assertEquals(BLUE, RobotStatusLights.intakeColor(0, true, true, false, 1, false));
        assertEquals(OFF, RobotStatusLights.intakeColor(0, true, true, true, 0, false));
    }
}
