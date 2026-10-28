package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import static org.junit.Assert.*;

public class TeleOpFieldReferenceTest {
    @Test public void cornerPoseAccountsForRearAndSideRatherThanPodLocations() {
        assertEquals(1.566319, TeleOpFieldReference.START_X_METERS, 1e-9);
        assertEquals(-1.67045, TeleOpFieldReference.START_Y_METERS, 1e-9);
        assertEquals(180, TeleOpFieldReference.START_HEADING_DEG, 0);
        assertEquals(32, PinpointOdometry.PINPOINT_X_POD_OFFSET_MM, 0);
        assertEquals(-74, PinpointOdometry.PINPOINT_Y_POD_OFFSET_MM, 0);
    }
    @Test public void nearestCellChangesAtMidlineAndAlwaysKeepsAlliance() {
        for (TurretSubsystem.Alliance a : TurretSubsystem.Alliance.values()) {
            for (double y : new double[] { -1, 1 }) {
                TurretSubsystem.HiveSide side = TeleOpFieldReference.nearestSide(y, TurretSubsystem.HiveSide.FAR);
                GoalTarget target = TeleOpFieldReference.target(a, side);
                assertEquals(a, target.alliance);
                assertTrue(y * target.y > 0);
                assertEquals(a == TurretSubsystem.Alliance.BLUE ? .32385 : -.32385, target.x, 1e-9);
                assertEquals(1.51257, target.height, 1e-9);
                assertEquals(.38322519, Math.abs(target.y), 1e-8);
                assertTrue(target.scoreable());
                assertEquals(4, target.tags.size()); // Nominal map; camera verification is separate.
                assertEquals(ShootingConfiguration.CALIBRATION_GEOMETRY_ID, target.geometryId);
            }
        }
        assertEquals(TurretSubsystem.HiveSide.AUDIENCE,
                TeleOpFieldReference.nearestSide(0, TurretSubsystem.HiveSide.AUDIENCE));
    }
    @Test public void odometryBearingUsesChassisHeadingAndTurretPivot() {
        GoalTarget g = TeleOpFieldReference.target(TurretSubsystem.Alliance.BLUE, TurretSubsystem.HiveSide.FAR);
        MotionSample pose = new MotionSample(1, g.x - ShootingConfiguration.MOUNT.pivot.x, g.y - 1,
                0, 0, 0, 0, 0, .02, true, true, "test");
        assertEquals(Math.PI, Math.abs(TeleOpFieldReference.yawRadians(pose, g, ShootingConfiguration.MOUNT)), 1e-9);
    }
    @Test public void currentPoseFieldShotCanUseInterpolatedProfileWithoutCameraOrTagMap() {
        GoalTarget g = TeleOpFieldReference.target(TurretSubsystem.Alliance.BLUE, TurretSubsystem.HiveSide.FAR);
        ShotProfile profile = new ShotProfile(g.geometryId, true,
                new ShotProfile.Row(1, 2, 3000),
                new ShotProfile.Row(3, 6, 3500));
        ShotPlanner planner = ControllerFixtures.planner(ControllerFixtures.mount());
        ShotPlanner.Solution solution = null;
        for (int i = 0; i <= 10; i++) {
            double now = 1 + i * .02;
            MotionSample pose = ControllerFixtures.pose(now, g.x - 2, g.y, 0, 0, 0, 0);
            solution = planner.solve(now, pose, g, profile);
        }
        assertTrue(solution.reason, solution.valid);
        assertEquals(3250, solution.rpm, 1e-9);
        assertEquals(4, solution.hoodDeg, 1e-9);
        assertEquals(0, solution.yaw, 1e-9);
    }
}
