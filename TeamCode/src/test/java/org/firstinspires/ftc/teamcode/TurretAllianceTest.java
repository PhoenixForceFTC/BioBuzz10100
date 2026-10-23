package org.firstinspires.ftc.teamcode;

import org.junit.Test;

import java.util.Collections;

import static org.junit.Assert.*;

public class TurretAllianceTest {
    @Test public void everyGoalTagMapsOnlyToItsOwnAllianceAndSide() {
        for (int id = 30; id <= 45; id++) {
            boolean red = id <= 37;
            TurretSubsystem.Alliance alliance = red
                    ? TurretSubsystem.Alliance.RED : TurretSubsystem.Alliance.BLUE;
            TurretSubsystem.Alliance opponent = red
                    ? TurretSubsystem.Alliance.BLUE : TurretSubsystem.Alliance.RED;
            TurretSubsystem.HiveSide side = id <= 33 || id >= 42
                    ? TurretSubsystem.HiveSide.FAR : TurretSubsystem.HiveSide.AUDIENCE;
            assertEquals("Tag " + id, side, TurretSubsystem.getHiveSideForTag(alliance, id));
            assertNull("Opponent tag " + id, TurretSubsystem.getHiveSideForTag(opponent, id));
        }
    }

    @Test public void unrelatedTagsAndMissingAllianceAreRejected() {
        for (TurretSubsystem.Alliance alliance : TurretSubsystem.Alliance.values()) {
            for (int id : new int[] {-1, 0, 20, 29, 46, 100}) {
                assertNull(TurretSubsystem.getHiveSideForTag(alliance, id));
            }
        }
        assertNull(TurretSubsystem.getHiveSideForTag(null, 30));
        assertNull(TurretSubsystem.selectAllianceFiducial(TurretSubsystem.Alliance.RED, null));
        assertNull(TurretSubsystem.selectAllianceFiducial(
                TurretSubsystem.Alliance.BLUE, Collections.emptyList()));
    }

    @Test public void gateHomeIsCenteredAndOpenTravelIsNinetyDegrees() {
        assertEquals(0.5, GateSubsystem.CLOSED_ANGLE_DEG / GateSubsystem.SERVO_TRAVEL_DEG, 1e-9);
        assertEquals(-90.0, GateSubsystem.OPEN_ANGLE_DEG - GateSubsystem.CLOSED_ANGLE_DEG, 1e-9);
        assertEquals(0.2, GateSubsystem.OPEN_ANGLE_DEG / GateSubsystem.SERVO_TRAVEL_DEG, 1e-9);
        assertEquals(com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD, GateSubsystem.SERVO_DIRECTION);
    }

    @Test public void depollinatorDeploysClockwiseFromThirtyDegreeUpPosition() {
        assertEquals(30.0 / 355.0, DepollinatorSubsystem.HOME_ANGLE_DEG / DepollinatorSubsystem.SERVO_TRAVEL_DEG, 1e-9);
        assertEquals(135.0, DepollinatorSubsystem.DEPLOYED_ANGLE_DEG - DepollinatorSubsystem.HOME_ANGLE_DEG, 1e-9);
        assertEquals(165.0 / 355.0, DepollinatorSubsystem.DEPLOYED_ANGLE_DEG / DepollinatorSubsystem.SERVO_TRAVEL_DEG, 1e-9);
        assertEquals(com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD, DepollinatorSubsystem.SERVO_DIRECTION);
    }
}
