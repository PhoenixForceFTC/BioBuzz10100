package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import java.util.Arrays;
import java.util.List;
import static org.junit.Assert.*;

/**
 * Raw Limelight readings copied from Driver Station photos (2026-11-05, Test: Limelight Goal State 3D,
 * Red / Far, raised tags 30-33 visible, robot stationary). SDK roll/pitch/yaw are in degrees.
 * Before the convention fix every tag failed only the facing check with a score near -1.
 */
public class PhysicalLimelightFrameTest {
    private static final double[][] RAW = {
            // id, right, down, forward (m), roll, pitch, yaw (deg), area (%)
            {30, .186, -.016, 1.117,   5.2,  0.1, -179.6, .007},
            {31, .091, -.017, 1.119,  11.5,  0.8, -179.8, .006},
            {32, -.048, -.017, 1.120, -11.6,  7.4, -179.9, .007},
            {33, -.142, -.017, 1.119,   9.2, -4.0, -179.4, .006},
    };
    private static TagObservation converted(double[] r) {
        return new TagObservation((int) r[0], r[1], r[2], r[3],
                LimelightObservation.robotTagRoll(Math.toRadians(r[4])), Math.toRadians(r[5]), Math.toRadians(r[6]), r[7]);
    }
    private static TagObservation unconverted(double[] r) {
        return new TagObservation((int) r[0], r[1], r[2], r[3],
                Math.toRadians(r[4]), Math.toRadians(r[5]), Math.toRadians(r[6]), r[7]);
    }

    @Test public void physicalFrameFacesTheCameraOnlyAfterTheConventionConversion() {
        assertTrue(LimelightObservation.LIMELIGHT_TAG_Z_INTO_PRINT);
        for (double[] r : RAW) {
            assertTrue(unconverted(r).facingCosine() < -.9);
            TagObservation tag = converted(r);
            assertTrue(tag.id + " facing " + tag.facingCosine(), tag.facingCosine() > .9);
            assertNull(tag.id + ": " + tag.rejectionReason(), tag.rejectionReason());
        }
    }

    @Test public void redFarTagOrderAndClusterCenterMatchThePhysicalFrame() {
        // Camera-right decreases from tag 30 to tag 33 in the photographed frame.
        for (int i = 1; i < RAW.length; i++) assertTrue(RAW[i - 1][1] > RAW[i][1]);
        // Robot below the far cell looks toward -Y, so camera-right is field -X: tag 30 has the lowest X.
        GoalTarget.Tag[] map = HiveTagGeometry.nominalRaisedTags(TurretSubsystem.Alliance.RED, TurretSubsystem.HiveSide.FAR);
        for (int i = 1; i < map.length; i++) assertTrue(map[i - 1].id + " vs " + map[i].id, map[i - 1].x < map[i].x);
        // Each tag's own +X axis and sticker offset point back to one shared cluster center.
        double[] first = null;
        for (double[] r : RAW) {
            TagObservation tag = converted(r);
            double[] offset = tag.rotate(-OpeningGeometry.tagOffsetMeters(tag.id), 0, 0);
            double[] center = {tag.x + offset[0], tag.y + offset[1], tag.z + offset[2]};
            if (first == null) first = center;
            assertEquals(tag.id + " right", first[0], center[0], .03);
            assertEquals(tag.id + " forward", first[2], center[2], .03);
        }
    }

    @Test public void reportDownstreamGeometryForThePhysicalFrame() {
        // Diagnostic only: prints what the next checks see. Tolerances are not asserted here
        // because these readings came from separate photos and the bench placement is unknown.
        ShooterMount mount = ShootingConfiguration.MOUNT;
        List<TagObservation> tags = Arrays.asList(converted(RAW[0]), converted(RAW[1]), converted(RAW[2]), converted(RAW[3]));
        for (TagObservation tag : tags) {
            double[] n = tag.rotate(0, 0, 1);
            double up = TagObservation.level(n[0], n[1], n[2], mount.cameraPitch)[2];
            System.out.printf(java.util.Locale.US, "PHYSICAL tag %d height %.3f m plane %.1f deg normalUp %.3f | raised check: %s%n",
                    tag.id, mount.cameraHeight + tag.levelPosition(mount)[2], tag.planeDegrees(mount), up,
                    TeleOpFieldReference.raisedTagRejectionReason(tag, mount));
        }
        GoalStateObserver observer = new GoalStateObserver(mount);
        observer.select(TurretSubsystem.Alliance.RED, TurretSubsystem.HiveSide.FAR);
        observer.update(1.0, 1, .95, tags, true);
        System.out.println("PHYSICAL goal observer: " + observer.reason());
    }
}
