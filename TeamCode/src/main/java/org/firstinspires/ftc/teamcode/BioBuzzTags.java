package org.firstinspires.ftc.teamcode;

import java.util.List;

//usage: BioBuzz AprilTag IDs and helpers for picking which tag to aim at.
// IDs come from the SDK's AprilTagGameDatabase.getBioBuzzTagLibrary():
// each structure has a cluster of 4 tags in a row (L1, L2, R1, R2), 3.25in each,
// spaced at x = -6.50, -2.75, +2.75, +6.50 inches from the cluster center.

public class BioBuzzTags {

    public static final int RED_SCORING_FIRST = 30;   // 30-33
    public static final int RED_AUDIENCE_FIRST = 34;  // 34-37
    public static final int BLUE_AUDIENCE_FIRST = 38; // 38-41
    public static final int BLUE_SCORING_FIRST = 42;  // 42-45

    public static boolean inCluster(int id, int firstId) {
        return id >= firstId && id < firstId + 4;
    }

    public static boolean isScoringTag(int id, boolean redAlliance) {
        return inCluster(id, redAlliance ? RED_SCORING_FIRST : BLUE_SCORING_FIRST);
    }

    public static String clusterName(int id) {
        if (inCluster(id, RED_SCORING_FIRST)) return "RED SCORING";
        if (inCluster(id, RED_AUDIENCE_FIRST)) return "RED AUDIENCE";
        if (inCluster(id, BLUE_AUDIENCE_FIRST)) return "BLUE AUDIENCE";
        if (inCluster(id, BLUE_SCORING_FIRST)) return "BLUE SCORING";
        return "UNKNOWN";
    }

    // Picks the tag the robot should turn to face, from everything the Limelight sees this loop.
    // Returns null if none of our alliance's scoring tags are visible (RotateToLimelight then stops turning).
    public static LimelightData selectAimTarget(List<LimelightData> tags, boolean redAlliance) {
        int first = redAlliance ? RED_SCORING_FIRST : BLUE_SCORING_FIRST;

        LimelightData innerLeft = null;
        LimelightData innerRight = null;
        LimelightData closest = null;

        for (LimelightData tag : tags) {
            if (!isScoringTag(tag.getId(), redAlliance)) continue;

            if (tag.getId() == first + 1) innerLeft = tag;
            if (tag.getId() == first + 2) innerRight = tag;
            if (closest == null || tag.getDistance() < closest.getDistance()) closest = tag;
        }

        // the two inner tags are evenly spaced around the center, so halfway between them is the center of the row.
        if (innerLeft != null && innerRight != null) return midpoint(innerLeft, innerRight);

        // an inner tag is never more than 2.75in off-center, so it beats a closer outer tag.
        if (innerLeft != null) return innerLeft;
        if (innerRight != null) return innerRight;

        return closest;
    }

    private static LimelightData midpoint(LimelightData a, LimelightData b) {
        return new LimelightData(a.getId(),
                (a.getAngleX() + b.getAngleX()) / 2,
                (a.getAngleY() + b.getAngleY()) / 2,
                (a.getX() + b.getX()) / 2,
                (a.getY() + b.getY()) / 2,
                (a.getZ() + b.getZ()) / 2);
    }
}
