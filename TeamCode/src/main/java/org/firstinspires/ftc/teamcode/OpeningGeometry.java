package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.RobotMath.Vec;

/** Local printed-tag axes: +X sticker right, +Y sticker bottom/toward hive pivot, +Z out of printed underside.
 * Origin is cluster center on tag-center line. Offsets are meters to the chosen opening aim point.
 * Spacing/order are TU06 Fig 9-15; the remaining aim-point transform must be verified before use.
 */
public final class OpeningGeometry {
    public final String identity;
    public final boolean verified;
    public final double openingY, openingZ;
    public OpeningGeometry(String identity, boolean verified, double openingY, double openingZ) {
        if (identity == null || identity.isEmpty() || !RobotMath.finite(openingY, openingZ)) throw new IllegalArgumentException("Invalid opening transform");
        this.identity = identity; this.verified = verified; this.openingY = openingY; this.openingZ = openingZ;
    }
    public static double tagOffsetMeters(int id) {
        return HiveTagGeometry.offsetFromStickerCenterMeters(id);
    }
    /** Result is in the same fixed odometry frame as capturePose (local or field). */
    public double[] opening(TagObservation tag, ShooterMount mount, MotionSample capturePose, double captureTurretYaw) {
        if (!verified || tag == null || !tag.usable() || !mount.measured || capturePose == null
                || !capturePose.fresh(capturePose.time,.1) || !Double.isFinite(captureTurretYaw)) return null;
        double[] offset = tag.rotate(-tagOffsetMeters(tag.id), openingY, openingZ);
        double[] level = TagObservation.level(tag.x+offset[0],tag.y+offset[1],tag.z+offset[2],mount.cameraPitch);
        Vec camera = mount.cameraPosition(capturePose,captureTurretYaw);
        Vec planar = new Vec(level[0],level[1]).rotate(capturePose.heading+mount.zeroYaw+captureTurretYaw+mount.cameraYaw).plus(camera);
        return new double[]{planar.x,planar.y,mount.cameraHeight+level[2]};
    }
}
