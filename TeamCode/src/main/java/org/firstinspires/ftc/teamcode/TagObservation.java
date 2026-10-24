package org.firstinspires.ftc.teamcode;

/** Metric tag pose in optical camera axes: right, down, forward. No field map is involved. */
public final class TagObservation {
    public static final double MIN_FORWARD_METERS = .1, MIN_RANGE_METERS = .25, MAX_RANGE_METERS = 6;
    public static final double MIN_AREA_PERCENT = .005, MIN_FACING_COSINE = .12;
    public final int id;
    public final double x, y, z, roll, pitch, yaw, area;
    public TagObservation(int id, double x, double y, double z, double roll, double pitch, double yaw, double area) {
        this.id = id; this.x = x; this.y = y; this.z = z;
        this.roll = roll; this.pitch = pitch; this.yaw = yaw; this.area = area;
    }
    /** SDK YPR is intrinsic Z-Y-X; rotates a vector from tag axes to camera axes. */
    public double[] rotate(double a, double b, double c) {
        double y1 = Math.cos(roll) * b - Math.sin(roll) * c;
        double z1 = Math.sin(roll) * b + Math.cos(roll) * c;
        double x2 = Math.cos(pitch) * a + Math.sin(pitch) * z1;
        double z2 = -Math.sin(pitch) * a + Math.cos(pitch) * z1;
        return new double[]{Math.cos(yaw) * x2 - Math.sin(yaw) * y1,
                Math.sin(yaw) * x2 + Math.cos(yaw) * y1, z2};
    }
    public boolean usable() {
        double range = Math.sqrt(x*x + y*y + z*z);
        if (!RobotMath.finite(x, y, z, roll, pitch, yaw, area) || z <= MIN_FORWARD_METERS || range < MIN_RANGE_METERS || range > MAX_RANGE_METERS || area < MIN_AREA_PERCENT) return false;
        double[] n = rotate(0, 0, 1);
        // Reject edge-on / back-facing detections. The printed normal must face the lens.
        return -(n[0]*x + n[1]*y + n[2]*z) / range > MIN_FACING_COSINE;
    }
    /** Level camera-yaw frame: forward, left, up, after removing the measured mount pitch. */
    public static double[] level(double x, double y, double z, double cameraPitch) {
        return new double[]{z*Math.cos(cameraPitch) + y*Math.sin(cameraPitch), -x,
                z*Math.sin(cameraPitch) - y*Math.cos(cameraPitch)};
    }
    public double[] levelPosition(ShooterMount mount) { return level(x, y, z, mount.cameraPitch); }
    public double planeDegrees(ShooterMount mount) {
        double[] n = rotate(0, 0, 1);
        double up = level(n[0], n[1], n[2], mount.cameraPitch)[2];
        return Math.toDegrees(Math.acos(Math.min(1, Math.abs(up))));
    }
    /** Conservative range/area quality model, not an SDK ambiguity score. Tolerances unvalidated. */
    public double uncertaintyMeters() { return Math.max(.025, Math.min(.14, .015 + .008*(x*x+y*y+z*z) + .002/Math.sqrt(Math.max(area, .005)))); }
}
