package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.RobotMath.Vec;

/** Current-pose range and bearing lookup. No projectile lead or motion-based firing gate. */
public final class ShotPlanner {
    public static final double MAX_SAMPLE_AGE_SECONDS = .1;
    public static final double MAX_YAW_RAD = Math.toRadians(TurretSubsystem.ROTATION_MAX_ANGLE_DEG);
    public static final double MIN_RANGE_METERS = .001;
    public static final class Solution {
        public final boolean valid, aimable;
        public final String reason;
        public final double yaw, hoodDeg, rpm, effectiveRange;
        private Solution(boolean valid, boolean aimable, String reason, double yaw, double hoodDeg,
                         double rpm, double range) {
            this.valid = valid; this.aimable = aimable; this.reason = reason;
            this.yaw = yaw; this.hoodDeg = hoodDeg; this.rpm = rpm; effectiveRange = range;
        }
        public static Solution invalid(String reason) {
            return new Solution(false, false, reason, 0, 0, 0, 0);
        }
        public Solution inhibit(String rejection) {
            return new Solution(false, aimable, rejection, yaw, hoodDeg, rpm, effectiveRange);
        }
    }
    private final ShooterMount mount;
    public ShotPlanner(ShooterMount mount) {
        if (mount == null)
            throw new IllegalArgumentException("Invalid aiming configuration");
        this.mount = mount;
    }
    public Solution solve(double now, MotionSample pose, GoalTarget target, ShotProfile profile) {
        if (pose == null || !pose.fresh(now, MAX_SAMPLE_AGE_SECONDS)) return Solution.invalid("Stale / invalid pose");
        if (!pose.fieldReferenced && (target == null || target.frame == GoalTarget.Frame.FIELD))
            return Solution.invalid("Starting field pose not configured");
        if (target == null || !target.scoreable()) return Solution.invalid("Opening geometry unavailable");
        if (profile == null || !profile.calibrated) return Solution.invalid("Shot profile UNCALIBRATED");
        if (!profile.geometryId.equals(target.geometryId)) return Solution.invalid("Profile does not support target geometry");
        if (!mount.measured) return Solution.invalid("Camera/pivot/muzzle mounting UNMEASURED");
        Vec pivot = new Vec(pose.x, pose.y).plus(mount.pivot.rotate(pose.heading));
        Vec direction = new Vec(target.x, target.y).minus(pivot);
        double distance = direction.norm();
        if (distance <= Math.abs(mount.muzzle.y) || distance < MIN_RANGE_METERS)
            return Solution.invalid("Degenerate launch direction");
        // Exact planar mount geometry, including a possible lateral muzzle offset.
        double azimuth = direction.angle() - Math.asin(mount.muzzle.y / distance);
        double range = Math.sqrt(distance * distance - mount.muzzle.y * mount.muzzle.y) - mount.muzzle.x;
        if (range < MIN_RANGE_METERS) return Solution.invalid("Degenerate launch range");
        ShotProfile.Row row = profile.sample(range);
        if (row == null) return Solution.invalid("Outside calibrated range");
        double yaw = RobotMath.wrap(azimuth - pose.heading - mount.zeroYaw);
        if (Math.abs(yaw) > MAX_YAW_RAD) return Solution.invalid("Turret yaw unreachable");
        return new Solution(true, true, "VALID", yaw, row.hoodDeg, row.rpm, range);
    }
}
