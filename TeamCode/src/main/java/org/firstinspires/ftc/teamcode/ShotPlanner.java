package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.RobotMath.Vec;

/**
 * Current-pose range and bearing lookup. No projectile lead or motion-based firing gate. The turret command
 * alone leads the pose by the turret's actuation delay so it arrives on the current bearing.
 */
public final class ShotPlanner {
    public static final double MAX_SAMPLE_AGE_SECONDS = .1;
    public static final double MAX_YAW_RAD = Math.toRadians(TurretSubsystem.ROTATION_MAX_ANGLE_DEG);
    public static final double MIN_RANGE_METERS = .001;
    public static final class Solution {
        public final boolean valid, aimable;
        public final String reason;
        /** yaw: bearing now (readiness, telemetry). commandYaw: led bearing sent to the servos. */
        public final double yaw, commandYaw, hoodDeg, rpm, effectiveRange;
        private Solution(boolean valid, boolean aimable, String reason, double yaw, double commandYaw,
                         double hoodDeg, double rpm, double range) {
            this.valid = valid; this.aimable = aimable; this.reason = reason;
            this.yaw = yaw; this.commandYaw = commandYaw; this.hoodDeg = hoodDeg; this.rpm = rpm; effectiveRange = range;
        }
        public static Solution invalid(String reason) {
            return new Solution(false, false, reason, 0, 0, 0, 0, 0);
        }
        /** Preserve calibration validity and reject trims outside the physical operating envelope. */
        public Solution withOffsets(double yawDeg, double hoodOffsetDeg, double rpmOffset) {
            if (!aimable) return this;
            double adjustedYaw = yaw + Math.toRadians(yawDeg);
            double adjustedHood = hoodDeg + hoodOffsetDeg, adjustedRpm = rpm + rpmOffset;
            if (!RobotMath.finite(adjustedYaw, adjustedHood, adjustedRpm)
                    || Math.abs(adjustedYaw) > MAX_YAW_RAD
                    || adjustedHood < TurretSubsystem.PITCH_MIN_DEG || adjustedHood > TurretSubsystem.PITCH_MAX_DEG
                    || adjustedRpm <= 0 || adjustedRpm > TurretSubsystem.SHOOTER_MAX_RPM)
                return invalid("Auto aim offsets exceed mechanism limits");
            return new Solution(valid, true, reason, adjustedYaw, RobotMath.wrap(commandYaw + Math.toRadians(yawDeg)),
                    adjustedHood, adjustedRpm, effectiveRange);
        }
        public Solution inhibit(String rejection) {
            return new Solution(false, aimable, rejection, yaw, commandYaw, hoodDeg, rpm, effectiveRange);
        }
    }
    private final ShooterMount mount;
    private final double leadSeconds;
    public ShotPlanner(ShooterMount mount) { this(mount, TurretControl.AIM_LEAD_SECONDS); }
    /** leadSeconds: turret actuation delay from the pose sample (0 disables the command lead). */
    public ShotPlanner(ShooterMount mount, double leadSeconds) {
        if (mount == null || !Double.isFinite(leadSeconds) || leadSeconds < 0 || leadSeconds > TurretControl.MAX_AIM_LEAD_SECONDS)
            throw new IllegalArgumentException("Invalid aiming configuration");
        this.mount = mount; this.leadSeconds = leadSeconds;
    }
    public double leadSeconds() { return leadSeconds; }
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
        double commandYaw = leadSeconds > 0 ? yawFrom(pose.led(leadSeconds), target) : yaw;
        return new Solution(true, true, "VALID", yaw, Double.isFinite(commandYaw) ? commandYaw : yaw,
                row.hoodDeg, row.rpm, range);
    }
    /** Same exact planar mount geometry as solve(); NaN when degenerate. */
    private double yawFrom(MotionSample p, GoalTarget target) {
        Vec pivot = new Vec(p.x, p.y).plus(mount.pivot.rotate(p.heading));
        Vec direction = new Vec(target.x, target.y).minus(pivot);
        double distance = direction.norm();
        if (distance <= Math.abs(mount.muzzle.y) || distance < MIN_RANGE_METERS) return Double.NaN;
        return RobotMath.wrap(direction.angle() - Math.asin(mount.muzzle.y / distance) - p.heading - mount.zeroYaw);
    }
}
