package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.RobotMath.Vec;

/** Bounded constant-field-velocity interception solver. All internal distances are meters. */
public final class ShotPlanner {
    public static final int MAX_ITERATIONS = 24;
    public static final double MAX_SAMPLE_AGE_SECONDS = .1, MAX_RELEASE_DELAY_SECONDS = 1;
    public static final double RATE_DIFFERENCE_SECONDS = .02, MAX_YAW_RAD = Math.toRadians(TurretSubsystem.ROTATION_MAX_ANGLE_DEG);
    public static final double CONVERGENCE_METERS = .0001, CONVERGENCE_SECONDS = .0001;
    public static final double MIN_RANGE_METERS = .001, MIN_SLEW_HORIZON_SECONDS = .15;
    public static final class Envelope {
        public final double speed, turnRate, acceleration, angularAcceleration, uncertainty, turretRate;
        public Envelope(double speed, double turnRate, double acceleration, double uncertainty, double turretRate) {
            this(speed, turnRate, acceleration, 2.0, uncertainty, turretRate);
        }
        public Envelope(double speed, double turnRate, double acceleration, double angularAcceleration, double uncertainty, double turretRate) {
            if (!RobotMath.finite(speed, turnRate, acceleration, angularAcceleration, uncertainty, turretRate)
                    || speed <= 0 || turnRate <= 0 || acceleration <= 0 || angularAcceleration <= 0 || uncertainty <= 0 || turretRate <= 0)
                throw new IllegalArgumentException("Invalid motion envelope");
            this.speed = speed; this.turnRate = turnRate; this.acceleration = acceleration;
            this.angularAcceleration = angularAcceleration;
            this.uncertainty = uncertainty; this.turretRate = turretRate;
        }
    }
    public static final class Solution {
        public final boolean valid;
        public final boolean aimable;
        public final String reason;
        public final double yaw, yawRate, hoodDeg, rpm, releaseTime, flightSeconds, effectiveRange, lead;
        public final Vec releasePosition, inheritedVelocity;
        public final int iterations;
        private Solution(boolean valid, String reason, double yaw, double yawRate, double hoodDeg, double rpm,
                         double releaseTime, double flightSeconds, double range, double lead, Vec position, Vec velocity, int iterations) {
            this(valid, valid, reason, yaw, yawRate, hoodDeg, rpm, releaseTime, flightSeconds, range, lead, position, velocity, iterations);
        }
        private Solution(boolean valid, boolean aimable, String reason, double yaw, double yawRate, double hoodDeg, double rpm,
                         double releaseTime, double flightSeconds, double range, double lead, Vec position, Vec velocity, int iterations) {
            this.valid = valid; this.aimable = aimable; this.reason = reason; this.yaw = yaw; this.yawRate = yawRate;
            this.hoodDeg = hoodDeg; this.rpm = rpm; this.releaseTime = releaseTime;
            this.flightSeconds = flightSeconds; effectiveRange = range; this.lead = lead;
            releasePosition = position; inheritedVelocity = velocity; this.iterations = iterations;
        }
        public static Solution invalid(String reason) {
            return new Solution(false, reason, 0, 0, 0, 0, 0, 0, 0, 0, new Vec(0, 0), new Vec(0, 0), 0);
        }
        /** Command now so that constant-rate tracking reaches the solved yaw at release. */
        public double currentYaw(double remainingDelay) { return yaw - yawRate * remainingDelay; }
        public Solution inhibit(String rejection) {
            return new Solution(false, aimable, rejection, yaw, yawRate, hoodDeg, rpm, releaseTime,
                    flightSeconds, effectiveRange, lead, releasePosition, inheritedVelocity, iterations);
        }
    }
    private final StationaryMotion stationary = new StationaryMotion();
    public boolean stationaryQualified() { return stationary.qualified(); }
    private final ShooterMount mount;
    private final Envelope envelope;
    public ShotPlanner(ShooterMount mount, Envelope envelope) { this.mount = mount; this.envelope = envelope; }
    public Solution solve(double now, MotionSample pose, GoalTarget target, ShotProfile profile,
                          double turretYaw, double measuredTurretRate, double releaseDelay) {
        if (pose == null || !pose.fresh(now, MAX_SAMPLE_AGE_SECONDS)) return Solution.invalid("Stale / invalid pose or velocity");
        if (!pose.fieldReferenced && (target == null || target.frame == GoalTarget.Frame.FIELD)) return Solution.invalid("Starting field pose not configured");
        if (target == null || !target.scoreable()) return Solution.invalid("Hive state / opening geometry unavailable");
        if (profile == null || !profile.calibrated) return Solution.invalid("Shot profile UNCALIBRATED");
        if (!profile.geometryId.equals(target.geometryId)) return Solution.invalid("Profile does not support target geometry");
        if (!mount.measured) return Solution.invalid("Camera/pivot/muzzle mounting UNMEASURED");
        if (!RobotMath.finite(now, turretYaw, measuredTurretRate, releaseDelay) || releaseDelay < 0 || releaseDelay > MAX_RELEASE_DELAY_SECONDS)
            return Solution.invalid("Invalid release timing");
        boolean stopped = stationary.update(now, pose, measuredTurretRate);
        pose = stationary.stationarySample(pose);
        if (stopped) measuredTurretRate = 0;
        if (Math.hypot(pose.vx, pose.vy) > envelope.speed || Math.abs(pose.omega) > envelope.turnRate
                || pose.acceleration > envelope.acceleration || pose.angularAcceleration > envelope.angularAcceleration || pose.uncertainty > envelope.uncertainty)
            return Solution.invalid("Outside speed / turn / acceleration / uncertainty envelope");
        if (Math.abs(measuredTurretRate) > envelope.turretRate) return Solution.invalid("Turret moving too fast");
        double horizon = releaseDelay + now - pose.time;
        Solution first = intercept(pose, target, profile, measuredTurretRate, now + releaseDelay, horizon);
        if (!first.valid) return first;
        Solution future = intercept(pose, target, profile, measuredTurretRate, now + releaseDelay + RATE_DIFFERENCE_SECONDS, horizon + RATE_DIFFERENCE_SECONDS);
        if (!future.valid) return Solution.invalid("Upcoming solution: " + future.reason);
        double rate = RobotMath.wrap(future.yaw - first.yaw) / RATE_DIFFERENCE_SECONDS;
        if (Math.abs(first.yaw - rate * releaseDelay) > MAX_YAW_RAD) return Solution.invalid("Current tracking yaw unreachable");
        if (Math.abs(rate) > envelope.turretRate || Math.abs(first.yaw - turretYaw) > envelope.turretRate * Math.max(releaseDelay, MIN_SLEW_HORIZON_SECONDS))
            return new Solution(false, true, "Required turret slew exceeds envelope", first.yaw, rate, first.hoodDeg,
                    first.rpm, first.releaseTime, first.flightSeconds, first.effectiveRange, first.lead,
                    first.releasePosition, first.inheritedVelocity, first.iterations);
        return new Solution(true, "VALID", first.yaw, rate, first.hoodDeg, first.rpm, first.releaseTime,
                first.flightSeconds, first.effectiveRange, first.lead, first.releasePosition, first.inheritedVelocity, first.iterations);
    }
    private Solution intercept(MotionSample p, GoalTarget target, ShotProfile profile, double turretRate,
                               double releaseTime, double horizon) {
        double heading = p.heading + p.omega * horizon;
        Vec pivotOffset = mount.pivot.rotate(heading);
        Vec pivot = new Vec(p.x + p.vx * horizon, p.y + p.vy * horizon).plus(pivotOffset);
        Vec goal = new Vec(target.x, target.y);
        Vec direction = goal.minus(pivot);
        double flight = 0;
        boolean moving = !stationary.qualified();
        for (int i = 0; i < MAX_ITERATIONS; i++) {
            Vec muzzleOffset = mount.muzzle.rotate(direction.angle());
            Vec release = pivot.plus(muzzleOffset);
            Vec velocity = new Vec(p.vx, p.vy).plus(pivotOffset.cross(p.omega))
                    .plus(muzzleOffset.cross(p.omega + turretRate));
            Vec displacement = goal.minus(release).minus(velocity.times(flight));
            ShotProfile.Row row = profile.sample(displacement.norm());
            if (row == null) return Solution.invalid("Outside calibrated range");
            if (moving && !Double.isFinite(row.flightSeconds)) return Solution.invalid("Moving shot needs measured flight time");
            double nextFlight = Double.isFinite(row.flightSeconds) ? row.flightSeconds : 0;
            Vec next = goal.minus(release).minus(velocity.times(nextFlight));
            if (next.norm() < MIN_RANGE_METERS) return Solution.invalid("Degenerate launch direction");
            if (next.minus(direction).norm() < CONVERGENCE_METERS && Math.abs(nextFlight - flight) < CONVERGENCE_SECONDS) {
                double yaw = RobotMath.wrap(next.angle() - heading - mount.zeroYaw);
                if (Math.abs(yaw) > MAX_YAW_RAD) return Solution.invalid("Turret yaw unreachable");
                return new Solution(true, "VALID", yaw, 0, row.hoodDeg, row.rpm, releaseTime, row.flightSeconds,
                        displacement.norm(), RobotMath.wrap(next.angle() - goal.minus(release).angle()), release, velocity, i + 1);
            }
            direction = next; flight = nextFlight;
        }
        return Solution.invalid("Shot solver did not converge");
    }
}
