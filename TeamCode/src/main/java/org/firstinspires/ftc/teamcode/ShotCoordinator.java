package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Sole owner of target/pose selection and automatic aiming; shared by TeleOp and autonomous. */
public final class ShotCoordinator {
    private final TurretSubsystem turret;
    private final ShooterMount mount;
    private final ShotProfile profile;
    private final ShotPlanner planner;
    private final PoseHistory history = new PoseHistory();
    private final FiringCoordinator firing;
    private final boolean timingMeasured;
    private GoalTarget target;
    private ShotPlanner.Solution solution = ShotPlanner.Solution.invalid("Not requested");
    private boolean previousAutomatic, previousHomed;
    private MotionSample pose;
    private double lastFrame = Double.NEGATIVE_INFINITY;
    private double lastTrackingFrame = Double.NEGATIVE_INFINITY;
    public ShotCoordinator(TurretSubsystem turret, GoalTarget target) {
        this(turret, target, ShootingConfiguration.MOUNT, ShootingConfiguration.PROFILE,
                ShootingConfiguration.ENVELOPE, ShootingConfiguration.firing(),
                ShootingConfiguration.RELEASE_TIMING_MEASURED && ShootingConfiguration.VISION_TIMING_MEASURED);
    }
    public ShotCoordinator(TurretSubsystem turret, GoalTarget target, ShooterMount mount, ShotProfile profile,
                           ShotPlanner.Envelope envelope, FiringCoordinator firing, boolean timingMeasured) {
        this.turret = turret; this.target = target; this.mount = mount; this.profile = profile;
        planner = new ShotPlanner(mount, envelope); this.firing = firing; this.timingMeasured = timingMeasured;
        turret.setTargetHiveCell(target.alliance, target.side);
    }
    public void select(GoalTarget next) {
        target = next; history.invalidateObservation(false); lastFrame = Double.NEGATIVE_INFINITY; solution = ShotPlanner.Solution.invalid("Target changed");
        turret.setTargetHiveCell(next.alliance, next.side); turret.resetTrackingTrim();
    }
    /** Call after one odometry update and one turret update. No second hardware reads. */
    public void update(double now, MotionSample sample, boolean automatic, boolean manualFeed,
                       boolean inhibit, boolean flywheelStop) {
        if (automatic != previousAutomatic) {
            turret.resetTrackingTrim(); lastTrackingFrame = Double.NEGATIVE_INFINITY;
        }
        previousAutomatic = automatic;
        updatePose(now, sample);
        double delay = firing.releaseDelay(now);
        solution = !turret.isHomed() ? ShotPlanner.Solution.invalid("Turret: " + turret.getHomingStatus())
                : !timingMeasured ? ShotPlanner.Solution.invalid("Release / vision timing UNMEASURED")
                : planner.solve(now, pose, target, profile, Math.toRadians(turret.getEncoderAngleDegrees()),
                        Math.toRadians(turret.getEncoderVelocityDegPerSec()), delay);
        // A known field opening can be acquired using odometry while vision still inhibits feeding.
        if (solution.aimable && !history.targetFresh(now)) solution = solution.inhibit("Target observation lost / stale: " + history.reason());
        if (automatic && solution.aimable) {
            turret.setTargetYawDegrees(Math.toDegrees(solution.currentYaw(delay))); turret.setPitchAngleDegrees(solution.hoodDeg);
            if (!flywheelStop) turret.setShooterRpm(solution.rpm);
        } else if (automatic && turret.isHomed() && turret.getVisionTag() != null
                && turret.getVisionResult().getTimestamp() > lastTrackingFrame) {
            // Acquire the selected hive before shot calibration is available; this does not enable firing.
            // Vision selection already filters alliance, hive side, finite tx and stale frames.
            turret.setTargetYawDegrees(visualTrackingYawDegrees(turret.getEncoderAngleDegrees(),
                    turret.getVisionTag().getTargetXDegrees(), Math.toDegrees(mount.cameraYaw)));
            lastTrackingFrame = turret.getVisionResult().getTimestamp();
        }
        if (flywheelStop) turret.stopShooter();
        double yawError = solution.valid ? Math.abs(Math.toDegrees(solution.currentYaw(delay)) - turret.getEncoderAngleDegrees()) : Double.POSITIVE_INFINITY;
        double rateError = Math.abs(Math.toDegrees(solution.yawRate) - turret.getEncoderVelocityDegPerSec());
        boolean tight = yawError <= 1 && rateError <= 8 && turret.isHoodTracking() && turret.isShooterAtSpeed();
        boolean loose = yawError <= 1.5 && rateError <= 12 && turret.isHoodTracking()
                && turret.getTargetShooterRpm() > 0 && Math.abs(turret.getShooterCurrentRpm() - solution.rpm) <= 200;
        firing.update(now, automatic, manualFeed, inhibit, flywheelStop, solution.valid, tight, loose, solution.reason);
        turret.setTrackingStatus(solution.reason);
    }
    static double visualTrackingYawDegrees(double measuredYawDeg, double txDeg, double cameraYawDeg) {
        // tx is right-positive; internal turret yaw and camera mounting yaw are left-positive.
        double bearing = TurretSubsystem.normalizeAngleDegrees(measuredYawDeg + cameraYawDeg - txDeg);
        // In the excluded left sector, hold the current side instead of slewing across home.
        if (Math.abs(bearing) > TurretSubsystem.ROTATION_MAX_ANGLE_DEG)
            return Math.copySign(TurretSubsystem.ROTATION_MAX_ANGLE_DEG, measuredYawDeg);
        return bearing;
    }
    public void updatePose(double now, MotionSample sample) {
        if (turret.isHomed() != previousHomed) {
            history.invalidateObservation(true); lastFrame = Double.NEGATIVE_INFINITY;
            turret.resetTrackingTrim(); previousHomed = turret.isHomed();
        }
        if (sample == null) { pose = null; return; }
        history.add(sample, Math.toRadians(turret.getEncoderAngleDegrees()));
        LLResult result = turret.getVisionResult();
        LLResultTypes.FiducialResult tag = turret.getVisionTag();
        if (result != null && tag != null && turret.isHomed() && result.getTimestamp() > lastFrame) {
            lastFrame = result.getTimestamp();
            double capture = PoseHistory.captureTime(now, System.currentTimeMillis(), result.getControlHubTimeStamp(),
                    result.getCaptureLatency(), result.getTargetingLatency(), ShootingConfiguration.ADDITIONAL_VISION_TRANSPORT_SECONDS);
            history.observe(now, result.getTimestamp(), capture, tag.getFiducialId(),
                    Math.toRadians(tag.getTargetXDegrees()), Math.toRadians(tag.getTargetYDegrees()), target, mount);
        }
        pose = history.current(now);
    }
    public FiringCoordinator firing() { return firing; }
    public ShotPlanner.Solution solution() { return solution; }
    public MotionSample pose() { return pose; }
    public void addTelemetry(Telemetry telemetry, double now) {
        telemetry.addData("Shot target", target.identity());
        telemetry.addData("Shot profile", "%s | %s", profile.geometryId, profile.calibrated ? "CALIBRATED" : "UNCALIBRATED EXAMPLE");
        telemetry.addData("Shot solution", solution.reason);
        telemetry.addData("Vision age (s)", history.observationAge(now));
        telemetry.addData("Vision decision", history.reason());
        if (pose != null) telemetry.addData("Field velocity (m/s, rad/s)", "%.2f, %.2f | %.2f", pose.vx, pose.vy, pose.omega);
        telemetry.addData("Shot lead / range (deg, m)", "%.2f | %.3f", Math.toDegrees(solution.lead), solution.effectiveRange);
        telemetry.addData("Predicted release (s, m)", "%.3f | %.3f, %.3f | flight %.3f", solution.releaseTime - now,
                solution.releasePosition.x, solution.releasePosition.y, solution.flightSeconds);
        telemetry.addData("Firing", "%s | %s | committed ball possible: %s", firing.state(), firing.reason(), firing.ballMayBeCommitted());
        telemetry.addData("Timed feed attempts (not balls)", firing.feedAttempts());
    }
}
