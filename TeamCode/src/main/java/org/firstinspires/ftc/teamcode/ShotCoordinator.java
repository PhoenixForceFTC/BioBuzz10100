package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Sole owner of target/pose selection and automatic aiming; shared by TeleOp and autonomous. */
public final class ShotCoordinator {
    public static final double MAX_OPENING_DISAGREEMENT_METERS = .08, MAX_OPENING_HEIGHT_ERROR_METERS = .1;
    private MotionSample lastPoseSample;
    private double lastPoseUpdate = Double.NEGATIVE_INFINITY;
    private final TurretSubsystem turret;
    private final ShooterMount mount;
    private final ShotProfile profile;
    private final ShotPlanner planner;
    private final PoseHistory history = new PoseHistory();
    private final GoalStateObserver goalState;
    private GoalTarget relativeTarget;
    private TagObservation trackingTag;
    private long previousReferenceId = Long.MIN_VALUE;
    private double trackingCapture = Double.NaN;
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
        goalState = new GoalStateObserver(mount,ShootingConfiguration.LOWERED_TAG_HEIGHT_METERS,ShootingConfiguration.LOWERED_TAG_PLANE_DEG); goalState.select(target.alliance,target.side);
        planner = new ShotPlanner(mount, envelope); this.firing = firing; this.timingMeasured = timingMeasured;
        turret.setTargetHiveCell(target.alliance, target.side);
    }
    public void select(GoalTarget next) {
        target = next; history.invalidateObservation(false); lastFrame = Double.NEGATIVE_INFINITY; solution = ShotPlanner.Solution.invalid("Target changed");
        relativeTarget = null; lastPoseSample = null; goalState.select(next.alliance,next.side);
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
                : planner.solve(now, pose, relativeTarget != null ? relativeTarget : target, profile, Math.toRadians(turret.getEncoderAngleDegrees()),
                        Math.toRadians(turret.getEncoderVelocityDegPerSec()), delay);
        // A known field opening can be acquired using odometry while vision still inhibits feeding.
        if (solution.aimable && !history.targetFresh(now)) solution = solution.inhibit("Target observation lost / stale: " + history.reason());
        if (solution.valid && !ShootingConfiguration.GOAL_OBSERVATION_VALIDATED) solution = solution.inhibit("Goal detector / pipeline UNVALIDATED");
        if (solution.valid && !goalState.raised(now)) solution = solution.inhibit("Goal state: " + goalState.reason());
        if (automatic && solution.aimable) {
            turret.setTargetYawDegrees(Math.toDegrees(solution.currentYaw(delay))); turret.setPitchAngleDegrees(solution.hoodDeg);
            if (!flywheelStop) turret.setShooterRpm(solution.rpm);
        } else if (automatic && turret.isHomed() && turret.getVisionTag() != null
                && turret.getVisionResult().getTimestamp() > lastTrackingFrame) {
            // Acquire the selected hive before shot calibration is available; this does not enable firing.
            // Vision selection already filters alliance, hive side, finite tx and stale frames.
            double compensated = trackingTag != null ? history.delayedYaw3d(now,trackingCapture,trackingTag,mount)
                    : history.delayedYaw(now,trackingCapture,Math.toRadians(turret.getVisionTag().getTargetXDegrees()),
                            Math.toRadians(turret.getVisionTag().getTargetYDegrees()),mount);
            if (Double.isFinite(compensated)) {
                double degrees = Math.toDegrees(compensated);
                if (Math.abs(degrees) > TurretSubsystem.ROTATION_MAX_ANGLE_DEG)
                    degrees = Math.copySign(TurretSubsystem.ROTATION_MAX_ANGLE_DEG,turret.getEncoderAngleDegrees());
                turret.setTargetYawDegrees(degrees);
            }
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
        if (sample == lastPoseSample && now == lastPoseUpdate && turret.isHomed() == previousHomed) return;
        lastPoseSample = sample; lastPoseUpdate = now;
        if (turret.isHomed() != previousHomed) {
            history.invalidateObservation(true); lastFrame = Double.NEGATIVE_INFINITY;
            turret.resetTrackingTrim(); relativeTarget = null; goalState.invalidate("Turret reference changed"); previousHomed = turret.isHomed();
        }
        if (sample == null) { pose = null; relativeTarget = null; goalState.invalidate("No odometry"); return; }
        if (sample.referenceId != previousReferenceId) {
            history.reset(); relativeTarget = null; goalState.invalidate("Odometry reference changed");
            previousReferenceId = sample.referenceId;
        }
        history.add(sample, Math.toRadians(turret.getEncoderAngleDegrees()));
        LLResult result = turret.getVisionResult();
        LimelightObservation observation = new LimelightObservation(now,System.currentTimeMillis(),result);
        trackingCapture = observation.captureTime; trackingTag = null;
        for (TagObservation tag : observation.tags) {
            if (TurretSubsystem.getHiveSideForTag(target.alliance,tag.id) == target.side && tag.usable()
                    && (trackingTag == null || tag.uncertaintyMeters()<trackingTag.uncertaintyMeters())) trackingTag = tag;
        }
        goalState.update(now,observation.frame,observation.captureTime,observation.tags,ShootingConfiguration.LEVEL_CHASSIS_ASSUMED);
        if (!goalState.raised(now)) relativeTarget = null;
        if (result != null && turret.isHomed() && result.getTimestamp() > lastFrame) {
            lastFrame = result.getTimestamp();
            if (goalState.raised(now)) {
                TagObservation best = trackingTag;
                if (best != null && target.scoreable() && sample.fieldReferenced) {
                    if (!history.observe3d(now,observation.frame,observation.captureTime,best,target,mount)) {
                        relativeTarget = null; history.invalidateObservation(false);
                    }
                } else if (best != null) observeOpening(observation);
            }
        }
        pose = history.current(now);
    }
    private void observeOpening(LimelightObservation observation) {
        PoseHistory.Entry capture = history.at(observation.captureTime);
        if (capture == null || !ShootingConfiguration.OPENING.verified) { relativeTarget = null; return; }
        double x = 0, y = 0, z = 0, uncertainty = .025;
        int count = 0;
        java.util.List<GoalTarget.Tag> tags = new java.util.ArrayList<>();
        for (TagObservation tag : observation.tags) {
            if (TurretSubsystem.getHiveSideForTag(target.alliance,tag.id) != target.side) continue;
            double[] point = ShootingConfiguration.OPENING.opening(tag,mount,capture.pose,capture.turretYaw);
            if (point == null) { relativeTarget = null; return; }
            if (count > 0 && Math.sqrt(Math.pow(point[0]-x/count,2)+Math.pow(point[1]-y/count,2)+Math.pow(point[2]-z/count,2)) > MAX_OPENING_DISAGREEMENT_METERS) {
                goalState.invalidate("Opening reprojections disagree"); relativeTarget = null; return;
            }
            x += point[0]; y += point[1]; z += point[2]; count++;
            uncertainty = Math.max(uncertainty,tag.uncertaintyMeters());
            double[] level = tag.levelPosition(mount);
            org.firstinspires.ftc.teamcode.RobotMath.Vec camera = mount.cameraPosition(capture.pose,capture.turretYaw);
            org.firstinspires.ftc.teamcode.RobotMath.Vec position = new org.firstinspires.ftc.teamcode.RobotMath.Vec(level[0],level[1])
                    .rotate(capture.pose.heading+mount.zeroYaw+capture.turretYaw+mount.cameraYaw).plus(camera);
            tags.add(new GoalTarget.Tag(tag.id,position.x,position.y,mount.cameraHeight+level[2]));
        }
        if (count == 0) { relativeTarget = null; return; }
        x /= count; y /= count; z /= count;
        if (Math.abs(z-ShootingConfiguration.HIGH_GOAL_CENTER_HEIGHT_METERS) > MAX_OPENING_HEIGHT_ERROR_METERS
                || (relativeTarget != null && Math.hypot(x-relativeTarget.x,y-relativeTarget.y) > PoseHistory.MAX_JUMP_METERS)) {
            goalState.invalidate("Implausible observed opening correction"); relativeTarget = null; return;
        }
        relativeTarget = new GoalTarget(target.alliance,target.side,GoalTarget.HiveState.RAISED,x,y,z,true,
                ShootingConfiguration.OPENING.identity,tags.toArray(new GoalTarget.Tag[0])).inLocalFrame();
        history.acceptRelative(observation.captureTime,uncertainty);
    }
    public FiringCoordinator firing() { return firing; }
    public ShotPlanner.Solution solution() { return solution; }
    public MotionSample pose() { return pose; }
    public void addTelemetry(Telemetry telemetry, double now) {
        telemetry.addData("Shot target", relativeTarget != null ? relativeTarget.identity() : target.identity());
        telemetry.addData("Observed cell state", "%s | %s",goalState.state(now),goalState.reason());
        telemetry.addData("Tag height / plane (m, deg)", "%.3f | %.1f | age %.3f s",goalState.heightMeters(),goalState.planeDegrees(),goalState.age(now));
        telemetry.addData("Stationary envelope", planner.stationaryQualified() ? "QUALIFIED" : "Moving / qualifying");
        telemetry.addData("Shot profile", "%s | %s", profile.geometryId, profile.calibrated ? "CALIBRATED" : "UNCALIBRATED EXAMPLE");
        telemetry.addData("Shot solution", solution.reason);
        telemetry.addData("Vision age (s)", history.observationAge(now));
        telemetry.addData("Vision decision", history.reason());
        if (pose != null) telemetry.addData("Odometry-frame velocity (m/s, rad/s)", "%.2f, %.2f | %.2f", pose.vx, pose.vy, pose.omega);
        telemetry.addData("Shot lead / range (deg, m)", "%.2f | %.3f", Math.toDegrees(solution.lead), solution.effectiveRange);
        telemetry.addData("Predicted release (s, m)", "%.3f | %.3f, %.3f | flight %.3f", solution.releaseTime - now,
                solution.releasePosition.x, solution.releasePosition.y, solution.flightSeconds);
        telemetry.addData("Firing", "%s | %s | committed ball possible: %s", firing.state(), firing.reason(), firing.ballMayBeCommitted());
        telemetry.addData("Timed feed attempts (not balls)", firing.feedAttempts());
    }
}
