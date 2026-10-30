package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Sole owner of target/pose selection and automatic aiming; shared by TeleOp and autonomous. */
public final class ShotCoordinator {
    public enum AimMode { DIAGNOSTIC, TELEOP_ODOMETRY, AUTONOMOUS_ODOMETRY }
    public static final double MAX_OPENING_DISAGREEMENT_METERS = .08, MAX_OPENING_HEIGHT_ERROR_METERS = .1;
    public static final double FIELD_RELOCALIZATION_INTERVAL_SECONDS = .5;
    private final boolean fieldOdometryOnly;
    private final AimMode mode;
    private double lastFieldCorrection = Double.NEGATIVE_INFINITY;
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
        this(turret, target, AimMode.DIAGNOSTIC);
    }
    /** Competition TeleOp uses fixed field targets; camera observations only correct the pose. */
    public ShotCoordinator(TurretSubsystem turret, GoalTarget target, AimMode mode) {
        this(turret, target, ShootingConfiguration.MOUNT, ShootingConfiguration.PROFILE,
                ShootingConfiguration.firing(),
                ShootingConfiguration.RELEASE_TIMING_MEASURED
                        && (mode != AimMode.DIAGNOSTIC || ShootingConfiguration.VISION_TIMING_MEASURED), mode);
    }
    public ShotCoordinator(TurretSubsystem turret, GoalTarget target, ShooterMount mount, ShotProfile profile,
                           FiringCoordinator firing, boolean timingMeasured) {
        this(turret, target, mount, profile, firing, timingMeasured, AimMode.DIAGNOSTIC);
    }
    private ShotCoordinator(TurretSubsystem turret, GoalTarget target, ShooterMount mount, ShotProfile profile,
                            FiringCoordinator firing, boolean timingMeasured,
                            AimMode mode) {
        this.mode = mode;
        this.fieldOdometryOnly = mode != AimMode.DIAGNOSTIC;
        this.turret = turret; this.target = target; this.mount = mount; this.profile = profile;
        goalState = new GoalStateObserver(mount,ShootingConfiguration.LOWERED_TAG_HEIGHT_METERS,ShootingConfiguration.LOWERED_TAG_PLANE_DEG); goalState.select(target.alliance,target.side);
        planner = new ShotPlanner(mount); this.firing = firing; this.timingMeasured = timingMeasured;
        turret.setTargetHiveCell(target.alliance, target.side);
    }
    public void select(GoalTarget next) {
        target = next; history.invalidateObservation(false); lastFrame = Double.NEGATIVE_INFINITY; solution = ShotPlanner.Solution.invalid("Target changed");
        relativeTarget = null; lastPoseSample = null;
        if (mode != AimMode.TELEOP_ODOMETRY) lastFieldCorrection = Double.NEGATIVE_INFINITY;
        goalState.select(next.alliance,next.side);
        turret.setTargetHiveCell(next.alliance, next.side); turret.resetTrackingTrim();
    }
    /** Call after one odometry update and one turret update. No second hardware reads. */
    public void update(double now, MotionSample sample, boolean automatic, boolean manualFeed,
                       boolean inhibit, boolean flywheelStop) {
        update(now, sample, automatic, automatic, manualFeed, inhibit, flywheelStop);
    }
    /** Autonomous can track throughout a route while requesting feed only in scoring states. */
    public void update(double now, MotionSample sample, boolean automatic, boolean automaticFeed,
                       boolean manualFeed, boolean inhibit, boolean flywheelStop) {
        if (automatic != previousAutomatic) {
            turret.resetTrackingTrim(); lastTrackingFrame = Double.NEGATIVE_INFINITY;
        }
        previousAutomatic = automatic;
        updatePose(now, sample);
        if (mode == AimMode.TELEOP_ODOMETRY && pose != null && pose.fresh(now, ShotPlanner.MAX_SAMPLE_AGE_SECONDS) && pose.fieldReferenced) {
            TurretSubsystem.HiveSide side = TeleOpFieldReference.nearestSide(pose.y, target.side);
            if (side != target.side) {
                select(TeleOpFieldReference.target(target.alliance, side));
                firing.stop(now); // A different cell must earn its own readiness interval.
            }
        }
        solution = !turret.isHomed() ? ShotPlanner.Solution.invalid("Turret: " + turret.getHomingStatus())
                : planner.solve(now, pose, relativeTarget != null ? relativeTarget : target, profile);
        // Timing blocks feed only; keep current-pose hood/RPM/yaw updates available while tuning.
        if (solution.valid && !timingMeasured) solution = solution.inhibit(fieldOdometryOnly
                ? "Release timing UNMEASURED" : "Release / vision timing UNMEASURED");
        // Camera freshness/state gates apply only to the legacy diagnostics.
        if (!fieldOdometryOnly) {
            if (solution.aimable && !history.targetFresh(now)) solution = solution.inhibit("Target observation lost / stale: " + history.reason());
            if (solution.valid && !ShootingConfiguration.GOAL_OBSERVATION_VALIDATED) solution = solution.inhibit("Goal detector / pipeline UNVALIDATED");
            if (solution.valid && !goalState.raised(now)) solution = solution.inhibit("Goal state: " + goalState.reason());
        }
        if (automatic && solution.aimable) {
            turret.setTargetYawDegrees(Math.toDegrees(solution.yaw)); turret.setPitchAngleDegrees(solution.hoodDeg);
            if (!flywheelStop) turret.setShooterRpm(solution.rpm);
        } else if (fieldOdometryOnly && automatic && turret.isHomed() && pose != null
                && pose.fieldReferenced && pose.fresh(now, ShotPlanner.MAX_SAMPLE_AGE_SECONDS)) {
            // Keep odometry yaw tracking available while RPM/hood calibration is pending.
            double degrees = Math.toDegrees(TeleOpFieldReference.yawRadians(pose, target, mount));
            if (Math.abs(degrees) > TurretSubsystem.ROTATION_MAX_ANGLE_DEG)
                degrees = Math.copySign(TurretSubsystem.ROTATION_MAX_ANGLE_DEG, turret.getEncoderAngleDegrees());
            turret.setTargetYawDegrees(degrees);
        } else if (!fieldOdometryOnly && automatic && turret.isHomed() && turret.getVisionTag() != null
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
        double yawError = solution.valid ? Math.abs(Math.toDegrees(solution.yaw) - turret.getEncoderAngleDegrees()) : Double.POSITIVE_INFINITY;
        boolean angleReady = ShotReadiness.angleReady(yawError);
        boolean tight = angleReady && turret.isHoodTracking() && turret.isShooterAtSpeed();
        boolean loose = angleReady && turret.isHoodTracking()
                && turret.getTargetShooterRpm() > 0 && Math.abs(turret.getShooterCurrentRpm() - solution.rpm)
                <= TurretSubsystem.SHOOTER_LOOSE_READY_TOLERANCE_RPM;
        firing.update(now, automatic && automaticFeed, manualFeed, inhibit, flywheelStop, solution.valid, tight, loose, solution.reason);
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
            lastFieldCorrection = Double.NEGATIVE_INFINITY;
            previousReferenceId = sample.referenceId;
        }
        history.add(sample, Math.toRadians(turret.getEncoderAngleDegrees()));
        if (mode == AimMode.AUTONOMOUS_ODOMETRY) {
            // Routes and shots share odometry. No mid-route camera correction or direct visual aim.
            pose = history.current(now);
            return;
        }
        LLResult result = turret.getVisionResult();
        LimelightObservation observation = new LimelightObservation(now,System.currentTimeMillis(),result);
        if (mode == AimMode.TELEOP_ODOMETRY) {
            updateFieldLocalization(now,observation);
            pose = history.current(now);
            return;
        }
        trackingCapture = observation.captureTime; trackingTag = null;
        for (TagObservation tag : observation.tags) {
            if (TurretSubsystem.getHiveSideForTag(target.alliance,tag.id) == target.side && tag.usable()
                    && (trackingTag == null || tag.uncertaintyMeters()<trackingTag.uncertaintyMeters())) trackingTag = tag;
        }
        if (!fieldOdometryOnly) {
            goalState.update(now,observation.frame,observation.captureTime,observation.tags,ShootingConfiguration.LEVEL_CHASSIS_ASSUMED);
            if (!goalState.raised(now)) relativeTarget = null;
        }
        if (result != null && turret.isHomed() && result.getTimestamp() > lastFrame) {
            lastFrame = result.getTimestamp();
            if (fieldOdometryOnly || goalState.raised(now)) {
                TagObservation best = trackingTag;
                if (best != null && target.scoreable() && sample.fieldReferenced) {
                    boolean correctionAllowed = !fieldOdometryOnly
                            || (ShootingConfiguration.FIELD_RELOCALIZATION_ENABLED
                            && now - lastFieldCorrection >= FIELD_RELOCALIZATION_INTERVAL_SECONDS);
                    if (correctionAllowed) {
                        if (history.observe3d(now,observation.frame,observation.captureTime,best,target,mount)) {
                            lastFieldCorrection = now;
                        } else {
                            relativeTarget = null; history.invalidateObservation(false);
                        }
                    }
                } else if (!fieldOdometryOnly && best != null) observeOpening(observation);
            }
        }
        pose = history.current(now);
    }
    private void updateFieldLocalization(double now, LimelightObservation observation) {
        MotionSample current = history.current(now);
        if (!ShootingConfiguration.FIELD_RELOCALIZATION_ENABLED || !ShootingConfiguration.LEVEL_CHASSIS_ASSUMED
                || !turret.isHomed() || current == null || !current.fresh(now,ShotPlanner.MAX_SAMPLE_AGE_SECONDS)
                || !Double.isFinite(observation.frame) || observation.frame <= lastFrame) return;
        lastFrame = observation.frame;
        if (current.fieldReferenced && now-lastFieldCorrection < FIELD_RELOCALIZATION_INTERVAL_SECONDS) return;
        TagObservation best = null;
        for (TagObservation tag : observation.tags) {
            if (tag.usable() && tag.id >= 30 && tag.id <= 45
                    && Math.abs(tag.planeDegrees(mount)-ShootingConfiguration.HIGH_GOAL_OPENING_PLANE_DEG) <= PoseHistory.TAG_PLANE_TOLERANCE_DEG
                    && Math.abs(mount.cameraHeight+tag.levelPosition(mount)[2]-ShootingConfiguration.HIGH_GOAL_TAG_CENTER_HEIGHT_METERS) <= .1
                    && (best == null || tag.uncertaintyMeters() < best.uncertaintyMeters())) best = tag;
        }
        if (best == null) return;
        GoalTarget landmark = TeleOpFieldReference.targetForTag(best.id);
        boolean accepted = current.fieldReferenced
                ? history.observe3d(now,observation.frame,observation.captureTime,best,landmark,mount)
                : history.acquireFieldReference3d(now,observation.frame,observation.captureTime,best,landmark,mount);
        if (accepted) lastFieldCorrection = now;
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
        telemetry.addData("Aim reference", mode == AimMode.AUTONOMOUS_ODOMETRY ? "FIELD ODOMETRY | fixed route target"
                : fieldOdometryOnly ? "FIELD ODOMETRY | periodic validated vision corrections" : "Field / target-relative diagnostics");
        telemetry.addData("Shot target", relativeTarget != null ? relativeTarget.identity() : target.identity());
        if (fieldOdometryOnly) {
            telemetry.addData("Hive state", "Driver responsibility; no automatic state tracking");
            telemetry.addData("Vision relocalization", mode == AimMode.AUTONOMOUS_ODOMETRY ? "Disabled during autonomous route" : target.tags.isEmpty()
                    ? "No tag field map" : ShootingConfiguration.FIELD_RELOCALIZATION_ENABLED
                    ? "Enabled | nominal TU06 map | " + history.reason() : "Disabled by configuration");
            telemetry.addData("Field localization", pose != null && pose.fieldReferenced
                    ? "LOCALIZED" : "WAITING FOR MAPPED RAISED TAG + HOMED TURRET; automatic aim unavailable");
            if (pose != null && pose.fieldReferenced) {
                telemetry.addData("Aiming field pose (m, deg)", "%.3f, %.3f | %.1f", pose.x, pose.y, Math.toDegrees(pose.heading));
                org.firstinspires.ftc.teamcode.RobotMath.Vec pivot = new org.firstinspires.ftc.teamcode.RobotMath.Vec(pose.x, pose.y).plus(mount.pivot.rotate(pose.heading));
                telemetry.addData("Odometry target range / yaw (m, deg)", "%.3f | %.1f",
                        Math.hypot(target.x - pivot.x, target.y - pivot.y),
                        Math.toDegrees(TeleOpFieldReference.yawRadians(pose, target, mount)));
            }
        } else {
            telemetry.addData("Observed cell state", "%s | %s",goalState.state(now),goalState.reason());
            telemetry.addData("Tag height / plane (m, deg)", "%.3f | %.1f | age %.3f s",goalState.heightMeters(),goalState.planeDegrees(),goalState.age(now));
        }
        telemetry.addData("Shot profile", "%s | %s", profile.geometryId, profile.calibrated ? "CALIBRATED" : "UNCALIBRATED EXAMPLE");
        telemetry.addData("Shot solution", solution.reason);
        telemetry.addData("Vision age (s)", history.observationAge(now));
        telemetry.addData("Vision decision", history.reason());
        if (pose != null) telemetry.addData("Odometry-frame velocity (m/s, rad/s)", "%.2f, %.2f | %.2f", pose.vx, pose.vy, pose.omega);
        if (pose != null) telemetry.addData("Pose uncertainty estimate (m)", "%.3f | no drift assumed; no feed cutoff", pose.uncertainty);
        telemetry.addData("Current aim yaw / range (deg, m)", "%.2f | %.3f", Math.toDegrees(solution.yaw), solution.effectiveRange);
        telemetry.addData("Auto feed readiness", "Angle within 3 deg + hood/RPM; no motion gate");
        telemetry.addData("Firing", "%s | %s | committed ball possible: %s", firing.state(), firing.reason(), firing.ballMayBeCommitted());
        telemetry.addData("Timed feed attempts (not balls)", firing.feedAttempts());
        telemetry.addData("Timed shooting (s)", "gate %.2f | feed %.2f | recovery %.2f",
                firing.gateTravelSeconds, firing.feedWindowSeconds, firing.recoverySeconds);
    }
}
