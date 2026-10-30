package org.firstinspires.ftc.teamcode;

import java.util.ArrayDeque;
import java.util.Deque;
import org.firstinspires.ftc.teamcode.RobotMath.Vec;

/** Timestamped pose/turret history with a local-to-field transform and translation corrections. */
public final class PoseHistory {
    public static final double HISTORY_SECONDS = 1.0, MAX_OBSERVATION_AGE_SECONDS = .35;
    public static final double MAX_JUMP_METERS = .5, MAX_TARGET_AGE_SECONDS = .75;
    public static final int ACQUISITION_FRAMES = 3;
    public static final double ACQUISITION_POSITION_TOLERANCE_METERS = .15;
    public static final double ACQUISITION_HEADING_TOLERANCE_RAD = Math.toRadians(8);
    public static final double TAG_PLANE_TOLERANCE_DEG = 10;
    public static final class Entry {
        public final MotionSample pose;
        public final double turretYaw;
        Entry(MotionSample pose, double turretYaw) { this.pose = pose; this.turretYaw = turretYaw; }
    }
    private final Deque<Entry> history = new ArrayDeque<>();
    private double correctionX, correctionY, lastFrame = Double.NEGATIVE_INFINITY;
    private double correctionYaw, candidateX, candidateY, candidateYaw, candidateTime;
    private int candidateFrames;
    private boolean visionFieldReference;
    private double lastObservation = Double.NEGATIVE_INFINITY;
    private double acceptedUncertainty = Double.NaN;
    private String reason = "No target observation";
    public void reset() {
        history.clear(); acceptedUncertainty = Double.NaN; correctionX = correctionY = 0; lastFrame = lastObservation = Double.NEGATIVE_INFINITY;
        correctionYaw = 0; visionFieldReference = false; candidateFrames = 0;
        reason = "Reference / target changed";
    }
    /** A new target or encoder home does not change the established local-to-field transform. */
    public void invalidateObservation(boolean clearHistory) {
        if (clearHistory) history.clear();
        candidateFrames = 0;
        lastFrame = lastObservation = Double.NEGATIVE_INFINITY;
        if (!visionFieldReference) acceptedUncertainty = Double.NaN;
        reason = "Target / turret reference changed";
    }
    public void add(MotionSample sample, double turretYaw) {
        if (!sample.fresh(sample.time, .1) || !Double.isFinite(turretYaw)) { history.clear(); return; }
        if (!history.isEmpty() && sample.referenceId != history.getLast().pose.referenceId) reset();
        if (!history.isEmpty() && sample.time <= history.getLast().pose.time) return;
        history.addLast(new Entry(sample, turretYaw));
        while (history.size() > 2 && (sample.time - history.getFirst().pose.time > HISTORY_SECONDS || history.size() > 256)) history.removeFirst();
    }
    public Entry at(double time) {
        Entry previous = null;
        for (Entry next : history) {
            if (next.pose.time == time) return corrected(next);
            if (next.pose.time > time) {
                if (previous == null || next.pose.time - previous.pose.time > .1) return null;
                double t = (time - previous.pose.time) / (next.pose.time - previous.pose.time);
                MotionSample a = previous.pose, b = next.pose;
                MotionSample p = new MotionSample(time, a.x + t * (b.x - a.x), a.y + t * (b.y - a.y),
                        a.heading + t * RobotMath.wrap(b.heading - a.heading), a.vx + t * (b.vx - a.vx),
                        a.vy + t * (b.vy - a.vy), a.omega + t * (b.omega - a.omega),
                        Math.max(a.acceleration, b.acceleration), Math.max(a.angularAcceleration, b.angularAcceleration), Math.max(a.uncertainty, b.uncertainty),
                        a.ready && b.ready, a.fieldReferenced && b.fieldReferenced, b.status).withReference(b.referenceId);
                return corrected(new Entry(p, previous.turretYaw + t * RobotMath.wrap(next.turretYaw - previous.turretYaw)));
            }
            previous = next;
        }
        return null;
    }
    private MotionSample corrected(MotionSample p, double uncertainty) {
        Vec position = new Vec(p.x,p.y).rotate(correctionYaw).plus(new Vec(correctionX,correctionY));
        Vec velocity = new Vec(p.vx,p.vy).rotate(correctionYaw);
        return new MotionSample(p.time,position.x,position.y,RobotMath.wrap(p.heading+correctionYaw),
                velocity.x,velocity.y,p.omega,p.acceleration,p.angularAcceleration,uncertainty,
                p.ready,p.fieldReferenced || visionFieldReference,p.status).withReference(p.referenceId);
    }
    private Entry corrected(Entry e) { return new Entry(corrected(e.pose,e.pose.uncertainty), e.turretYaw); }
    public MotionSample current(double now) {
        if (history.isEmpty()) return null;
        MotionSample p = history.getLast().pose;
        // No assumed drift or missing-camera penalty. Observation age still controls frame use.
        double uncertainty = Double.isFinite(acceptedUncertainty) ? acceptedUncertainty : p.uncertainty;
        return corrected(p, uncertainty);
    }
    /** Bootstrap both position and heading from a raised tag's outward normal, not a local IMU yaw. */
    public boolean acquireFieldReference3d(double now, double frame, double capture, TagObservation tag,
                                           GoalTarget target, ShooterMount mount) {
        if (tag == null || !tag.usable() || !RobotMath.finite(now,frame,capture) || frame <= lastFrame)
            return reject("Invalid / duplicate acquisition frame");
        lastFrame = frame;
        if (now < capture || now-capture > MAX_OBSERVATION_AGE_SECONDS) return rejectAcquisition("Stale acquisition frame");
        Entry e = at(capture);
        GoalTarget.Tag known = target.tags.get(tag.id);
        if (e == null || e.pose.fieldReferenced || !target.scoreable() || target.frame != GoalTarget.Frame.FIELD
                || known == null || !mount.measured) return rejectAcquisition("Missing acquisition history / field map");
        double[] level = tag.levelPosition(mount), normal = tag.rotate(0,0,1);
        normal = TagObservation.level(normal[0],normal[1],normal[2],mount.cameraPitch);
        if (Math.abs(mount.cameraHeight+level[2]-known.height) > .1
                || Math.abs(tag.planeDegrees(mount)-ShootingConfiguration.HIGH_GOAL_OPENING_PLANE_DEG) > TAG_PLANE_TOLERANCE_DEG)
            return rejectAcquisition("Tag is not in nominal raised geometry");
        double fieldNormalYaw = target.side == TurretSubsystem.HiveSide.AUDIENCE ? -Math.PI/2 : Math.PI/2;
        double heading = RobotMath.wrap(fieldNormalYaw-Math.atan2(normal[1],normal[0])
                -mount.zeroYaw-e.turretYaw-mount.cameraYaw);
        MotionSample origin = new MotionSample(capture,0,0,heading,0,0,0,0,.02,true,true,"Vision origin");
        Vec ray = new Vec(level[0],level[1]).rotate(heading+mount.zeroYaw+e.turretYaw+mount.cameraYaw);
        Vec position = new Vec(known.x,known.y).minus(ray).minus(mount.cameraPosition(origin,e.turretYaw));
        if (Math.abs(position.x) > TeleOpFieldReference.FIELD_HALF_WIDTH_METERS
                || Math.abs(position.y) > TeleOpFieldReference.FIELD_HALF_WIDTH_METERS)
            return rejectAcquisition("Acquired pose outside field");
        double yaw = RobotMath.wrap(heading-e.pose.heading);
        Vec translation = position.minus(new Vec(e.pose.x,e.pose.y).rotate(yaw));
        if (candidateFrames == 0 || capture-candidateTime > MAX_OBSERVATION_AGE_SECONDS
                || Math.hypot(translation.x-candidateX,translation.y-candidateY) > ACQUISITION_POSITION_TOLERANCE_METERS
                || Math.abs(RobotMath.wrap(yaw-candidateYaw)) > ACQUISITION_HEADING_TOLERANCE_RAD) {
            candidateFrames = 0; candidateX = translation.x; candidateY = translation.y; candidateYaw = yaw;
        }
        candidateTime = capture;
        if (++candidateFrames < ACQUISITION_FRAMES) return reject("Acquiring field reference " + candidateFrames + "/" + ACQUISITION_FRAMES);
        correctionYaw = yaw; correctionX = translation.x; correctionY = translation.y; visionFieldReference = true;
        acceptRelative(capture,tag.uncertaintyMeters()); reason = "Field position / heading acquired from tag " + tag.id;
        return true;
    }
    private boolean rejectAcquisition(String message) { candidateFrames = 0; return reject(message); }
    /** Known tag positions are state-specific. Never consume botpose as though it were camera pose. */
    public boolean observe(double now, double frameId, double captureTime, int tagId, double tx, double ty,
                           GoalTarget target, ShooterMount mount) {
        if (!RobotMath.finite(now, frameId, captureTime, tx, ty) || frameId <= lastFrame) return reject("Duplicate / out-of-order vision frame");
        lastFrame = frameId;
        if (now < captureTime || now - captureTime > MAX_OBSERVATION_AGE_SECONDS) return reject("Stale vision frame");
        if (!target.scoreable() || !mount.measured) return reject("Unmeasured or incompatible hive/mount geometry");
        GoalTarget.Tag tag = target.tags.get(tagId);
        if (tag == null) return reject("Tag outside selected opening/state map");
        Entry e = at(captureTime);
        if (e == null || !e.pose.fieldReferenced) return reject("Missing capture history / starting field reference");
        double yaw = e.pose.heading + mount.zeroYaw + e.turretYaw + mount.cameraYaw;
        Vec ray = mount.cameraToTag(tag, tx, ty, yaw);
        if (ray == null) return reject("Invalid tag-height range");
        Vec observedCamera = new Vec(tag.x, tag.y).minus(ray);
        Vec delta = observedCamera.minus(mount.cameraPosition(e.pose, e.turretYaw));
        if (delta.norm() > MAX_JUMP_METERS) return reject("Implausible vision position jump");
        // Correct history in a shared transform, never by differentiating a pose reset.
        correctionX += delta.x * .35; correctionY += delta.y * .35;
        acceptedUncertainty = Math.hypot(.05,delta.norm()*.65);
        lastObservation = captureTime; reason = "Accepted tag " + tagId; return true;
    }
    /** 3D observation fusion; does not infer range from a configured tag height. */
    public boolean observe3d(double now, double frame, double capture, TagObservation tag, GoalTarget target, ShooterMount mount) {
        if (tag == null || !tag.usable() || !RobotMath.finite(now,frame,capture) || frame <= lastFrame) return reject("Invalid / duplicate 3D observation");
        lastFrame = frame;
        if (now < capture || now-capture > MAX_OBSERVATION_AGE_SECONDS) return reject("Stale 3D observation");
        Entry e = at(capture);
        GoalTarget.Tag known = target.tags.get(tag.id);
        if (e == null || !e.pose.fieldReferenced || !target.scoreable() || target.frame != GoalTarget.Frame.FIELD || known == null || !mount.measured) return reject("Missing field map / capture reference");
        double[] level = tag.levelPosition(mount);
        Vec ray = new Vec(level[0],level[1]).rotate(e.pose.heading+mount.zeroYaw+e.turretYaw+mount.cameraYaw);
        Vec delta = new Vec(known.x,known.y).minus(ray).minus(mount.cameraPosition(e.pose,e.turretYaw));
        if (delta.norm() > MAX_JUMP_METERS || Math.abs(mount.cameraHeight+level[2]-known.height) > .1) return reject("Implausible 3D field correction");
        correctionX += delta.x*.35; correctionY += delta.y*.35;
        acceptRelative(capture,Math.hypot(tag.uncertaintyMeters(),delta.norm()*.65)); reason = "Accepted metric tag " + tag.id;
        return true;
    }
    /** Target-relative measurement constrains relative aiming uncertainty, never creates a field reference. */
    public void acceptRelative(double capture, double uncertainty) {
        if (!RobotMath.finite(capture,uncertainty) || uncertainty <= 0) return;
        acceptedUncertainty = Math.max(.025,uncertainty); lastObservation = capture;
    }
    /** Camera error from capture pose to the current chassis frame, with stale/missing history rejected. */
    public double delayedYaw(double now, double capture, double txRad, ShooterMount mount) {
        return delayedYaw(now,capture,txRad,0,mount);
    }
    public double delayedYaw(double now, double capture, double txRad, double tyRad, ShooterMount mount) {
        Entry e = at(capture); MotionSample current = current(now);
        if (e == null || current == null || !current.fresh(now,.1) || now < capture || now-capture > MAX_OBSERVATION_AGE_SECONDS
                || !RobotMath.finite(txRad,tyRad) || Math.abs(txRad)>=Math.PI/2 || Math.abs(tyRad)>=Math.PI/2) return Double.NaN;
        // Without metric range, constrain the uncorrected parallax from camera translation.
        if (mount.cameraPosition(current,history.getLast().turretYaw).minus(mount.cameraPosition(e.pose,e.turretYaw)).norm() > .03) return Double.NaN;
        double forward=Math.cos(mount.cameraPitch)-Math.tan(tyRad)*Math.sin(mount.cameraPitch);
        if (forward <= 0) return Double.NaN;
        double bearing=Math.atan2(-Math.tan(txRad),forward);
        return RobotMath.wrap(e.pose.heading+e.turretYaw+mount.cameraYaw+bearing-current.heading);
    }
    public double delayedYaw3d(double now, double capture, TagObservation tag, ShooterMount mount) {
        Entry e=at(capture); MotionSample current=current(now);
        if (tag == null || !tag.usable() || e == null || current == null || !current.fresh(now,.1)
                || now < capture || now-capture > MAX_OBSERVATION_AGE_SECONDS) return Double.NaN;
        double[] level=tag.levelPosition(mount);
        Vec target=mount.cameraPosition(e.pose,e.turretYaw).plus(new Vec(level[0],level[1])
                .rotate(e.pose.heading+mount.zeroYaw+e.turretYaw+mount.cameraYaw));
        Vec currentPivot=new Vec(current.x,current.y).plus(mount.pivot.rotate(current.heading));
        Vec direction=target.minus(currentPivot);
        if (direction.norm()<.1) return Double.NaN;
        return RobotMath.wrap(direction.angle()-current.heading-mount.zeroYaw);
    }
    private boolean reject(String message) { reason = message; return false; }
    public double observationAge(double now) { return now - lastObservation; }
    public boolean targetFresh(double now) { double age = observationAge(now); return age >= 0 && age <= MAX_TARGET_AGE_SECONDS; }
    public String reason() { return reason; }
    /** SDK 12 hub timestamp is currentTimeMillis at parsing start. Age already includes JSON parse time. */
    public static double captureTime(double monotonicNow, long wallNowMs, long hubReceiptMs,
                                     double captureLatencyMs, double pipelineLatencyMs, double transportSeconds) {
        double age = (wallNowMs - hubReceiptMs) * .001 + (captureLatencyMs + pipelineLatencyMs) * .001 + transportSeconds;
        if (!RobotMath.finite(age, captureLatencyMs, pipelineLatencyMs, transportSeconds) || age < 0
                || captureLatencyMs < 0 || pipelineLatencyMs < 0 || transportSeconds < 0) return Double.NaN;
        return monotonicNow - age;
    }
}
