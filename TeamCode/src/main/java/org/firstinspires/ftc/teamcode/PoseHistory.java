package org.firstinspires.ftc.teamcode;

import java.util.ArrayDeque;
import java.util.Deque;
import org.firstinspires.ftc.teamcode.RobotMath.Vec;

/** Timestamped pose/turret history with translation-only vision corrections; velocities stay measured. */
public final class PoseHistory {
    public static final double HISTORY_SECONDS = 1.0, MAX_OBSERVATION_AGE_SECONDS = .35;
    public static final double MAX_JUMP_METERS = .5, MAX_TARGET_AGE_SECONDS = .75;
    public static final class Entry {
        public final MotionSample pose;
        public final double turretYaw;
        Entry(MotionSample pose, double turretYaw) { this.pose = pose; this.turretYaw = turretYaw; }
    }
    private final Deque<Entry> history = new ArrayDeque<>();
    private double correctionX, correctionY, lastFrame = Double.NEGATIVE_INFINITY;
    private double lastObservation = Double.NEGATIVE_INFINITY;
    private String reason = "No target observation";
    public void reset() {
        history.clear(); correctionX = correctionY = 0; lastFrame = lastObservation = Double.NEGATIVE_INFINITY;
        reason = "Reference / target changed";
    }
    /** A new target or encoder home does not change the established field translation. */
    public void invalidateObservation(boolean clearHistory) {
        if (clearHistory) history.clear();
        lastFrame = lastObservation = Double.NEGATIVE_INFINITY; reason = "Target / turret reference changed";
    }
    public void add(MotionSample sample, double turretYaw) {
        if (!sample.fresh(sample.time, .1) || !Double.isFinite(turretYaw)) { history.clear(); return; }
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
                        a.ready && b.ready, a.fieldReferenced && b.fieldReferenced, b.status);
                return corrected(new Entry(p, previous.turretYaw + t * RobotMath.wrap(next.turretYaw - previous.turretYaw)));
            }
            previous = next;
        }
        return null;
    }
    private Entry corrected(Entry e) { return new Entry(e.pose.shifted(correctionX, correctionY, e.pose.uncertainty), e.turretYaw); }
    public MotionSample current(double now) {
        if (history.isEmpty()) return null;
        MotionSample p = history.getLast().pose;
        double age = Double.isFinite(lastObservation) ? Math.max(0, now - lastObservation) : HISTORY_SECONDS;
        return p.shifted(correctionX, correctionY, p.uncertainty + age * .05);
    }
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
        lastObservation = captureTime; reason = "Accepted tag " + tagId; return true;
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
