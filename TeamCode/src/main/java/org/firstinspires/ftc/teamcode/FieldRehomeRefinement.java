package org.firstinspires.ftc.teamcode;

import java.util.List;
import org.firstinspires.ftc.teamcode.RobotMath.Vec;

/** Non-blocking, bounded vision refinement of an explicitly placed corner reference. */
public final class FieldRehomeRefinement {
    public static final double TIMEOUT_SECONDS = 3.0;
    public static final int REQUIRED_FRAMES = 5;
    public static final double CONSISTENCY_METERS = .06;
    public static final double MAX_CORRECTION_METERS = .15;
    public static final double MAX_SPEED_METERS_PER_SECOND = .05;
    public static final double MAX_TURN_RADIANS_PER_SECOND = Math.toRadians(5);
    public static final double TURRET_SETTLE_SECONDS = .10;
    public static final double MAX_TURRET_CHANGE_RADIANS = Math.toRadians(.5);
    private double requestedAt, lastFrame, sumX, sumY, sumWeight, sumUncertainty;
    private int frames;
    private long referenceId;
    private boolean pending;
    public static final String NOT_REQUESTED = "Not requested";
    private String status = NOT_REQUESTED;

    public void request(double now) {
        requestedAt = now; lastFrame = Double.NEGATIVE_INFINITY;
        frames = 0; sumX = sumY = sumWeight = sumUncertainty = 0;
        referenceId = Long.MIN_VALUE; pending = true;
        status = "WAITING FOR FRESH LIMELIGHT FRAMES; keep robot stationary";
    }
    public boolean pending() { return pending; }
    public String status() { return status; }
    public void cancel(String reason) { pending = false; status = "CORNER ONLY: " + reason; }
    public void expire(double now) {
        if (pending && now - requestedAt >= TIMEOUT_SECONDS) cancel("Limelight refinement timed out");
    }

    /** Each camera frame counts once; use capture-time chassis/turret history, never botpose. */
    public boolean update(double now, double frame, double capture, List<TagObservation> tags,
                          PoseHistory history, ShooterMount mount) {
        expire(now);
        if (!pending) return false;
        MotionSample current = history.current(now);
        if (current == null || !current.fresh(now, ShotPlanner.MAX_SAMPLE_AGE_SECONDS)
                || !current.fieldReferenced || !mount.measured) {
            status = !mount.measured ? "WAITING: camera mount unmeasured"
                    : "WAITING: requires fresh field-referenced Pinpoint sample";
            return false;
        }
        if (referenceId == Long.MIN_VALUE) referenceId = current.referenceId;
        if (current.referenceId != referenceId) { cancel("odometry reference changed"); return false; }
        if (Math.hypot(current.vx,current.vy) > MAX_SPEED_METERS_PER_SECOND
                || Math.abs(current.omega) > MAX_TURN_RADIANS_PER_SECOND) {
            cancel("robot moved during refinement"); return false;
        }
        if (!RobotMath.finite(now,frame,capture)) { status = "WAITING: missing / invalid camera timing"; return false; }
        if (frame <= lastFrame) return false;
        lastFrame = frame;
        if (capture < requestedAt || capture > now || now-capture > PoseHistory.MAX_OBSERVATION_AGE_SECONDS) {
            status = "WAITING: stale / future / pre-request capture"; return false;
        }
        PoseHistory.Entry entry = history.at(capture);
        if (entry == null || !entry.pose.fieldReferenced || entry.pose.referenceId != referenceId) {
            status = "WAITING: missing capture-time Pinpoint history"; return false;
        }
        // A rotating camera magnifies encoder/latency error. Require settled capture-time and current yaw.
        PoseHistory.Entry before = history.at(capture-TURRET_SETTLE_SECONDS), latest = history.at(current.time);
        if (before == null || latest == null || before.pose.referenceId != referenceId
                || Math.abs(RobotMath.wrap(entry.turretYaw-before.turretYaw)) > MAX_TURRET_CHANGE_RADIANS
                || Math.abs(RobotMath.wrap(latest.turretYaw-entry.turretYaw)) > MAX_TURRET_CHANGE_RADIANS) {
            status = "WAITING: turret settling / missing yaw history"; return false;
        }
        double x = 0, y = 0, weight = 0, uncertainty = 0;
        Vec first = null;
        String rejected = "No mapped hive tags in camera frame";
        if (tags == null) { status = "WAITING: " + rejected; return false; }
        for (TagObservation tag : tags) {
            String rejection = TeleOpFieldReference.raisedTagRejectionReason(tag,mount);
            if (rejection != null) { rejected = rejection; continue; }
            GoalTarget landmark = TeleOpFieldReference.targetForTag(tag.id);
            GoalTarget.Tag known = landmark.tags.get(tag.id);
            double[] level = tag.levelPosition(mount);
            Vec ray = new Vec(level[0],level[1]).rotate(entry.pose.heading+mount.zeroYaw+entry.turretYaw+mount.cameraYaw);
            Vec delta = new Vec(known.x,known.y).minus(ray).minus(mount.cameraPosition(entry.pose,entry.turretYaw));
            if (delta.norm() > MAX_CORRECTION_METERS) { rejected = "Tag " + tag.id + ": correction exceeds corner bound"; continue; }
            if (first != null && delta.minus(first).norm() > CONSISTENCY_METERS) {
                status = "WAITING: tags disagree within frame"; return false;
            }
            first = first == null ? delta : first;
            double sigma = tag.uncertaintyMeters(), w = 1/(sigma*sigma);
            x += delta.x*w; y += delta.y*w; weight += w; uncertainty += sigma*w;
        }
        if (weight == 0) { status = "WAITING: " + rejected; return false; }
        x /= weight; y /= weight; uncertainty /= weight;
        if (frames > 0 && Math.hypot(x-sumX/sumWeight,y-sumY/sumWeight) > CONSISTENCY_METERS) {
            frames = 0; sumX = sumY = sumWeight = sumUncertainty = 0;
            status = "WAITING: consecutive frames disagree";
            return false;
        }
        // Tags in one frame and consecutive frames share systematic error: do not divide sigma by sqrt(N).
        double frameWeight = 1/(uncertainty*uncertainty);
        sumX += x*frameWeight; sumY += y*frameWeight;
        sumUncertainty += uncertainty*frameWeight; sumWeight += frameWeight;
        if (++frames < REQUIRED_FRAMES) {
            status = "COLLECTING LIMELIGHT " + frames + "/" + REQUIRED_FRAMES; return false;
        }
        double sigma = sumUncertainty/sumWeight;
        double prior = current.uncertainty;
        double gain = prior*prior/(prior*prior+sigma*sigma);
        if (!history.refineFieldTranslation(sumX/sumWeight*gain,sumY/sumWeight*gain,
                Math.sqrt((1-gain)*prior*prior),capture)) {
            cancel("invalid fused correction"); return false;
        }
        pending = false; status = "VISION REFINED: 5 fresh frames + corner reference";
        return true;
    }
}
