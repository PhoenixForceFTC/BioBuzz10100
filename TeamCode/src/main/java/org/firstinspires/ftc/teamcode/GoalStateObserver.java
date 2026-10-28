package org.firstinspires.ftc.teamcode;

import java.util.List;
import java.util.HashSet;
import java.util.Set;

/** Independent height + measured tag-plane classification. Requires an upright, level chassis.
 * Numeric tolerances are provisional. LOWERED requires a separately supplied measured low pose.
 */
public final class GoalStateObserver {
    public enum State { UNKNOWN, TRANSITIONING, RAISED, LOWERED }
    public static final double MAX_AGE_SECONDS = .15, STABLE_SECONDS = .12;
    public static final int CONSISTENT_FRAMES = 3;
    public static final double RAISED_TAG_PLANE_DEG = FieldGeometry.HIVE_TILT_DEG;
    public static final double HEIGHT_ENTER_TOLERANCE_METERS = .07, HEIGHT_RETAIN_TOLERANCE_METERS = .10;
    public static final double PLANE_ENTER_TOLERANCE_DEG = 8, PLANE_RETAIN_TOLERANCE_DEG = 12;
    public static final double TAG_HEIGHT_AGREEMENT_METERS = .06, TAG_CLUSTER_AGREEMENT_METERS = .08, TAG_AXES_AGREEMENT_DEG = 8;
    public static final double HEIGHT_NOISE_METERS = .025, MAX_HEIGHT_RATE_METERS_PER_SECOND = .12;
    public static final double PLANE_NOISE_DEG = 2, MAX_PLANE_RATE_DEG_PER_SECOND = 20;
    private final ShooterMount mount;
    private final double loweredHeight, loweredPlane;
    private TurretSubsystem.Alliance alliance;
    private TurretSubsystem.HiveSide side;
    private State state = State.UNKNOWN, candidate = State.UNKNOWN;
    private int frames;
    private double lastFrame = Double.NEGATIVE_INFINITY, capture = Double.NEGATIVE_INFINITY, since;
    private double height = Double.NaN, plane = Double.NaN, previousHeight, previousPlane;
    private String reason = "No observation";
    public GoalStateObserver(ShooterMount mount) { this(mount, Double.NaN, Double.NaN); }
    public GoalStateObserver(ShooterMount mount, double loweredHeight, double loweredPlane) {
        this.mount = mount; this.loweredHeight = loweredHeight; this.loweredPlane = loweredPlane;
    }
    public void select(TurretSubsystem.Alliance alliance, TurretSubsystem.HiveSide side) {
        if (this.alliance != alliance || this.side != side) {
            this.alliance = alliance; this.side = side; lastFrame = capture = Double.NEGATIVE_INFINITY;
            invalidate("Selected cell changed");
        }
    }
    public void invalidate(String reason) { state = candidate = State.UNKNOWN; frames = 0; this.reason = reason; }
    public void update(double now, double frame, double captureTime, List<TagObservation> tags, boolean levelChassisSupported) {
        if (!levelChassisSupported || !mount.measured) { invalidate("Requires measured mount and level chassis (no tilt feedback)"); return; }
        if (!RobotMath.finite(now, frame, captureTime) || now < captureTime || now-captureTime > MAX_AGE_SECONDS) {
            invalidate("Missing / stale capture time"); return;
        }
        boolean selectedVisible = false;
        if (tags != null) for (TagObservation tag : tags) {
            if (tag == null || TurretSubsystem.getHiveSideForTag(alliance,tag.id) != side) continue;
            selectedVisible = true;
            if (!tag.usable()) { invalidate("Invalid / zero / unusable selected-tag 3D pose"); return; }
        }
        // Invalid/no-target camera payloads revoke permission even if the camera frame counter stalls.
        if (!selectedVisible) { invalidate("Selected cell missing"); return; }
        if (now-capture > MAX_AGE_SECONDS) invalidate("Observation gap; requalifying");
        if (frame < lastFrame) { invalidate("Out-of-order frame"); return; }
        if (frame == lastFrame) return; // Polling the same fresh frame never qualifies another frame.
        lastFrame = frame;
        double sumHeight = 0, sumPlane = 0, low = Double.POSITIVE_INFINITY, high = Double.NEGATIVE_INFINITY;
        double lowPlane = Double.POSITIVE_INFINITY, highPlane = Double.NEGATIVE_INFINITY;
        double[] firstNormal = null, firstAxis = null, firstCluster = null;
        Set<Integer> ids = new HashSet<>();
        if (tags != null) for (TagObservation tag : tags) {
            if (tag == null || TurretSubsystem.getHiveSideForTag(alliance, tag.id) != side) continue;
            if (!ids.add(tag.id) || !tag.usable()) { invalidate("Invalid / zero / unusable selected-tag 3D pose"); return; }
            double h = mount.cameraHeight + tag.levelPosition(mount)[2], p = tag.planeDegrees(mount);
            double[] normal = tag.rotate(0, 0, 1);
            if (firstNormal != null && normal[0]*firstNormal[0]+normal[1]*firstNormal[1]+normal[2]*firstNormal[2] < Math.cos(Math.toRadians(TAG_AXES_AGREEMENT_DEG))) {
                invalidate("Conflicting tag-plane orientations in selected cell"); return;
            }
            double[] axis = tag.rotate(1,0,0);
            double[] offset = tag.rotate(-OpeningGeometry.tagOffsetMeters(tag.id),0,0);
            double[] cluster = {tag.x+offset[0],tag.y+offset[1],tag.z+offset[2]};
            if (firstAxis != null && (axis[0]*firstAxis[0]+axis[1]*firstAxis[1]+axis[2]*firstAxis[2] < Math.cos(Math.toRadians(TAG_AXES_AGREEMENT_DEG))
                    || Math.sqrt(Math.pow(cluster[0]-firstCluster[0],2)+Math.pow(cluster[1]-firstCluster[1],2)+Math.pow(cluster[2]-firstCluster[2],2)) > TAG_CLUSTER_AGREEMENT_METERS)) {
                invalidate("Conflicting tag cluster poses / axes"); return;
            }
            firstNormal = normal; firstAxis = axis; firstCluster = cluster;
            if (TagObservation.level(normal[0], normal[1], normal[2], mount.cameraPitch)[2] >= 0) {
                invalidate("Tag plane faces upward; incompatible underside pose"); return;
            }
            sumHeight += h; sumPlane += p; low = Math.min(low,h); high = Math.max(high,h);
            lowPlane = Math.min(lowPlane,p); highPlane = Math.max(highPlane,p);
        }
        if (ids.isEmpty()) { invalidate("Selected cell missing"); return; }
        height = sumHeight/ids.size(); plane = sumPlane/ids.size();
        if (high-low > TAG_HEIGHT_AGREEMENT_METERS || highPlane-lowPlane > TAG_AXES_AGREEMENT_DEG) { invalidate("Conflicting tags in selected cell"); return; }
        double dt = captureTime-capture;
        boolean tipping = frames > 0 && dt > 0 && (Math.abs(height-previousHeight) > HEIGHT_NOISE_METERS + MAX_HEIGHT_RATE_METERS_PER_SECOND*dt
                || Math.abs(plane-previousPlane) > PLANE_NOISE_DEG + MAX_PLANE_RATE_DEG_PER_SECOND*dt);
        previousHeight = height; previousPlane = plane;
        if (captureTime <= capture) { invalidate("Duplicate / out-of-order capture"); return; }
        capture = captureTime;
        double hTolerance = state == State.RAISED ? HEIGHT_RETAIN_TOLERANCE_METERS : HEIGHT_ENTER_TOLERANCE_METERS,
                pTolerance = state == State.RAISED ? PLANE_RETAIN_TOLERANCE_DEG : PLANE_ENTER_TOLERANCE_DEG;
        State next = !tipping && Math.abs(height-ShootingConfiguration.HIGH_GOAL_TAG_CENTER_HEIGHT_METERS) <= hTolerance
                && Math.abs(plane-RAISED_TAG_PLANE_DEG) <= pTolerance ? State.RAISED
                : !tipping && Double.isFinite(loweredHeight) && Double.isFinite(loweredPlane)
                && Math.abs(height-loweredHeight) <= HEIGHT_ENTER_TOLERANCE_METERS && Math.abs(plane-loweredPlane) <= PLANE_ENTER_TOLERANCE_DEG ? State.LOWERED : State.TRANSITIONING;
        if (next == State.TRANSITIONING) { state = candidate = next; frames = 0; reason = "Moving / unsupported height or tag plane"; return; }
        if (candidate != next || frames == 0) { candidate = next; frames = 0; since = captureTime; state = State.TRANSITIONING; }
        frames++;
        if (frames >= CONSISTENT_FRAMES && captureTime-since >= STABLE_SECONDS) {
            state = next; reason = "Stable " + next + " (provisional tolerances; level chassis)";
        } else reason = "Qualifying distinct consistent frames";
    }
    public State state(double now) {
        if (!Double.isFinite(now) || now < capture || now-capture > MAX_AGE_SECONDS) invalidate("Selected cell lost / stale");
        return state;
    }
    public boolean raised(double now) { return state(now) == State.RAISED; }
    public double heightMeters() { return height; }
    public double planeDegrees() { return plane; }
    public double age(double now) { return now-capture; }
    public String reason() { return reason; }
}
