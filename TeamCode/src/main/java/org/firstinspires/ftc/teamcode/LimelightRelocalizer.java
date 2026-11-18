package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Set;

/**
 * Limelight AprilTag relocalization that corrects the Pedro follower pose directly.
 *
 * Works in Pedro coordinates (inches, origin in the field corner, radians).
 * Handles the turret-mounted camera: the Limelight's camera offsets (forward, side, yaw) must be 0,
 * with only pitch and height set, so each tag's "robot pose" is really the lens position.
 *
 * Rules:
 *  1. Latency: each camera frame is compared with where odometry said the robot was when the
 *     picture was taken, not where it is now.
 *  2. Bad frames are dropped: repeated frames, old frames, tags that disagree with each other,
 *     frames taken while the robot was turning fast, and frames taken while the turret was moving.
 *  3. First fix: 3 agreeing frames set the pose directly (one jump).
 *  4. After that:
 *       - small difference (inside the deadband): odometry is trusted, nothing changes;
 *       - medium drift: corrected after 5 agreeing frames;
 *       - large difference: needs 2+ tags per frame and 0.5 s of agreeing frames in a row.
 *     Corrections are blended into the follower over 0.4 s so the pose never jumps.
 *  5. resetTo() sets a known pose (e.g. robot placed at a marked spot) and clears the history.
 *
 * Loop, after drivetrain.run() (which calls follower.update()):
 * <pre>
 *   double turretAngle = (0.25 - shooter.rotation.position) * 2 * Math.PI;
 *   reloc.update(drivetrain.follower, turretAngle);
 *   telemetry.addData("Reloc", reloc.status());
 * </pre>
 */
public class LimelightRelocalizer {

    // Timing and counts.
    public static int ACQUIRE_FRAMES = 3;
    public static int DRIFT_FRAMES = 5;
    public static double RELOCALIZE_SPAN_S = 0.5;
    public static double BLEND_S = 0.4;
    public static double MAX_FRAME_AGE_S = 0.25;
    public static double HISTORY_S = 1.0;

    // Thresholds in inches and radians. Starting guesses: tune on the robot.
    public static double DEADBAND_IN = 1.2, DEADBAND_RAD = Math.toRadians(1);
    public static double LARGE_IN = 12, LARGE_RAD = Math.toRadians(10);
    public static double AGREE_IN = 2, AGREE_RAD = Math.toRadians(3);
    public static double MAX_TURN_RATE_RAD_S = Math.toRadians(90);
    public static double TURRET_SETTLE_S = 0.4, TURRET_STILL_RAD = Math.toRadians(2);

    // Robot geometry in inches.
    public static double PIVOT_FWD_IN = 2.0;   // turret pivot ahead of robot center
    public static double CAM_ARM_IN = 3.92;    // lens distance from the turret pivot

    // Odometry/IMU heading is usually more accurate than a single tag's heading,
    // so heading is left alone unless this is turned on and tested.
    public static boolean CORRECT_HEADING = false;

    private static final double M_TO_IN = 39.3701;

    /** Lens position seen by one tag, in Pedro coordinates. */
    private static final class Lens {
        final double x, y, camYaw;
        Lens(double x, double y, double camYaw) { this.x = x; this.y = y; this.camYaw = camYaw; }
    }

    private static final class Frame {
        final double id, captureTime;
        final List<Lens> lenses;
        Frame(double id, double captureTime, List<Lens> lenses) {
            this.id = id; this.captureTime = captureTime; this.lenses = lenses;
        }
    }

    private static final class Sample {
        final double time, turret; final Pose odom;
        Sample(double time, Pose odom, double turret) { this.time = time; this.odom = odom; this.turret = turret; }
    }

    private final Limelight3A limelight;
    private final Set<Integer> tagIds;

    private final ArrayDeque<Sample> history = new ArrayDeque<>();
    private boolean hasFix = false;
    private Pose blendTarget;     // correction still being blended in, or null
    private double blendLeft;
    private double lastFrameId = Double.NaN;

    // Confirmation run: a series of frames that all point at the same correction.
    private Pose runOffset;
    private int runFrames;
    private double runStart;
    private boolean runLarge;

    private String status = "Waiting for tags";

    public LimelightRelocalizer(Limelight3A limelight, Set<Integer> tagIds) {
        this.limelight = limelight;
        this.tagIds = tagIds;
    }

    // ---------------------------------------------------------------- public

    /** Call every loop after follower.update(). turretAngle in radians, 0 = facing robot-forward, CCW positive. */
    public void update(Follower f, double turretAngle) {
        double now = System.nanoTime() * 1e-9;
        double previous = history.isEmpty() ? now : history.peekLast().time;
        history.addLast(new Sample(now, f.pose(), turretAngle));
        while (now - history.peekFirst().time > HISTORY_S) history.removeFirst();

        Frame frame = fromLimelight(limelight.getLatestResult(), now);
        if (frame != null) addFrame(now, frame, f);

        stepBlend(now - previous, f);
    }

    public boolean hasFieldReference() { return hasFix; }

    public String status() { return status; }

    /** Known-pose reset (robot placed at a marked spot). Discards earlier vision work. */
    public void resetTo(Follower f, Pose known) {
        f.setPose(known);
        history.clear();
        blendTarget = null;
        clearRun();
        hasFix = true;
        status = "Reset to known pose";
    }

    // ---------------------------------------------------------------- camera

    private boolean addFrame(double now, Frame frame, Follower f) {
        if (frame.id == lastFrameId) return reject("Repeated frame");
        lastFrameId = frame.id;

        double age = now - frame.captureTime;
        if (!(age >= 0 && age <= MAX_FRAME_AGE_S)) return reject("Frame too old or in the future");
        if (frame.lenses.isEmpty()) return reject("No usable tags");

        Sample at = odometryAt(frame.captureTime);
        if (at == null) return reject("No odometry at capture time");
        if (Math.abs(turnRateAt(frame.captureTime)) > MAX_TURN_RATE_RAD_S)
            return reject("Turning too fast at capture");

        Sample settled = odometryAt(frame.captureTime - TURRET_SETTLE_S);
        if (settled == null) return reject("Collecting odometry history");
        if (Math.abs(wrap(at.turret - settled.turret)) > TURRET_STILL_RAD) return reject("Turret moving");

        List<Pose> robots = new ArrayList<>();
        for (Lens l : frame.lenses) robots.add(robotFromLens(l, at));
        Pose camera = average(robots);
        for (Pose r : robots) {
            if (!close(r, camera, AGREE_IN, AGREE_RAD)) { clearRun(); return reject("Tags disagree with each other"); }
        }

        // The correction this frame asks for, using odometry from the moment of capture.
        Pose implied = offsetFrom(camera, at.odom);

        // 1) No fix yet: a few agreeing frames set the pose directly.
        if (!hasFix) {
            countRun(implied, at.odom, frame.captureTime, false);
            if (runFrames < ACQUIRE_FRAMES) return accept("Acquiring " + runFrames + "/" + ACQUIRE_FRAMES);
            clearRun();
            blendTarget = null;
            moveTo(f, apply(implied, history.peekLast().odom));
            hasFix = true;
            return accept("Field reference acquired");
        }

        // 2) Compare the camera with our own estimate at capture time.
        Pose ours = blendTarget != null ? apply(blendTarget, at.odom) : at.odom;
        double distance = Math.hypot(camera.x() - ours.x(), camera.y() - ours.y());
        double turn = Math.abs(wrap(camera.heading() - ours.heading()));

        if (distance <= DEADBAND_IN && turn <= DEADBAND_RAD) {
            clearRun();
            return accept("Odometry agrees with camera");
        }

        boolean large = distance > LARGE_IN || turn > LARGE_RAD;
        if (large && frame.lenses.size() < 2) {
            clearRun();
            return reject(String.format("Camera disagrees by %.1f in; need 2+ tags", distance));
        }

        countRun(implied, at.odom, frame.captureTime, large);

        if (!large) {
            if (runFrames < DRIFT_FRAMES) return accept("Checking drift " + runFrames + "/" + DRIFT_FRAMES);
            startBlend(implied);
            return accept("Odometry drift corrected");
        }
        double span = frame.captureTime - runStart;
        if (span < RELOCALIZE_SPAN_S)
            return accept(String.format("Camera disagrees by %.1f in, confirming %.2f/%.1f s", distance, span, RELOCALIZE_SPAN_S));
        startBlend(implied);
        return accept("Re-localized");
    }

    /** Robot-center pose from one lens position, undoing the turret and pivot offsets. */
    private static Pose robotFromLens(Lens l, Sample at) {
        double robotHeading = CORRECT_HEADING ? wrap(l.camYaw - at.turret) : at.odom.heading();
        double camHeading = robotHeading + at.turret;
        double x = l.x - CAM_ARM_IN * Math.cos(camHeading) - PIVOT_FWD_IN * Math.cos(robotHeading);
        double y = l.y - CAM_ARM_IN * Math.sin(camHeading) - PIVOT_FWD_IN * Math.sin(robotHeading);
        return new Pose(x, y, robotHeading);
    }

    /** Lens positions per tag, converted from the Limelight's field frame (meters, center origin) to Pedro. */
    private Frame fromLimelight(LLResult result, double now) {
        if (result == null || !result.isValid()) return null;
        double ageMs = result.getStaleness() + result.getCaptureLatency() + result.getTargetingLatency();
        double captureTime = now - ageMs / 1000.0;

        List<Lens> lenses = new ArrayList<>();
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            if (!tagIds.contains(tag.getFiducialId())) continue;
            Pose3D p = tag.getRobotPoseFieldSpace();
            if (p == null) continue;
            Position m = p.getPosition().toUnit(DistanceUnit.METER);
            double x = m.y * M_TO_IN + 72;          // same conversion as Relocalization.ftcMetersToPedroXY
            double y = 72 - m.x * M_TO_IN;
            double yaw = wrap(p.getOrientation().getYaw(AngleUnit.RADIANS) - Math.PI / 2);
            if (Double.isFinite(x) && Double.isFinite(y) && Double.isFinite(yaw)
                    && x >= 0 && x <= 144 && y >= 0 && y <= 144) {
                lenses.add(new Lens(x, y, yaw));
            }
        }
        return new Frame(result.getTimestamp(), captureTime, lenses);
    }

    // ---------------------------------------------------------------- writing to Pedro

    /** Sets the follower pose and re-expresses stored history and pending corrections in the new frame. */
    private void moveTo(Follower f, Pose newPose) {
        f.setPose(newPose);
        if (history.isEmpty()) return;
        Pose current = history.peekLast().odom;
        Pose d = offsetFrom(newPose, current);   // the rigid move just applied

        ArrayDeque<Sample> moved = new ArrayDeque<>();
        for (Sample s : history) moved.addLast(new Sample(s.time, apply(d, s.odom), s.turret));
        history.clear();
        history.addAll(moved);

        if (blendTarget != null) blendTarget = rebase(blendTarget, d, current);
        if (runOffset != null) runOffset = rebase(runOffset, d, current);
    }

    /** A correction defined in the old frame, rewritten for the frame after move d. */
    private static Pose rebase(Pose correction, Pose d, Pose ref) {
        return offsetFrom(apply(correction, ref), apply(d, ref));
    }

    /** Moves the follower part of the way toward the pending correction each loop. */
    private void stepBlend(double dt, Follower f) {
        if (blendTarget == null || dt <= 0 || history.isEmpty()) return;
        Pose cur = history.peekLast().odom;
        Pose to = apply(blendTarget, cur);
        double fraction = Math.min(1, dt / blendLeft);
        blendLeft -= dt;
        Pose next = new Pose(
                cur.x() + fraction * (to.x() - cur.x()),
                cur.y() + fraction * (to.y() - cur.y()),
                wrap(cur.heading() + fraction * wrap(to.heading() - cur.heading())));
        moveTo(f, next);
        if (fraction >= 1) blendTarget = null;
    }

    // ---------------------------------------------------------------- helpers

    private void countRun(Pose implied, Pose odomAtCapture, double captureTime, boolean large) {
        boolean continues = runOffset != null && runLarge == large
                && close(apply(implied, odomAtCapture), apply(runOffset, odomAtCapture), AGREE_IN, AGREE_RAD);
        if (!continues) { runFrames = 0; runStart = captureTime; runLarge = large; }
        runOffset = implied;
        runFrames++;
    }

    private void clearRun() { runOffset = null; runFrames = 0; }

    private void startBlend(Pose correction) {
        blendTarget = correction;
        blendLeft = BLEND_S;
        clearRun();
    }

    /** Odometry pose and turret angle at time t, interpolated between the two nearest samples. */
    private Sample odometryAt(double t) {
        Sample before = null;
        for (Sample s : history) {
            if (s.time == t) return s;
            if (s.time > t) {
                if (before == null) return null;
                double k = (t - before.time) / (s.time - before.time);
                return new Sample(t, new Pose(
                        before.odom.x() + k * (s.odom.x() - before.odom.x()),
                        before.odom.y() + k * (s.odom.y() - before.odom.y()),
                        before.odom.heading() + k * wrap(s.odom.heading() - before.odom.heading())),
                        before.turret + k * (s.turret - before.turret));
            }
            before = s;
        }
        return null;
    }

    private double turnRateAt(double t) {
        Sample before = null;
        Iterator<Sample> it = history.iterator();
        while (it.hasNext()) {
            Sample s = it.next();
            if (s.time >= t && before != null)
                return wrap(s.odom.heading() - before.odom.heading()) / (s.time - before.time);
            before = s;
        }
        return 0;
    }

    /** Rigid correction (rotation + shift) that maps this odometry pose onto this field pose. */
    private static Pose offsetFrom(Pose field, Pose odom) {
        double turn = wrap(field.heading() - odom.heading()), c = Math.cos(turn), s = Math.sin(turn);
        return new Pose(field.x() - (c * odom.x() - s * odom.y()),
                field.y() - (s * odom.x() + c * odom.y()), turn);
    }

    private static Pose apply(Pose correction, Pose odom) {
        double c = Math.cos(correction.heading()), s = Math.sin(correction.heading());
        return new Pose(correction.x() + c * odom.x() - s * odom.y(),
                correction.y() + s * odom.x() + c * odom.y(),
                wrap(odom.heading() + correction.heading()));
    }

    private static Pose average(List<Pose> poses) {
        double x = 0, y = 0, sin = 0, cos = 0;
        for (Pose p : poses) {
            x += p.x(); y += p.y();
            sin += Math.sin(p.heading()); cos += Math.cos(p.heading());
        }
        return new Pose(x / poses.size(), y / poses.size(), Math.atan2(sin, cos));
    }

    private static boolean close(Pose a, Pose b, double inches, double radians) {
        return Math.hypot(a.x() - b.x(), a.y() - b.y()) <= inches
                && Math.abs(wrap(a.heading() - b.heading())) <= radians;
    }

    private static double wrap(double angle) { return Math.atan2(Math.sin(angle), Math.cos(angle)); }

    private boolean accept(String why) { status = why; return true; }
    private boolean reject(String why) { status = why; return false; }
}