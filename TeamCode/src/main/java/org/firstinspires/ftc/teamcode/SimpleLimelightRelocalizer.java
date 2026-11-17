package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.function.DoubleSupplier;

/**
 * Limelight 3A MegaTag2 relocalization for Pedro Pathing 3.x, with the camera on a turret.
 *
 * Because the camera moves relative to the robot, we do NOT give the Limelight the camera's
 * horizontal offset. Instead:
 *
 *   LIMELIGHT WEB UI ("camera pose in robot space"):
 *     forward = 0, side = 0, yaw = 0
 *     up    = real lens height above the floor (meters)
 *     pitch = real camera tilt, roll = real roll
 *   (Height and tilt don't change when the turret spins about a vertical axis, so those stay
 *    in the Limelight. With forward/side zeroed, MT2 returns the lens position on the field.)
 *
 *   THIS CODE, every loop:
 *     1. Sends the camera's field heading (robot heading + turret angle) to the Limelight.
 *     2. Gets back the lens position on the field.
 *     3. Subtracts the turret pivot offset and the lens-from-pivot offset, each rotated by
 *        the right angle, to get the robot center.
 *     4. Gates and blends that into the Pedro pose (x/y only).
 *
 * Call update() once per loop, AFTER follower.update().
 */
public class SimpleLimelightRelocalizer {

    // ---------------- Turret geometry (inches, robot frame: +X forward, +Y left) ----------------
    /** Turret rotation axis, measured from the robot center. */
    public static double PIVOT_FWD_IN = 0.0;
    public static double PIVOT_LEFT_IN = 0.0;
    /** Camera lens, measured from the turret axis, when the turret angle is 0. */
    public static double CAM_FWD_FROM_PIVOT_IN = 4.0;
    public static double CAM_LEFT_FROM_PIVOT_IN = 0.0;

    // ---------------- Tuning ----------------
    /** AprilTag pipeline index configured in the Limelight web UI. */
    public static int PIPELINE = 0;

    /** Fraction of the vision/odometry difference applied per accepted reading (0..1). */
    public static double BLEND = 0.3;

    /** Ignore readings older than this (ms). */
    public static double MAX_STALENESS_MS = 100;

    /** Ignore readings when the average tag distance is beyond this (meters). */
    public static double MAX_TAG_DIST_M = 3.0;

    /** Only correct when the robot is moving slower than this (in/s) ... */
    public static double MAX_LINEAR_VEL = 20;
    /** ... and the CAMERA (robot turn + turret turn) is rotating slower than this (rad/s). */
    public static double MAX_CAMERA_ANGULAR_VEL = Math.toRadians(90);

    /** Reject readings that disagree with odometry by more than this (inches). */
    public static double MAX_JUMP_IN = 12;

    /** Minimum time between corrections (ms). */
    public static double MIN_UPDATE_INTERVAL_MS = 100;

    // ---------------- Field frame conversion ----------------
    private static final double M_TO_IN = 39.3701;
    private static final double HALF_FIELD_IN = 72.0;

    private final Limelight3A limelight;
    private final Follower follower;
    /** Turret angle relative to robot forward, radians, counter-clockwise positive. */
    private final DoubleSupplier turretAngle;

    private double lastTurretAngle = Double.NaN;
    private long lastTurretTimeNs = 0;
    private double turretRate = 0;

    private long lastCorrectionMs = 0;
    private Pose lastVisionPose = null;
    private String lastStatus = "init";
    private int acceptedCount = 0;

    public SimpleLimelightRelocalizer(HardwareMap hardwareMap, Follower follower, DoubleSupplier turretAngle) {
        this.follower = follower;
        this.turretAngle = turretAngle;
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(PIPELINE);
        limelight.start(); // results are never valid without this
    }

    /** Normal per-loop call: gated, blended correction. Returns true if a correction was applied. */
    public boolean update() {
        return relocalize(false);
    }

    /** Snap straight to the vision pose (no blending, jump gate or speed gate). */
    public boolean forceRelocalize() {
        return relocalize(true);
    }

    private boolean relocalize(boolean force) {
        Pose odo = follower.pose(); // Pedro 3: inches + radians
        double robotHeading = odo.heading();
        double turret = turretAngle.getAsDouble();
        updateTurretRate(turret);

        // MegaTag2 needs the CAMERA's yaw in the FTC field frame, in degrees.
        double cameraHeading = robotHeading + turret;
        limelight.updateRobotOrientation(Math.toDegrees(pedroHeadingToFtc(cameraHeading)));

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) { lastStatus = "no valid result"; return false; }
        if (result.getStaleness() > MAX_STALENESS_MS) { lastStatus = "stale"; return false; }
        if (result.getBotposeTagCount() < 1) { lastStatus = "no tags"; return false; }
        if (result.getBotposeAvgDist() > MAX_TAG_DIST_M) { lastStatus = "tags too far"; return false; }

        Pose3D botpose = result.getBotpose_MT2();
        if (botpose == null) { lastStatus = "no MT2 pose"; return false; }

        // With forward/side zeroed in the web UI, this is the lens position on the field.
        double[] lens = ftcMetersToPedroXY(botpose.getPosition().x, botpose.getPosition().y);

        // Robot center = lens - R(robotHeading)*pivot - R(robotHeading + turret)*lensFromPivot
        double[] pivot = rotate(PIVOT_FWD_IN, PIVOT_LEFT_IN, robotHeading);
        double[] lensFromPivot = rotate(CAM_FWD_FROM_PIVOT_IN, CAM_LEFT_FROM_PIVOT_IN, cameraHeading);
        double visionX = lens[0] - pivot[0] - lensFromPivot[0];
        double visionY = lens[1] - pivot[1] - lensFromPivot[1];

        Pose vision = new Pose(visionX, visionY, robotHeading);
        lastVisionPose = vision;

        if (!force) {
            long now = System.currentTimeMillis();
            if (now - lastCorrectionMs < MIN_UPDATE_INTERVAL_MS) { lastStatus = "rate limited"; return false; }

            double speed = Math.hypot(follower.velocity().vx, follower.velocity().vy);
            if (speed > MAX_LINEAR_VEL) { lastStatus = "moving too fast"; return false; }

            // The camera rotates with both the chassis and the turret.
            double cameraRate = follower.velocity().omega + turretRate;
            if (Math.abs(cameraRate) > MAX_CAMERA_ANGULAR_VEL) { lastStatus = "camera turning too fast"; return false; }

            double jump = Math.hypot(visionX - odo.x(), visionY - odo.y());
            if (jump > MAX_JUMP_IN) { lastStatus = String.format("jump %.1f in rejected", jump); return false; }
        }

        double blend = force ? 1.0 : BLEND;
        double newX = odo.x() + blend * (visionX - odo.x());
        double newY = odo.y() + blend * (visionY - odo.y());

        // Pedro 3 setPose takes heading in radians only.
        follower.setPose(new Pose(newX, newY, robotHeading));
        lastCorrectionMs = System.currentTimeMillis();
        acceptedCount++;
        lastStatus = force ? "forced" : "applied";
        return true;
    }

    private void updateTurretRate(double turret) {
        long now = System.nanoTime();
        if (!Double.isNaN(lastTurretAngle)) {
            double dt = (now - lastTurretTimeNs) / 1e9;
            if (dt > 0) turretRate = normalize(turret - lastTurretAngle) / dt;
        }
        lastTurretAngle = turret;
        lastTurretTimeNs = now;
    }

    // ---------------------------------------------------------------------------------
    // Coordinate conversion. Limelight botpose is in the FTC field frame: origin at field
    // center, meters. Pedro uses the bottom-left corner as origin, inches, 0..144.
    // Mapping from the Pedro 3 Coordinates docs page:
    //   pedroX = ftcY + 72,  pedroY = 72 - ftcX,  pedroHeading = ftcHeading - PI/2
    // ---------------------------------------------------------------------------------

    static double[] ftcMetersToPedroXY(double ftcXm, double ftcYm) {
        double ftcX = ftcXm * M_TO_IN;
        double ftcY = ftcYm * M_TO_IN;
        return new double[]{ftcY + HALF_FIELD_IN, HALF_FIELD_IN - ftcX};
    }

    static double pedroHeadingToFtc(double pedroHeadingRad) {
        return normalize(pedroHeadingRad + Math.PI / 2);
    }

    /** Rotate a robot-frame (forward, left) vector into the Pedro field frame. */
    private static double[] rotate(double fwd, double left, double heading) {
        double c = Math.cos(heading), s = Math.sin(heading);
        return new double[]{fwd * c - left * s, fwd * s + left * c};
    }

    private static double normalize(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    // ---------------- Telemetry helpers ----------------
    public String getStatus() { return lastStatus; }
    public Pose getLastVisionPose() { return lastVisionPose; }
    public int getAcceptedCount() { return acceptedCount; }

    public void stop() { limelight.stop(); }
}