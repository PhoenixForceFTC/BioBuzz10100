package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/**
 * Layer 1 Subsystem: goBILDA Pinpoint Odometry Computer + two goBILDA 4-Bar Odometry Pods
 * (3110-0001-0002, 32mm wheel, 2000 CPR quadrature encoder, ~19.8944 ticks/mm).
 *
 * Plain Java class (not an OpMode) so it can be reused across standalone Test OpModes,
 * TeleOp, and Autonomous.
 */
public class PinpointOdometry {

    // =========================================================================
    // goBILDA 4-Bar Odometry Pod (3110-0001-0002) & Pinpoint Offsets
    // 32mm wheel, 2000 CPR quadrature encoder => 2000 / (32 * pi) = 19.89436789 ticks/mm
    // =========================================================================
    public static final double POD_HALF_WIDTH_MM = 14.40 / 2.0; // 7.20 mm

    // Forward/Backward (X) pod physical position from robot center:
    //   (24.80 mm + 14.40 mm / 2) = +32.00 mm left of center
    //   72.20 mm back of center (-72.20 mm along forward axis)
    public static final double FORWARD_POD_LEFT_OF_CENTER_MM = 24.80 + POD_HALF_WIDTH_MM; // +32.00 mm
    public static final double FORWARD_POD_BACK_OF_CENTER_MM = 72.20;

    // Left/Right Strafe (Y) pod physical position from robot center:
    //   (81.20 mm - 14.40 mm / 2) = 74.00 mm behind center => -74.00 mm forward of center
    //   33.80 mm right of center (-33.80 mm along lateral axis)
    public static final double STRAFE_POD_BEHIND_CENTER_MM = 81.20 - POD_HALF_WIDTH_MM; // 74.00 mm
    public static final double STRAFE_POD_RIGHT_OF_CENTER_MM = 33.80;

    // Pinpoint setOffsets(xOffset, yOffset, DistanceUnit.MM):
    // - xOffset: how far sideways the X (forward) pod is from center (+left, -right) => +32.00 mm
    // - yOffset: how far forward the Y (strafe) pod is from center (+forward, -behind) => -74.00 mm
    public static final double PINPOINT_X_POD_OFFSET_MM = FORWARD_POD_LEFT_OF_CENTER_MM;   // +32.00 mm
    public static final double PINPOINT_Y_POD_OFFSET_MM = -STRAFE_POD_BEHIND_CENTER_MM;    // -74.00 mm

    public static final GoBildaPinpointDriver.EncoderDirection X_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;
    public static final GoBildaPinpointDriver.EncoderDirection Y_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;

    private final GoBildaPinpointDriver pinpoint;
    private Pose2D cachedPose = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0);
    private MotionSample sample = new MotionSample(0, 0, 0, 0, 0, 0, 0, 0, 0, false, false, "Not sampled");
    private boolean fieldReferenced;
    private long referenceId;
    private double lastVelocityTime = Double.NaN, lastVx, lastVy, lastOmega;
    // Team policy: assume no odometry drift; placement uncertainty is telemetry, not a timer.
    public static final double STARTING_POSE_UNCERTAINTY_METERS = .02;
    private GoBildaPinpointDriver.DeviceStatus cachedStatus;
    public MotionSample getSample() { return sample; }
    public boolean hasFieldReference() { return fieldReferenced; }

    public PinpointOdometry(HardwareMap hardwareMap) {
        // Active Robot Configuration: Control Hub I2C port 0, named "pinpoint".
        // The SDK resolves the configured bus from the name; Java does not assign the port.
        pinpoint = getPinpoint(hardwareMap, "pinpoint", "odo", "odometry");
        configure();
    }

    /**
     * Configures pod offsets, 4-Bar resolution, encoder directions, and recalibrates the IMU.
     * Call during INIT while the robot is stationary.
     */
    public void configure() {
        pinpoint.setOffsets(PINPOINT_X_POD_OFFSET_MM, PINPOINT_Y_POD_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(X_POD_DIRECTION, Y_POD_DIRECTION);
        referenceId++; fieldReferenced = false; lastVelocityTime = Double.NaN;
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0));
    }

    /**
     * Performs a single I2C bulk read from the Pinpoint computer.
     * Call once per control loop iteration before reading pose or velocity getters.
     */
    public void update() {
        pinpoint.update();
        double now = System.nanoTime() * 1e-9;
        cachedPose = pinpoint.getPosition();
        cachedStatus = pinpoint.getDeviceStatus();
        // Pinpoint fixed/global X/Y velocity: same axes as its reported X/Y position.
        // setPosition establishes our team field frame; do not rotate global velocities again.
        double vx = pinpoint.getVelX(DistanceUnit.METER), vy = pinpoint.getVelY(DistanceUnit.METER);
        double dt = now - lastVelocityTime;
        double acceleration = Double.isNaN(lastVelocityTime) || dt <= 0 || dt > .1
                ? Double.POSITIVE_INFINITY : Math.hypot(vx - lastVx, vy - lastVy) / dt;
        double omega = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
        double angularAcceleration = Double.isNaN(lastVelocityTime) || dt <= 0 || dt > .1
                ? Double.POSITIVE_INFINITY : Math.abs(omega - lastOmega) / dt;
        sample = new MotionSample(now, cachedPose.getX(DistanceUnit.METER), cachedPose.getY(DistanceUnit.METER),
                cachedPose.getHeading(AngleUnit.RADIANS), vx, vy, omega,
                acceleration, angularAcceleration, fieldReferenced ? STARTING_POSE_UNCERTAINTY_METERS : 1,
                cachedStatus == GoBildaPinpointDriver.DeviceStatus.READY, fieldReferenced, String.valueOf(cachedStatus)).withReference(referenceId);
        lastVelocityTime = now; lastVx = vx; lastVy = vy; lastOmega = omega;
    }

    /**
     * Zeroes the current robot pose to (0, 0, 0 deg) without recalibrating the IMU.
     */
    public void zeroPose() {
        referenceId++; fieldReferenced = false; lastVelocityTime = Double.NaN;
        pinpoint.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0));
    }

    /**
     * Sets the current robot pose to a known field position.
     */
    public void setPose(Pose2D pose) {
        if (pose == null || !RobotMath.finite(pose.getX(DistanceUnit.METER), pose.getY(DistanceUnit.METER), pose.getHeading(AngleUnit.RADIANS)))
            throw new IllegalArgumentException("Invalid starting field pose");
        pinpoint.setPosition(pose); cachedPose = pose;
        referenceId++; fieldReferenced = true; lastVelocityTime = Double.NaN;
    }

    /**
     * Sets the current robot pose in inches and degrees.
     */
    public void setPoseInches(double xInches, double yInches, double headingDegrees) {
        setPose(new Pose2D(DistanceUnit.INCH, xInches, yInches, AngleUnit.DEGREES, headingDegrees));
    }

    /**
     * Updates the robot's (X, Y) field position in inches while keeping the current Pinpoint IMU heading.
     */
    public void setPositionInchesPreservingHeading(double xInches, double yInches) {
        double currentHeadingDeg = cachedPose.getHeading(AngleUnit.DEGREES);
        setPoseInches(xInches, yInches, currentHeadingDeg);
    }

    public double getXInches() {
        return cachedPose.getX(DistanceUnit.INCH);
    }

    public double getYInches() {
        return cachedPose.getY(DistanceUnit.INCH);
    }

    public double getHeadingDegrees() {
        return cachedPose.getHeading(AngleUnit.DEGREES);
    }

    /**
     * Resets the pose to (0, 0, 0 deg) and recalibrates the IMU (robot must be stationary).
     */
    public void resetPoseAndIMU() {
        referenceId++; fieldReferenced = false; lastVelocityTime = Double.NaN;
        pinpoint.resetPosAndIMU();
    }

    public Pose2D getPose() {
        return cachedPose;
    }

    public GoBildaPinpointDriver.DeviceStatus getDeviceStatus() {
        return cachedStatus;
    }

    public GoBildaPinpointDriver getDriver() {
        return pinpoint;
    }

    /**
     * Adds initialization summary lines to telemetry during INIT.
     */
    public void addInitTelemetry(Telemetry telemetry) {
        telemetry.addData("Pinpoint Status", pinpoint.getDeviceStatus());
        telemetry.addData("Pinpoint Offsets (mm)", "xOffset: %+.2f (left), yOffset: %+.2f (fwd)",
                pinpoint.getXOffset(DistanceUnit.MM), pinpoint.getYOffset(DistanceUnit.MM));
    }

    /**
     * Adds live pose, velocity, and raw 4-bar pod counts to telemetry.
     * Does not call telemetry.update() so the caller OpMode can flush telemetry once per loop.
     */
    public void addTelemetry(Telemetry telemetry) {
        Pose2D pose = cachedPose;
        telemetry.addData("Pinpoint", "Status: %s | DEVICE cycle: %d us (%.1f Hz)",
                pinpoint.getDeviceStatus(), pinpoint.getLoopTime(), pinpoint.getFrequency());
        telemetry.addData("Pose (mm, deg)", "X: %.1f mm | Y: %.1f mm | Hdg: %.2f deg",
                pose.getX(DistanceUnit.MM),
                pose.getY(DistanceUnit.MM),
                pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Pose (in, deg)", "X: %.2f in | Y: %.2f in | Hdg: %.2f deg",
                pose.getX(DistanceUnit.INCH),
                pose.getY(DistanceUnit.INCH),
                pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Field reference", fieldReferenced ? "EXPLICIT FIELD" : "LOCAL ODOMETRY: no absolute field reference");
        telemetry.addData("Pinpoint Vel", "Vx: %.1f mm/s | Vy: %.1f mm/s | Vh: %.1f deg/s",
                sample.vx * 1000, sample.vy * 1000, Math.toDegrees(sample.omega));
        telemetry.addData("4-Bar Pods (raw)", "X (fwd) enc: %d | Y (strafe) enc: %d",
                pinpoint.getEncoderX(), pinpoint.getEncoderY());
    }

    private GoBildaPinpointDriver getPinpoint(HardwareMap hwMap, String... candidateNames) {
        for (String name : candidateNames) {
            GoBildaPinpointDriver device = hwMap.tryGet(GoBildaPinpointDriver.class, name);
            if (device != null) {
                return device;
            }
        }
        return hwMap.get(GoBildaPinpointDriver.class, candidateNames[0]);
    }
}
