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

    public PinpointOdometry(HardwareMap hardwareMap) {
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
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0));
    }

    /**
     * Performs a single I2C bulk read from the Pinpoint computer.
     * Call once per control loop iteration before reading pose or velocity getters.
     */
    public void update() {
        pinpoint.update();
    }

    /**
     * Zeroes the current robot pose to (0, 0, 0 deg) without recalibrating the IMU.
     */
    public void zeroPose() {
        pinpoint.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0));
    }

    /**
     * Sets the current robot pose to a known field position.
     */
    public void setPose(Pose2D pose) {
        pinpoint.setPosition(pose);
    }

    /**
     * Resets the pose to (0, 0, 0 deg) and recalibrates the IMU (robot must be stationary).
     */
    public void resetPoseAndIMU() {
        pinpoint.resetPosAndIMU();
    }

    public Pose2D getPose() {
        return pinpoint.getPosition();
    }

    public GoBildaPinpointDriver.DeviceStatus getDeviceStatus() {
        return pinpoint.getDeviceStatus();
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
        Pose2D pose = pinpoint.getPosition();
        telemetry.addData("Pinpoint", "Status: %s | Loop: %d us (%.1f Hz)",
                pinpoint.getDeviceStatus(), pinpoint.getLoopTime(), pinpoint.getFrequency());
        telemetry.addData("Pose (mm, deg)", "X: %.1f mm | Y: %.1f mm | Hdg: %.2f deg",
                pose.getX(DistanceUnit.MM),
                pose.getY(DistanceUnit.MM),
                pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Pose (in, deg)", "X: %.2f in | Y: %.2f in | Hdg: %.2f deg",
                pose.getX(DistanceUnit.INCH),
                pose.getY(DistanceUnit.INCH),
                pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Pinpoint Vel", "Vx: %.1f mm/s | Vy: %.1f mm/s | Vh: %.1f deg/s",
                pinpoint.getVelX(DistanceUnit.MM),
                pinpoint.getVelY(DistanceUnit.MM),
                pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
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
