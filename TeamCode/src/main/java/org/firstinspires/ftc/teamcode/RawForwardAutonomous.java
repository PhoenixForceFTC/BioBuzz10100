package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Layer 3 autonomous with no Pedro Pathing and no Zenith: the same {@link MecanumDrivetrain} mixing the TeleOp
 * uses plus the {@link PinpointOdometry} pods, nothing else. It drives straight forward, very slowly, until the
 * odometry pods report {@link #DISTANCE_INCHES} of forward travel, then stops.
 *
 * <p>Open loop: a constant forward command, no heading correction, no path following. Place the robot on any
 * clear floor facing the direction to drive, with a little clear space ahead, and keep it still during INIT
 * (the Pinpoint IMU recalibrates).
 */
@Autonomous(name = "Test: Raw Forward 5 in (no Pedro)", group = "Test")
public class RawForwardAutonomous extends LinearOpMode {

    /** Forward travel measured by the odometry pods before stopping, in inches. */
    public static final double DISTANCE_INCHES = 5.0;
    /** Normalized forward power. Wheel no-load top speed is about 100 in/s, so 0.10 is roughly 10 in/s. */
    public static final double FORWARD_POWER = 0.10;
    /** Safety stop if the pods never report the distance (dead pod, wheel slip). */
    public static final double TIMEOUT_SECONDS = 6.0;

    @Override
    public void runOpMode() {
        // This routine has no field pose. An older autonomous pose must never reach the next TeleOp.
        AutoPoseHandoff.clear();
        RobotLoop loop = new RobotLoop(hardwareMap);
        MecanumDrivetrain drivetrain = new MecanumDrivetrain(hardwareMap);
        PinpointOdometry odometry = new PinpointOdometry(hardwareMap);

        // INIT: wait for the Pinpoint IMU recalibration; keep the robot still.
        while (!isStarted() && !isStopRequested()) {
            loop.begin();
            odometry.update();
            telemetry.addLine("Raw Forward " + DISTANCE_INCHES + " in: keep robot still until Pinpoint is READY");
            odometry.addInitTelemetry(telemetry);
            odometry.addDriverTelemetry(telemetry);
            loop.addTelemetry(telemetry); telemetry.update();
        }
        if (isStopRequested()) return;

        try {
            // Do not start until the IMU has settled, then measure travel from wherever the pods read now.
            while (opModeIsActive() && !odometry.isSettled(System.nanoTime() * 1e-9)) {
                loop.begin();
                odometry.update();
                telemetry.addLine("Waiting for Pinpoint to settle");
                odometry.addDriverTelemetry(telemetry);
                telemetry.update();
            }
            if (!opModeIsActive()) return;

            loop.begin();
            odometry.update();
            final double startX = odometry.getXInches();
            final long startNanos = System.nanoTime();
            String result = "Running";

            while (opModeIsActive()) {
                loop.begin();
                odometry.update();
                double traveled = odometry.getXInches() - startX;
                double elapsed = (System.nanoTime() - startNanos) * 1e-9;

                if (odometry.hasDeviceFault()) {
                    result = "STOPPED: Pinpoint fault " + odometry.getDeviceStatus();
                } else if (traveled >= DISTANCE_INCHES) {
                    result = "DONE";
                } else if (elapsed >= TIMEOUT_SECONDS) {
                    result = "STOPPED: timeout before reaching distance";
                }
                if (!result.equals("Running")) {
                    drivetrain.stop();
                } else {
                    drivetrain.driveRobotCentric(FORWARD_POWER, 0.0, 0.0);
                }

                telemetry.addData("Status", result);
                telemetry.addData("Forward travel (in)", "%.2f of %.1f", traveled, DISTANCE_INCHES);
                Pose2D pose = odometry.getPose();
                telemetry.addData("Sideways (in) | Heading (deg)", "%.2f | %.2f",
                        pose.getY(DistanceUnit.INCH), odometry.getHeadingDegrees());
                drivetrain.addTelemetry(telemetry);
                loop.addTelemetry(telemetry); telemetry.update();
            }
        } finally {
            drivetrain.stop();
        }
    }
}
