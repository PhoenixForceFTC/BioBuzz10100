package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Layer 2 Standalone Test OpMode: Tests ONLY the PinpointOdometry subsystem in isolation
 * (no motors are energized; push the robot by hand to verify X, Y, Heading, and raw 4-bar pod counts).
 *
 * Controls (Gamepad 1):
 * - A Button: Zero pose to (0, 0, 0 deg)
 * - B Button: Reset pose to (0, 0, 0 deg) and recalibrate IMU (keep robot stationary)
 */
@TeleOp(name = "Test: Pinpoint Odometry Only", group = "Test")
public class PinpointTestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        PinpointOdometry odometry = new PinpointOdometry(hardwareMap);

        telemetry.addLine("Pinpoint Odometry Standalone Test (push robot by hand)");
        odometry.addInitTelemetry(telemetry);
        telemetry.addLine("Gamepad 1: A = Zero Pose | B = Recalibrate IMU & Zero Pose");
        telemetry.update();

        waitForStart();

        boolean previousA = false;
        boolean previousB = false;

        while (opModeIsActive()) {
            boolean currentA = gamepad1.a;
            boolean currentB = gamepad1.b;
            if (currentA && !previousA) {
                odometry.zeroPose();
            }
            if (currentB && !previousB) {
                odometry.resetPoseAndIMU();
            }
            previousA = currentA;
            previousB = currentB;

            odometry.update();
            odometry.addTelemetry(telemetry);
            telemetry.update();
        }
    }
}
