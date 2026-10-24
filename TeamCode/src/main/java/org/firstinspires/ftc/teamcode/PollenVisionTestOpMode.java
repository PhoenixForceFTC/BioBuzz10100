package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Camera-only test: no drivetrain, intake, turret, or odometry hardware is initialized. */
@TeleOp(name = "Test: Pollen Camera Calibration", group = "Test")
public class PollenVisionTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotLoop loop = new RobotLoop(hardwareMap);
        try (PollenVision vision = new PollenVision(hardwareMap)) {
            while (opModeInInit()) {
                loop.begin();
                telemetry.addLine("Calibration support: record contact pixels, measured floor range, model and intrinsics.");
                telemetry.addLine("Measure floor distance from robot center; tune intrinsics, offsets, and color.");
                vision.addTelemetry(telemetry);
                loop.addTelemetry(telemetry); telemetry.update();
                idle();
            }
            waitForStart();
            while (opModeIsActive()) {
                loop.begin();
                vision.addTelemetry(telemetry);
                loop.addTelemetry(telemetry); telemetry.update();
                idle();
            }
        }
    }
}
