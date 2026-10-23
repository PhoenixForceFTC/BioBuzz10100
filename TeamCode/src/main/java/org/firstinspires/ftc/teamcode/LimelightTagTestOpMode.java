package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Tests only the Limelight; no motors, servos, or odometry are initialized. */
@TeleOp(name = "Test: Limelight AprilTags Only", group = "Test")
public class LimelightTagTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        try (LimelightTagSubsystem vision = new LimelightTagSubsystem(hardwareMap)) {
            while (opModeInInit()) {
                vision.update();
                telemetry.addData("AprilTag pipeline", LimelightTagSubsystem.APRILTAG_PIPELINE);
                telemetry.addLine("Configure this pipeline for AprilTags; all reported tag IDs are accepted.");
                telemetry.addLine("After START: GP1 A starts/restarts a button-to-detection trial.");
                vision.addTelemetry(telemetry);
                telemetry.update();
                idle();
            }

            waitForStart();

            boolean previousA = gamepad1.a;
            boolean waitingForTag = false;
            long trialStartedNs = 0;
            double frameAtPress = Double.NEGATIVE_INFINITY;
            long trialCount = 0;
            double lastDelayMs = 0;
            double averageDelayMs = 0;

            while (opModeIsActive()) {
                boolean currentA = gamepad1.a;
                if (currentA && !previousA) {
                    trialStartedNs = System.nanoTime();
                    frameAtPress = vision.getFrameTimestamp();
                    waitingForTag = true;
                }
                previousA = currentA;

                vision.update();
                // A cached frame from before the button press must not finish a new trial.
                if (waitingForTag && vision.getFrameTimestamp() > frameAtPress
                        && vision.hasDetectedTag()) {
                    lastDelayMs = (System.nanoTime() - trialStartedNs) / 1_000_000.0;
                    trialCount++;
                    averageDelayMs += (lastDelayMs - averageDelayMs) / trialCount;
                    waitingForTag = false;
                }

                telemetry.addLine("GP1 A: start/restart trial | Present a tag at the button press");
                telemetry.addLine("If a tag is already visible, this times the next detected frame.");
                telemetry.addLine("Button-to-detection includes presentation delay; not calibrated camera latency.");
                telemetry.addData("Trial", waitingForTag ? "Waiting for tag" : "Press A to start");
                if (waitingForTag) {
                    telemetry.addData("Elapsed (ms)", "%.1f", (System.nanoTime() - trialStartedNs) / 1_000_000.0);
                }
                telemetry.addData("Completed trials", trialCount);
                if (trialCount > 0) {
                    telemetry.addData("Last button-to-detection (ms)", "%.1f", lastDelayMs);
                    telemetry.addData("Average button-to-detection (ms)", "%.1f", averageDelayMs);
                } else {
                    telemetry.addData("Last / average (ms)", "No completed trials");
                }
                vision.addTelemetry(telemetry);
                telemetry.update();
                idle();
            }
        }
    }
}
