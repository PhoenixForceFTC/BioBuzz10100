package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Turret-only diagnostic. A rehomes; B toggles trim; X invalidates home with a diagnostic zero.
 * Y explicitly requests home. RS-X is degrees/second. Dpad adjusts hood; RB spins the flywheel.
 * No Limelight or odometry is initialized; automatic aiming belongs in Test: Auto Aim.
 */
@TeleOp(name = "Test: Turret", group = "Test")
public class TurretTestOpMode extends LinearOpMode {

    private static final double PITCH_STEP_DEG = 0.5;

    @Override
    public void runOpMode() {
        RobotLoop loop = new RobotLoop(hardwareMap);
        TurretSubsystem turret = new TurretSubsystem(hardwareMap, false);

        telemetry.addLine("Manual Turret Standalone Test Initialized");
        telemetry.addLine("Home = All 3 Servos at 0.5 (Turret heading 90 deg Right, Launch +15 deg) | Left 265..275 deg excluded");
        telemetry.addLine("RS-X = Rotate | Dpad Up/Dn = Raise/Lower Launch (+15..-15 deg) | RB = Shooter");
        telemetry.addLine("A = Home (All 0.5) & Zero Enc | B = Toggle Enc Trim | X = Zero Enc Here | Y = Request Home (bounded)");
        turret.addTelemetry(telemetry);
        loop.addTelemetry(telemetry); telemetry.update();

        boolean prevA = false;
        boolean previousCenter = false;
        try {
            while (opModeInInit()) {
                loop.begin();
                if (gamepad1.a && !prevA) turret.homeTurretAndZeroEncoder();
                prevA = gamepad1.a;
                turret.update(); turret.addTelemetry(telemetry); loop.addTelemetry(telemetry); telemetry.update(); idle();
            }
            waitForStart();
            if (isStopRequested()) return;

            boolean prevB = false;
            boolean prevX = false;
            boolean prevDpadUp = false;
            boolean prevDpadDown = false;
            boolean prevDpadLeft = false;
            boolean prevDpadRight = false;

            double previousTime = System.nanoTime() * 1e-9;
            while (opModeIsActive()) {
                loop.begin();
                double now = System.nanoTime() * 1e-9;
                double dt = Math.max(0, Math.min(.1, now - previousTime)); previousTime = now;
                boolean curA = gamepad1.a;
                boolean curB = gamepad1.b;
                boolean curX = gamepad1.x;
                boolean curDpadUp = gamepad1.dpad_up;
                boolean curDpadDown = gamepad1.dpad_down;
                boolean curDpadLeft = gamepad1.dpad_left;
                boolean curDpadRight = gamepad1.dpad_right;

                // Explicit home request on an edge, treating coupled yaw servos as one mechanism.
                if (gamepad1.y && !previousCenter) turret.setAllServosToZeroPosition();
                previousCenter = gamepad1.y;

                // A: Home turret to center (0.5 = robot heading 90 deg) & zero relative encoder
                if (curA && !prevA) {
                    turret.setEncoderCorrectionEnabled(true);
                    turret.homeTurretAndZeroEncoder();
                    turret.setPitchAngleDegrees(TurretSubsystem.PITCH_HOME_DEG);
                }

                // B: Toggle Through-Bore Encoder closed-loop position trim
                if (curB && !prevB) {
                    turret.setEncoderCorrectionEnabled(!turret.isEncoderCorrectionEnabled());
                }

                // X: Zero Through-Bore Encoder at current physical angle
                if (curX && !prevX) {
                    turret.zeroEncoderAtCurrentPosition();
                }

                // Up raises launch angle by reducing downward hood travel; down lowers it.
                if (curDpadUp && !prevDpadUp) {
                    turret.adjustPitchAngleDegrees(-PITCH_STEP_DEG);
                }
                if (curDpadDown && !prevDpadDown) {
                    turret.adjustPitchAngleDegrees(+PITCH_STEP_DEG);
                }
                if (curDpadLeft && !prevDpadLeft) {
                    turret.setPitchAngleDegrees(TurretSubsystem.PITCH_MIN_DEG); // Home: launch +15 deg
                }
                if (curDpadRight && !prevDpadRight) {
                    turret.setPitchAngleDegrees(TurretSubsystem.PITCH_MAX_DEG); // Full CCW: launch -15 deg
                }

                prevA = curA;
                prevB = curB;
                prevX = curX;
                prevDpadUp = curDpadUp;
                prevDpadDown = curDpadDown;
                prevDpadLeft = curDpadLeft;
                prevDpadRight = curDpadRight;

                // Update the encoder-verified turret servo loop (camera disabled).
                turret.update();

                boolean shooterHeld = gamepad1.right_bumper;

                // Manual mechanism controls only; automatic aiming has its own test OpMode.
                {
                    // Manual turret yaw via Right Stick X
                    turret.updateManualYaw(gamepad1.right_stick_x, dt, false);

                    // Manual shooter control via Right Bumper
                    if (shooterHeld) {
                        turret.setShooterRpm(TurretSubsystem.SHOOTER_DEFAULT_TARGET_RPM);
                    } else {
                        turret.stopShooter();
                    }
                    telemetry.addData("Mode", "MANUAL (EncTrim: %s)",
                            turret.isEncoderCorrectionEnabled() ? "ON" : "OFF");
                }

                turret.addTelemetry(telemetry);
                loop.addTelemetry(telemetry); telemetry.update();
            }
        } finally {
            turret.stop();
        }
    }
}
