package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Layer 2 Standalone Test OpMode: Tests ONLY the {@link TurretSubsystem} in isolation
 * (rotation servos on ports 1 & 2, REV Through-Bore Encoder V1 on quad port 1 at 108:40 ratio,
 * 2:1 pitch servo on port 0 from 15 deg to 0 deg, 6000 RPM flywheel motor on port 0, and
 * Limelight 3A on USB 3.0 angled 30 deg toward the goal).
 *
 * <p>Controls (Gamepad 1):
 * <ul>
 *     <li><b>Right Stick X</b>: Manual turret rotation speed (clamped to {@code -170 deg .. +170 deg}, +/-10 deg left deadzone)</li>
 *     <li><b>D-Pad Up / Down</b>: Step turret pitch angle between {@code 15.0 deg} and {@code 0.0 deg} (0.5 deg steps)</li>
 *     <li><b>D-Pad Left / Right</b>: Jump turret pitch directly to {@code 15.0 deg} (Left) or {@code 0.0 deg} (Right)</li>
 *     <li><b>Right Trigger (&gt; 0.1)</b>: Spin 6000 RPM turret shooter motor (proportional or closed-loop RPM)</li>
 *     <li><b>Left Bumper (Hold)</b>: Limelight 3A Auto-Aim (tracks goal yaw via {@code tx} + auto pitch {@code 15..0 deg} & RPM via {@code ty})</li>
 *     <li><b>A Button</b>: Home turret to center ({@code 0.5 = 0 deg} right), pitch to {@code 15 deg}, and zero Through-Bore Encoder</li>
 *     <li><b>B Button</b>: Toggle Through-Bore Encoder closed-loop trim ON / OFF</li>
 *     <li><b>X Button</b>: Re-zero Through-Bore Encoder at current turret position</li>
 *     <li><b>Y Button (Hold)</b>: Drive all servos to {@code 0.0} raw zero position</li>
 * </ul>
 */
@TeleOp(name = "Test: Turret & Limelight Only", group = "Test")
public class TurretTestOpMode extends LinearOpMode {

    private static final double MANUAL_YAW_DEG_PER_LOOP = 1.8; // ~90-180 deg/sec at 50-100 Hz
    private static final double PITCH_STEP_DEG = 0.5;
    private static final double STICK_DEADBAND = 0.08;

    @Override
    public void runOpMode() {
        TurretSubsystem turret = new TurretSubsystem(hardwareMap);

        telemetry.addLine("Turret & Limelight 3A Standalone Test Initialized");
        telemetry.addLine("Home = 0.5 Servo (0 deg Right) | Left Deadzone = +/-10 deg (-170..+170 deg)");
        telemetry.addLine("RS-X = Rotate | Dpad Up/Dn = Pitch (15-0 deg) | RT = Shooter | LB = Limelight Auto-Aim");
        telemetry.addLine("A = Home & Zero Enc | B = Toggle Enc Trim | X = Zero Enc Here | Hold Y = All Servos 0.0");
        turret.addTelemetry(telemetry);
        telemetry.update();

        waitForStart();

        boolean prevA = false;
        boolean prevB = false;
        boolean prevX = false;
        boolean prevDpadUp = false;
        boolean prevDpadDown = false;
        boolean prevDpadLeft = false;
        boolean prevDpadRight = false;

        try {
            while (opModeIsActive()) {
                boolean curA = gamepad1.a;
                boolean curB = gamepad1.b;
                boolean curX = gamepad1.x;
                boolean curDpadUp = gamepad1.dpad_up;
                boolean curDpadDown = gamepad1.dpad_down;
                boolean curDpadLeft = gamepad1.dpad_left;
                boolean curDpadRight = gamepad1.dpad_right;

                // Hold Y: raw 0.0 servo calibration override
                if (gamepad1.y) {
                    turret.setAllServosToZeroPosition();
                    turret.stopShooter();
                    telemetry.addLine("MODE: RAW SERVO ZERO OVERRIDE (All Servos = 0.0)");
                    turret.addTelemetry(telemetry);
                    telemetry.update();
                    continue;
                }

                // A: Home turret to center (0.5 = 0 deg) & zero Through-Bore Encoder
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

                // D-Pad Pitch adjustments (15 deg down to 0 deg, 2:1 linkage)
                if (curDpadUp && !prevDpadUp) {
                    turret.adjustPitchAngleDegrees(+PITCH_STEP_DEG);
                }
                if (curDpadDown && !prevDpadDown) {
                    turret.adjustPitchAngleDegrees(-PITCH_STEP_DEG);
                }
                if (curDpadLeft && !prevDpadLeft) {
                    turret.setPitchAngleDegrees(TurretSubsystem.PITCH_MAX_DEG); // 15.0 deg
                }
                if (curDpadRight && !prevDpadRight) {
                    turret.setPitchAngleDegrees(TurretSubsystem.PITCH_MIN_DEG); // 0.0 deg
                }

                prevA = curA;
                prevB = curB;
                prevX = curX;
                prevDpadUp = curDpadUp;
                prevDpadDown = curDpadDown;
                prevDpadLeft = curDpadLeft;
                prevDpadRight = curDpadRight;

                // Update Limelight & encoder-verified turret servo loop
                turret.update();

                boolean triggerActive = gamepad1.right_trigger > 0.1;

                // Left Bumper: Limelight 3A Auto-Aim (tx yaw + ty distance pitch & RPM)
                if (gamepad1.left_bumper) {
                    boolean locked = turret.autoAimWithLimelight(true, triggerActive);
                    if (!triggerActive) {
                        turret.stopShooter();
                    }
                    telemetry.addData("Mode", locked
                            ? "LIMELIGHT AUTO-AIM (Locked onto Goal)"
                            : "LIMELIGHT AUTO-AIM (Searching for Target...)");
                } else {
                    // Manual turret yaw via Right Stick X
                    double stickX = gamepad1.right_stick_x;
                    if (Math.abs(stickX) > STICK_DEADBAND) {
                        turret.adjustTargetYawDegrees(stickX * MANUAL_YAW_DEG_PER_LOOP);
                    }

                    // Manual shooter control via Right Trigger
                    if (triggerActive) {
                        double targetRpm = gamepad1.right_bumper
                                ? TurretSubsystem.SHOOTER_DEFAULT_TARGET_RPM
                                : gamepad1.right_trigger * TurretSubsystem.SHOOTER_MOTOR_MAX_RPM;
                        turret.setShooterRpm(targetRpm);
                    } else {
                        turret.stopShooter();
                    }
                    telemetry.addData("Mode", "MANUAL (EncTrim: %s)",
                            turret.isEncoderCorrectionEnabled() ? "ON" : "OFF");
                }

                turret.addTelemetry(telemetry);
                telemetry.update();
            }
        } finally {
            turret.stop();
        }
    }
}
