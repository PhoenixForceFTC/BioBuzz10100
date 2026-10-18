package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Layer 3 Combined TeleOp: Coordinates {@link MecanumDrivetrain}, {@link PinpointOdometry},
 * and {@link TurretSubsystem} (rotation servos on ports 1 & 2, 108:40 REV Through-Bore Encoder
 * on quad port 1, 2:1 pitch servo on port 0 from 15-0 deg, 6000 RPM flywheel motor on port 0,
 * and Limelight 3A angled 30 deg toward the goal).
 *
 * <p>Controls (Gamepad 1 — Driver + Turret Assist, or Gamepad 2 — Dedicated Turret Operator):
 * <ul>
 *     <li><b>Gamepad 1 Left Stick</b>: Forward / Backward & Strafe translation</li>
 *     <li><b>Gamepad 1 Right Stick X</b>: Chassis rotation</li>
 *     <li><b>Gamepad 1/2 Left Bumper (Hold)</b>: Limelight 3A Auto-Aim (yaw via {@code tx} + auto pitch {@code 15..0 deg} & RPM via {@code ty})</li>
 *     <li><b>Gamepad 1/2 Right Trigger (&gt; 0.1)</b>: Spin 6000 RPM turret shooter motor</li>
 *     <li><b>Gamepad 1/2 D-Pad Left / Right (or GP2 Right Stick X)</b>: Manual turret rotation ({@code -170 deg .. +170 deg}, +/-10 deg left deadzone)</li>
 *     <li><b>Gamepad 1/2 D-Pad Up / Down</b>: Adjust turret pitch angle between {@code 15.0 deg} and {@code 0.0 deg}</li>
 *     <li><b>Gamepad 1 X / Gamepad 2 A</b>: Home turret to center ({@code 0.5 = 0 deg} right), pitch to {@code 15 deg}, & zero Through-Bore Encoder</li>
 *     <li><b>Gamepad 1 A</b>: Zero Pinpoint pose to (0, 0, 0 deg)</li>
 *     <li><b>Gamepad 1 B</b>: Reset Pinpoint pose to (0, 0, 0 deg) and recalibrate IMU (keep robot stationary)</li>
 * </ul>
 */
@TeleOp(name = "Mecanum TeleOp", group = "Drive")
public class MecanumTeleOp extends LinearOpMode {

    private static final double MANUAL_YAW_DEG_PER_LOOP = 1.6;
    private static final double PITCH_STEP_DEG = 0.5;
    private static final double STICK_DEADBAND = 0.08;

    @Override
    public void runOpMode() {
        MecanumDrivetrain drivetrain = new MecanumDrivetrain(hardwareMap);
        PinpointOdometry odometry = new PinpointOdometry(hardwareMap);
        TurretSubsystem turret = null;
        try {
            turret = new TurretSubsystem(hardwareMap);
        } catch (Exception e) {
            telemetry.addData("Turret Warning", "Turret hardware not fully plugged in (%s)", e.getMessage());
        }

        telemetry.addData("Status", "Initialized (Bare Motor: %.0f PPR, %.0f RPM)",
                MecanumDrivetrain.BARE_MOTOR_TICKS_PER_REV, MecanumDrivetrain.BARE_MOTOR_MAX_RPM);
        odometry.addInitTelemetry(telemetry);
        telemetry.addLine("GP1: LS = Drive | RS-X = Turn | A = Zero Pose | B = Recalibrate IMU");
        telemetry.addLine("Turret: LB = Limelight Auto-Aim | RT = Shoot | D-Pad L/R = Yaw | D-Pad U/D = Pitch (15-0 deg)");
        telemetry.update();

        waitForStart();

        boolean previousA = false;
        boolean previousB = false;
        boolean previousTurretHome = false;
        boolean previousDpadUp = false;
        boolean previousDpadDown = false;

        try {
            while (opModeIsActive()) {
                // 1. Update Pinpoint Odometry Computer
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

                // 2. Robot-centric Mecanum Drive on Controller 1
                double axial = -gamepad1.left_stick_y;
                double lateral = gamepad1.left_stick_x;
                double yaw = gamepad1.right_stick_x;

                drivetrain.driveRobotCentric(axial, lateral, yaw);

                // 3. Turret & Limelight 3A Control (supports both Gamepad 1 and Gamepad 2)
                if (turret != null) {
                    boolean currentTurretHome = gamepad1.x || gamepad2.a;
                    if (currentTurretHome && !previousTurretHome) {
                        turret.homeTurretAndZeroEncoder();
                        turret.setPitchAngleDegrees(TurretSubsystem.PITCH_HOME_DEG);
                    }
                    previousTurretHome = currentTurretHome;

                    boolean currentDpadUp = gamepad1.dpad_up || gamepad2.dpad_up;
                    boolean currentDpadDown = gamepad1.dpad_down || gamepad2.dpad_down;
                    if (currentDpadUp && !previousDpadUp) {
                        turret.adjustPitchAngleDegrees(+PITCH_STEP_DEG);
                    }
                    if (currentDpadDown && !previousDpadDown) {
                        turret.adjustPitchAngleDegrees(-PITCH_STEP_DEG);
                    }
                    previousDpadUp = currentDpadUp;
                    previousDpadDown = currentDpadDown;

                    turret.update();

                    double shootTrigger = Math.max(gamepad1.right_trigger, gamepad2.right_trigger);
                    boolean triggerActive = shootTrigger > 0.1;
                    boolean autoAimHeld = gamepad1.left_bumper || gamepad2.left_bumper;

                    if (autoAimHeld) {
                        turret.autoAimWithLimelight(true, triggerActive);
                        if (!triggerActive) {
                            turret.stopShooter();
                        }
                    } else {
                        // Manual turret yaw via Gamepad 2 Right Stick X or Gamepad 1/2 D-Pad Left/Right
                        double manualYawInput = 0.0;
                        if (Math.abs(gamepad2.right_stick_x) > STICK_DEADBAND) {
                            manualYawInput = gamepad2.right_stick_x;
                        } else if (gamepad1.dpad_right || gamepad2.dpad_right) {
                            manualYawInput = 1.0;
                        } else if (gamepad1.dpad_left || gamepad2.dpad_left) {
                            manualYawInput = -1.0;
                        }

                        if (manualYawInput != 0.0) {
                            turret.adjustTargetYawDegrees(manualYawInput * MANUAL_YAW_DEG_PER_LOOP);
                        }

                        if (triggerActive) {
                            turret.setShooterRpm(shootTrigger * TurretSubsystem.SHOOTER_MOTOR_MAX_RPM);
                        } else {
                            turret.stopShooter();
                        }
                    }
                }

                // 4. Flush Telemetry once per loop at the very end
                odometry.addTelemetry(telemetry);
                drivetrain.addTelemetry(telemetry);
                if (turret != null) {
                    turret.addTelemetry(telemetry);
                }
                telemetry.update();
            }
        } finally {
            drivetrain.stop();
            if (turret != null) {
                turret.stop();
            }
        }
    }
}
