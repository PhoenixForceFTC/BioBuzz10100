package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/**
 * Layer 2 Standalone Test OpMode: Commands all Turret Axon MAX servos to {@code 0.0} on start
 * so you can install servo horns/gears at a known reference, and lets you toggle between
 * {@code 0.0} (raw zero) and {@code 0.5} (turret center home = 0 deg, opposite the +/-10 deg left deadzone).
 *
 * <p>Hardware used (Expansion Hub via goBILDA 6V Power Distribution Hub):
 * <ul>
 *     <li>Servo Port 0: {@code "turret_pitch"} (Axon MAX 2:1 Pitch Servo, 0.0 = 15 deg)</li>
 *     <li>Servo Port 1: {@code "turret_servo_1"} (Axon MAX 1:1 Rotation Servo 1)</li>
 *     <li>Servo Port 2: {@code "turret_servo_2"} (Axon MAX 1:1 Rotation Servo 2)</li>
 * </ul>
 *
 * <p>Controls (Gamepad 1):
 * <ul>
 *     <li>Default on Start / A Button: Set ALL servos (ports 0, 1, 2) to {@code 0.0}</li>
 *     <li>B Button: Set Rotation Servos (ports 1 & 2) to {@code 0.5} (Turret 0 deg Home, right side) and Pitch (port 0) to {@code 0.0} (15 deg)</li>
 *     <li>X Button: Set Pitch Servo (port 0) to {@code 0.0845} (0 deg flat pitch via 2:1 linkage)</li>
 *     <li>Y Button: Set ALL servos (ports 0, 1, 2) to {@code 1.0}</li>
 * </ul>
 */
@TeleOp(name = "Test: Set All Servos to 0", group = "Test")
public class ServoZeroCalibrationOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        Servo pitchServo = getServo("turret_pitch", "turretPitch", "hood_servo", "pitch_servo");
        Servo rotationServo1 = getServo("turret_servo_1", "turretServo1", "turret_left", "turret1");
        Servo rotationServo2 = getServo("turret_servo_2", "turretServo2", "turret_right", "turret2");

        configureAxonServo(pitchServo, TurretSubsystem.PITCH_SERVO_DIRECTION);
        configureAxonServo(rotationServo1, TurretSubsystem.ROTATION_SERVO_1_DIRECTION);
        configureAxonServo(rotationServo2, TurretSubsystem.ROTATION_SERVO_2_DIRECTION);

        telemetry.addLine("Test: Set All Turret Axon MAX Servos to 0.0");
        telemetry.addData("Port 0 (Pitch)", pitchServo != null ? "Connected (turret_pitch)" : "MISSING");
        telemetry.addData("Port 1 (Rot 1)", rotationServo1 != null ? "Connected (turret_servo_1)" : "MISSING");
        telemetry.addData("Port 2 (Rot 2)", rotationServo2 != null ? "Connected (turret_servo_2)" : "MISSING");
        telemetry.addLine("Press PLAY to drive all servos to 0.0 immediately.");
        telemetry.update();

        waitForStart();

        double rotTargetPos = 0.0;
        double pitchTargetPos = 0.0;
        String modeLabel = "ALL SERVOS = 0.0 (Raw Zero)";

        while (opModeIsActive()) {
            if (gamepad1.a) {
                rotTargetPos = 0.0;
                pitchTargetPos = 0.0;
                modeLabel = "ALL SERVOS = 0.0 (Raw Zero)";
            } else if (gamepad1.b) {
                rotTargetPos = TurretSubsystem.ROTATION_HOME_SERVO_POS; // 0.5 center (0 deg right, deadzone left)
                pitchTargetPos = TurretSubsystem.PITCH_SERVO_POS_AT_15_DEG; // 0.0 (15 deg pitch)
                modeLabel = "TURRET HOME: Rot=0.5 (0 deg Right), Pitch=0.0 (15 deg)";
            } else if (gamepad1.x) {
                rotTargetPos = TurretSubsystem.ROTATION_HOME_SERVO_POS;
                pitchTargetPos = TurretSubsystem.pitchDegreesToServoPos(0.0); // 0 deg pitch (30 deg servo travel)
                modeLabel = "PITCH 0 DEG TEST: Rot=0.5, Pitch=" + String.format("%.4f", pitchTargetPos) + " (0 deg)";
            } else if (gamepad1.y) {
                rotTargetPos = 1.0;
                pitchTargetPos = 1.0;
                modeLabel = "ALL SERVOS = 1.0 (Full Max)";
            }

            if (pitchServo != null) {
                pitchServo.setPosition(pitchTargetPos);
            }
            if (rotationServo1 != null) {
                rotationServo1.setPosition(rotTargetPos);
            }
            if (rotationServo2 != null) {
                rotationServo2.setPosition(rotTargetPos);
            }

            telemetry.addData("Mode", modeLabel);
            telemetry.addData("Pitch Servo (Port 0)", "%.4f", pitchTargetPos);
            telemetry.addData("Rotation Servos (Ports 1 & 2)", "%.4f", rotTargetPos);
            telemetry.addLine("Controls: A = All 0.0 | B = Center Rot (0.5) & Pitch 15 deg (0.0) | X = Pitch 0 deg | Y = All 1.0");
            telemetry.update();
        }
    }

    private Servo getServo(String... names) {
        for (String name : names) {
            Servo s = hardwareMap.tryGet(Servo.class, name);
            if (s != null) {
                return s;
            }
        }
        return null;
    }

    private void configureAxonServo(Servo servo, Servo.Direction direction) {
        if (servo == null) {
            return;
        }
        servo.setDirection(direction);
        if (servo instanceof ServoImplEx) {
            ((ServoImplEx) servo).setPwmRange(
                    new PwmControl.PwmRange(
                            TurretSubsystem.AXON_MIN_PULSE_US,
                            TurretSubsystem.AXON_MAX_PULSE_US));
        }
    }
}
