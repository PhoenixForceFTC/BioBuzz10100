package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.List;

/**
 * Calibration for every positional Servo in the active Robot Configuration, on every hub.
 * Start / A commands the normalized midpoint (0.5); X commands 0.0; Y commands 1.0.
 * The midpoint is a calibration reference, not each mechanism's individual home position.
 * Turret rotation, pitch, encoder, and shooter tests belong in Test: Turret.
 */
@TeleOp(name = "Test: Set All Servos to 0", group = "Test")
public class ServoZeroCalibrationOpMode extends LinearOpMode {

    private static final double CENTER_SERVO_POS = 0.5;
    private static final double MIN_SERVO_POS = 0.0;
    private static final double MAX_SERVO_POS = 1.0;

    @Override
    public void runOpMode() {
        List<Servo> configuredServos = hardwareMap.getAll(Servo.class);

        // Apply known hardware settings without constructing subsystems or moving servos in INIT.
        configureServo(TurretSubsystem.PITCH_SERVO_DIRECTION,
                TurretSubsystem.AXON_MIN_PULSE_US, TurretSubsystem.AXON_MAX_PULSE_US,
                TurretSubsystem.HW_PITCH_SERVO, "turretPitch", "hood_servo", "pitch_servo");
        configureServo(TurretSubsystem.ROTATION_SERVO_1_DIRECTION,
                TurretSubsystem.AXON_MIN_PULSE_US, TurretSubsystem.AXON_MAX_PULSE_US,
                TurretSubsystem.HW_ROTATION_SERVO_1, "turretServo1", "turret_left", "turret1");
        configureServo(TurretSubsystem.ROTATION_SERVO_2_DIRECTION,
                TurretSubsystem.AXON_MIN_PULSE_US, TurretSubsystem.AXON_MAX_PULSE_US,
                TurretSubsystem.HW_ROTATION_SERVO_2, "turretServo2", "turret_right", "turret2");
        configureServo(GateSubsystem.SERVO_DIRECTION,
                GateSubsystem.MIN_PULSE_US, GateSubsystem.MAX_PULSE_US,
                GateSubsystem.HW_GATE_SERVO);
        configureServo(DepollinatorSubsystem.SERVO_DIRECTION,
                DepollinatorSubsystem.MIN_PULSE_US, DepollinatorSubsystem.MAX_PULSE_US,
                DepollinatorSubsystem.HW_DEPOLLINATOR);

        String[] servoLabels = new String[configuredServos.size()];
        telemetry.addLine("All configured positional servos: Start / A = 0.5 midpoint | X = 0.0 | Y = 1.0");
        telemetry.addData("Servo devices found", configuredServos.size());
        if (configuredServos.isEmpty()) {
            telemetry.addLine("No Servo devices are in the active Robot Configuration.");
            telemetry.addLine("Save/select the correct RC configuration and check Hub/port assignments.");
        }
        for (int i = 0; i < configuredServos.size(); i++) {
            Servo servo = configuredServos.get(i);
            servoLabels[i] = hardwareMap.getNamesOf(servo)
                    + " | " + servo.getController().getConnectionInfo()
                    + " | port " + servo.getPortNumber();
            telemetry.addData("Servo " + i, servoLabels[i]);
        }
        telemetry.addLine("Press PLAY to command every listed servo to 0.5 (midpoint).");
        telemetry.update();

        waitForStart();

        double targetPos = CENTER_SERVO_POS;
        while (opModeIsActive()) {
            if (gamepad1.a) {
                targetPos = CENTER_SERVO_POS;
            } else if (gamepad1.x) {
                targetPos = MIN_SERVO_POS;
            } else if (gamepad1.y) {
                targetPos = MAX_SERVO_POS;
            }

            for (int i = 0; i < configuredServos.size(); i++) {
                configuredServos.get(i).setPosition(targetPos);
                telemetry.addData("Servo " + i, "%s | command=%.2f", servoLabels[i], targetPos);
            }
            telemetry.addData("Servo devices controlled", configuredServos.size());
            telemetry.addData("All Servo Target (0..1)", "%.2f", targetPos);
            telemetry.addLine("A = All 0.5 (Midpoint) | X = All 0.0 | Y = All 1.0");
            telemetry.update();
        }
    }

    private void configureServo(Servo.Direction direction, int minPulseUs, int maxPulseUs,
                                String... names) {
        for (String name : names) {
            Servo servo = hardwareMap.tryGet(Servo.class, name);
            if (servo != null) {
                servo.setDirection(direction);
                if (servo instanceof ServoImplEx) {
                    ((ServoImplEx) servo).setPwmRange(new PwmControl.PwmRange(minPulseUs, maxPulseUs));
                }
                return;
            }
        }
    }
}
