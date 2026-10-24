package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;

/** Raw programmer reference ONLY for one mechanically disconnected positional servo.
 * Installed mechanisms use Test: Turret / Gate / Depollinator with subsystem bounds.
 */
@TeleOp(name = "Calibrate: DISCONNECTED Servo RAW", group = "Calibration")
public class ServoZeroCalibrationOpMode extends LinearOpMode {
    @Override public void runOpMode() {
        RobotLoop loop = new RobotLoop(hardwareMap);
        List<Servo> servos = hardwareMap.getAll(Servo.class);
        int selected = 0;
        boolean previousLeft = false, previousRight = false;
        boolean armed = false, previousArm = false;
        while (opModeInInit() || opModeIsActive()) {
            loop.begin();
            boolean left = gamepad1.dpad_left, right = gamepad1.dpad_right;
            if (!servos.isEmpty()) {
                if (left && !previousLeft) { selected = (selected + servos.size() - 1) % servos.size(); armed = false; }
                if (right && !previousRight) { selected = (selected + 1) % servos.size(); armed = false; }
                Servo servo = servos.get(selected);
                boolean arm = gamepad1.left_bumper && gamepad1.right_bumper;
                if (opModeIsActive() && arm && !previousArm) armed = !armed;
                if (!opModeIsActive()) armed = false;
                previousArm = arm;
                // Coupled installed turret servos must NEVER be exercised independently.
                if (opModeIsActive() && armed && gamepad1.right_trigger > .8) {
                    if (gamepad1.a) servo.setPosition(.5);
                    else if (gamepad1.x) servo.setPosition(0);
                    else if (gamepad1.y) servo.setPosition(1);
                }
                telemetry.addData("Selected ONLY", "%s | %s | port %d", hardwareMap.getNamesOf(servo),
                        servo.getController().getConnectionInfo(), servo.getPortNumber());
            }
            previousLeft = left; previousRight = right;
            telemetry.addLine("Disconnect selected servo from ALL linkages, including coupled turret gears.");
            telemetry.addLine("Installed mechanisms: use their bounded subsystem tests.");
            telemetry.addLine("Dpad L/R selects | AFTER START both bumpers toggle arm | hold RT + A/X/Y = .5/0/1");
            telemetry.addData("Raw calibration armed", armed);
            loop.addTelemetry(telemetry); telemetry.update(); idle();
        }
        // No return-to-center move at STOP: reference is unknown for a disconnected mechanism.
    }
}
