package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Tests only the intake motor, without requiring drivetrain, odometry, or turret hardware. */
@TeleOp(name = "Test: Intake Only", group = "Test")
public class IntakeTestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);

        telemetry.addLine("Intake: Expansion Hub motor port 1, intake_motor (no intake encoder)");
        telemetry.addLine("GP1 RB = Forward (Motor CCW) | LB = Reverse | Release/Both = Stop");
        telemetry.update();

        waitForStart();

        try {
            while (opModeIsActive()) {
                intake.setBumperInputs(gamepad1.right_bumper, gamepad1.left_bumper);
                intake.addTelemetry(telemetry);
                telemetry.update();
            }
        } finally {
            intake.stop();
        }
    }
}
