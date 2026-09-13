package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOp", group = "Drive")
public class TeleOpFullTest extends LinearOpMode {

    Intake intake;
    Shooter shooter;

    Mecanum mecanum;
    PassingMechanism passingMechanism;

    @Override
    public void runOpMode() {

        intake = new Intake("intake", telemetry, hardwareMap, gamepad1);

        shooter = new Shooter("shooter", telemetry, hardwareMap, gamepad1);

        mecanum = new Mecanum(hardwareMap, gamepad1, telemetry);

        passingMechanism = new PassingMechanism("passer", telemetry, hardwareMap, gamepad1);

        waitForStart();

        while (opModeIsActive()) {

            intake.run();
            shooter.run();
            passingMechanism.run();
            mecanum.run();
            telemetry.update();

        }
    }

}