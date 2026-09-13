package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "intaketest")
public class IntakeTest extends LinearOpMode {

     Intake intake;

    @Override
    public void runOpMode(){
        intake = new Intake("intake", telemetry, hardwareMap, gamepad1);

        waitForStart();

        while(opModeIsActive()){
            intake.run();
            telemetry.update();
        }

    }
}
