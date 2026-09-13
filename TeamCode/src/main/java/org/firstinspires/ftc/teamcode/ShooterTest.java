package org.firstinspires.ftc.teamcode;
//Imports

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Shooter;

//Step one
@TeleOp(name = "Shooter")
public class ShooterTest extends LinearOpMode {
    Shooter shooter;
    DcMotor ex;

    @Override
    public void runOpMode() {
        //When driver is in init
        shooter = new Shooter("shooter", telemetry, hardwareMap, gamepad1);
        waitForStart();
        while (opModeIsActive()) {
            shooter.run();
            telemetry.update();
        }
    }
}

