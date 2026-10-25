package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="Basic Turret Aim Test")
public class basicTurretAimTest extends LinearOpMode{
    basicTurretAim turret;

    @Override
    public void runOpMode() {
        turret = new basicTurretAim("turret_servo_1", "turret_servo_2", telemetry, hardwareMap, gamepad2);

        waitForStart();

        while(opModeIsActive()){
            turret.run();
            telemetry.update();
        }

    }
}
