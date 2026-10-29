package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Zero Servo")
public class zeroServo extends LinearOpMode{
    Servo targetServo1;
    Servo targetServo2;
    boolean twoServos = true;

    @Override
    public void runOpMode() {
        targetServo1 = hardwareMap.get(Servo.class, "turret_servo_1");
        if(twoServos){targetServo2 = hardwareMap.get(Servo.class, "turret_servo_2");}

        waitForStart();

        while(opModeIsActive()){
            targetServo1.setPosition(0.5);
            if(twoServos){targetServo2.setPosition(0.5);}
            telemetry.update();
        }

    }
}
