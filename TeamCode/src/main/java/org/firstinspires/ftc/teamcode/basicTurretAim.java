package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class basicTurretAim{

    private Servo turretServo1;
    private Servo turretServo2;
    private Telemetry telemetryInternal;
    private Gamepad gamepad;
    double position = 0.5;
    double speed = 0.005;

    public basicTurretAim(String servo1, String servo2, Telemetry t, HardwareMap h, Gamepad g) {
        turretServo1 = h.get(Servo.class, servo1);
        turretServo2 = h.get(Servo.class, servo2);
        telemetryInternal = t;
        gamepad = g;
    }

    public void run() {
        if(gamepad.left_stick_x > 0.08){
            position = Math.min(1, Math.max(0, position + speed));
        } else if (gamepad.left_stick_x < -0.08){
            position = Math.min(1, Math.max(0, position - speed));
        }

        if(gamepad.aWasPressed()){
            speed = Math.min(0.015, speed+0.001);
        } else if(gamepad.bWasPressed()){
            speed = Math.max(0.001, speed-0.001);
        }

        if(gamepad.xWasPressed()){
            position = 0.75;
        }

        turretServo1.setPosition(position);
        turretServo2.setPosition(position);

        telemetryInternal.addData("turret target position:\t", position);
        telemetryInternal.addData("turret speed:\t", speed);
    }
}
