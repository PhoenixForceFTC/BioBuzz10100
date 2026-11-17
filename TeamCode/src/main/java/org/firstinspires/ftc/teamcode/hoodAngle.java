package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class hoodAngle {

    private Servo hoodServo;

    private Telemetry telemetryInternal;
    private Gamepad gamepad;
    double position = 0.7;
    double speed = 0.002;

    public hoodAngle(String servo1, Telemetry t, HardwareMap h, Gamepad g) {
        hoodServo = h.get(Servo.class, servo1);
        telemetryInternal = t;
        gamepad = g;
    }

    public void moveToPosition(double targetPosition){
        position = targetPosition;
    }

    public void run() {
        if(gamepad.right_stick_y > 0.08){
            position = Math.min(0.7, Math.max(0.5, position + speed));
        } else if (gamepad.right_stick_y < -0.08){
            position = Math.min(0.7, Math.max(0.5, position - speed));
        }

//        if(gamepad.aWasPressed()){
//            speed = Math.min(0.015, speed+0.001);
//        } else if(gamepad.bWasPressed()){
//            speed = Math.max(0.001, speed-0.001);
//        }

//        if(gamepad.xWasPressed()){
//            position = 0.75;
//        }

        hoodServo.setPosition(position);
        //turretServo2.setPosition(position);

        telemetryInternal.addData("hood target position:\t", position);
        telemetryInternal.addData("hood speed:\t", speed);
    }
}
