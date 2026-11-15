package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Descorer {

    private Servo descorerServo;
    private Telemetry telemetryInternal;
    private Gamepad gamepad;
    double upPosition = 0.5;
    double downPosition = 0.2;
    double position = 0.5;



    public Descorer(String servo1, Telemetry t, HardwareMap h, Gamepad g) {
        descorerServo = h.get(Servo.class, servo1);
        telemetryInternal = t;
        gamepad = g;
    }

    public void run() {
        if(gamepad.dpadUpWasPressed()){
            position = upPosition;
        }
        if(gamepad.dpadDownWasPressed()){
            position = downPosition;
        }

//        if(gamepad.xWasPressed()){
//            position = 0.75;
//        }

        descorerServo.setPosition(position);

        telemetryInternal.addData("descorer position:\t", position);
    }
}
