package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class gate {

    private Servo gate;
    private Telemetry telemetryInternal;
    private Gamepad gamepad;
    boolean isClosed = true;
    double closed = 0.5;
    double open = 0.7;

    public gate(String servo1, Telemetry t, HardwareMap h, Gamepad g) {
        gate = h.get(Servo.class, servo1);
        telemetryInternal = t;
        gamepad = g;
    }

    public void run() {
        if(gamepad.dpadLeftWasPressed()){
            isClosed = !isClosed;
        }

        if(isClosed){
            gate.setPosition(closed);
        } else {
            gate.setPosition(open);
        }

        telemetryInternal.addData("closed:\t", isClosed);
    }

    public void open(){
        isClosed = false;
        gate.setPosition(open);
    }

    public void close(){
        isClosed = true;
        gate.setPosition(closed);
    }

    // automatically opens/closes gate, for combined shooter
    public void toggle() {
        isClosed = !isClosed;

        if(isClosed){
            gate.setPosition(closed);
        } else {
            gate.setPosition(open);
        }

        telemetryInternal.addData("closed:\t", isClosed);
    }
}
