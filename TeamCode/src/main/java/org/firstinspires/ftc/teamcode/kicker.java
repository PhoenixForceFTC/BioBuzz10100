package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class kicker {

    DcMotorEx kickerWheel;
    Telemetry telemetry;
    Gamepad gamepad;
    double reverseSpeed = -0.3;
    double forwardSpeed = 0.3;
    double zeroSpeed = 0;

    public kicker(String name, Telemetry t, HardwareMap h, Gamepad g){
        kickerWheel = h.get(DcMotorEx.class, name);
        kickerWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        kickerWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        kickerWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry = t;
        gamepad = g;
    }

    public void run(){
        if(gamepad.leftBumperWasPressed()){
            kickerWheel.setPower(forwardSpeed);
        }
        else if(gamepad.rightBumperWasPressed()){
            kickerWheel.setPower(reverseSpeed);
        }
        else if(gamepad.leftTriggerWasPressed()){
            kickerWheel.setPower(zeroSpeed);
        }
    }

}
