package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

class moveDistanceBasedTime{

    private double currentDistance;
    private double currentSpeed;
    private boolean dispatchedTask = false;
    private double currentMotorSpeed;
    private double timeNeeded;
    private ElapsedTime mainTimer = new ElapsedTime();
    private DcMotorEx[] wheels;

    public moveDistanceBasedTime(DcMotorEx[] wheels){
        this.wheels = wheels;
    }

    public void dispatchNewTask(double distance, double speed, double motorSpeed){
        if(dispatchedTask){
            return;
        }
        dispatchedTask = true;
        currentDistance = distance;
        currentSpeed = speed;
        currentMotorSpeed = motorSpeed;
        try{
            timeNeeded = distance/(speed*motorSpeed);
        } finally{
            timeNeeded = 0;
        }
        mainTimer.reset();
    }

    public void movemotor(){
        if(!dispatchedTask){
            return;
        }

        if(mainTimer.seconds() > timeNeeded){
            dispatchedTask = false;
            for(int i = 0; i < wheels.length; ++i){
                wheels[i].setPower(currentMotorSpeed);
            }
            return;
        }

        for(int i = 0; i < wheels.length; ++i){
            wheels[i].setPower(currentMotorSpeed);
        }

    }

    public boolean isTaskDispatched(){
        return dispatchedTask;
    }

}