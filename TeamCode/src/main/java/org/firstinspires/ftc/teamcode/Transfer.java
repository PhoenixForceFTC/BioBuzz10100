package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Transfer {

    private DcMotor TransferMotor;

    private boolean on = false;

    private boolean direction = true;

    private double power = 0.6;

    private Telemetry telemetry;

    private HardwareMap hardwareMap;

    private Gamepad gamepad;

    public Transfer(String transfername, Telemetry t, HardwareMap h, Gamepad g){

        telemetry = t;

        hardwareMap = h;

        gamepad = g;

        TransferMotor = hardwareMap.get(DcMotor.class, transfername);

        TransferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        TransferMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void run(){
        if(gamepad.left_bumper){
            direction = true;
            power = Math.abs(power);
        }

        if(gamepad.right_bumper){
            power = -Math.abs(power);
        }

        if(gamepad.dpadRightWasPressed()){
            on = !on;
        }


        if(on){
            TransferMotor.setPower(power);
        }
        else{
            TransferMotor.setPower(0);
        }

    }

}
