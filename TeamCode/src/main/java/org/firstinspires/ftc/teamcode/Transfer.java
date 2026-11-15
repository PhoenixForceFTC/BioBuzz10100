package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Transfer {

    private DcMotor TransferMotor;

    private boolean on = false;

    //private boolean direction = true;

    private double forwardPower = 1.0;
    private double reversePower = -1.0;

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

    public void forward(){
        TransferMotor.setPower(forwardPower);
    }

    public void backward(){
        TransferMotor.setPower(reversePower);
    }

    public void stop(){
        TransferMotor.setPower(0);
    }

    public void run(){
        if(gamepad.left_trigger > 0.05){
            //direction = true;
            forward();
        }

        if(gamepad.right_trigger > 0.05){
            backward();
        }

//        if(gamepad.yWasPressed()){
//            stop();
//        }


//        if(on){
//            TransferMotor.setPower(power);
//        }
//        else{
//            TransferMotor.setPower(0);
//        }

    }

}
