package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {

    private DcMotor IntakeMotor;

    private boolean on = false;

    private boolean direction = true;

    private double power = 0.6;

    private Telemetry telemetry;

    private HardwareMap hardwareMap;

    private Gamepad gamepad;

    public Intake(String intake, Telemetry t, HardwareMap h, Gamepad g){

        telemetry = t;

        hardwareMap = h;

        gamepad = g;

        IntakeMotor = hardwareMap.get(DcMotor.class, intake);

        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IntakeMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    public void run(){
        if(gamepad.left_bumper){
            direction = true;
            power = Math.abs(power);
        }

        if(gamepad.right_bumper){
            power = -Math.abs(power);
        }

        if(gamepad.yWasPressed()){
            on = !on;
        }


        if(on){
            IntakeMotor.setPower(power);
        }
        else{
            IntakeMotor.setPower(0);
        }

    }

}
