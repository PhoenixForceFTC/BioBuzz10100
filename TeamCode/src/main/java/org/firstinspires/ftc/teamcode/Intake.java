package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {

    private DcMotor IntakeMotor;

    private boolean on = false;

    private boolean direction = true;

    private double power = 1;

    private Telemetry telemetry;

    private HardwareMap hardwareMap;

    private Gamepad gamepad1;

    public Intake(String intake, Telemetry t, HardwareMap h, Gamepad g){

        telemetry = t;

        hardwareMap = h;

        gamepad1 = g;

        IntakeMotor = hardwareMap.get(DcMotor.class, intake);

        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IntakeMotor.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Intake", "Ready");
        telemetry.update();
    }

    public void run(){
        if(gamepad1.left_bumper){
            direction = true;
            power = Math.abs(power);
        }

        if(gamepad1.right_bumper){
            power = -Math.abs(power);
        }

        if(gamepad1.yWasPressed()){
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
