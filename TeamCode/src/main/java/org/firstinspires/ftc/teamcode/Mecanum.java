package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Mecanum", group = "Drive")
public class Mecanum {

    private HardwareMap hardwareMap;

    private Gamepad gamepad1;

    private Telemetry telemetry;

    private boolean fast = false;

    private DcMotorEx leftFront;

    private DcMotorEx leftBack;

    private DcMotorEx rightFront;

    private DcMotorEx rightBack;

    private double LFpower = 0, LBpower = 0, RFpower = 0, RBpower = 0;

    public Mecanum(HardwareMap h, Gamepad g, Telemetry t) {
        hardwareMap = h;
        gamepad1 = g;
        telemetry = t;

        leftFront = hardwareMap.get(DcMotorEx.class, "leftfront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftback");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightfront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightback");

        leftFront.setPower(LFpower);
        leftBack.setPower(LBpower);
        rightFront.setPower(RFpower);
        rightBack.setPower(RBpower);

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addLine("Verify directions/encoder pairing with Mecanum Wheel Test first");
        telemetry.addLine("Left stick: drive/strafe | Right stick X: rotate");
        telemetry.addLine("Hold left bumper for 25% driving speed");
        telemetry.setMsTransmissionInterval(100);
        telemetry.update();

    }

    public void run(){

        double drive = Math.abs(gamepad1.left_stick_y) <= 0.05 ? 0 : -gamepad1.left_stick_y;
        double strafe = Math.abs(gamepad1.left_stick_y) <= 0.05 ? 0 : gamepad1.left_stick_y;
        double turn = Math.abs(gamepad1.right_stick_x) <= 0.05 ? 0 : gamepad1.right_stick_x;
        double total = Math.abs(drive) + Math.abs(strafe) + Math.abs(turn);

        if(gamepad1.leftBumperWasPressed()){
            fast = !fast;
        }

        try{
            LFpower = (drive + strafe + turn)/total;
            RFpower = (drive - strafe - turn)/total;
            LBpower = (drive - strafe + turn)/total;
            RBpower = (drive + strafe - turn)/total;
        } catch (Exception e) {
            LFpower = 0;
            RFpower = 0;
            LBpower = 0;
            RBpower = 0;
        }

        if(fast){
            leftFront.setPower(LFpower);
            leftBack.setPower(LBpower);
            rightFront.setPower(RFpower);
            rightBack.setPower(RBpower);

        } else if (!fast){
            leftFront.setPower(0.6*LFpower);
            leftBack.setPower(0.6*LBpower);
            rightFront.setPower(0.6*RFpower);
            rightBack.setPower(0.6*RBpower);
        }
    }

    private void setEncoderMode(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

}
