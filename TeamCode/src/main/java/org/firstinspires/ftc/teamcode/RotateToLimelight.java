package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RotateToLimelight {

    // ---- Target rotation to hold, in degrees ----
    private double targetrotation = 0; // should always be zero, te look straight on at april tag I think.

    private boolean turnDispatched = false;

    // ---- PID gains: tune these ----
    private double kP = 0.012;
    private double kI = 0.0;

    private double kD = 0.0008;

    // Max motor power the PID loop is allowed to command, 0-1.
    private double MAX_POWER = 0.6;

    // Purely for the "at target" telemetry flag.
    private double TOLERANCE_DEG = 1.0;

    private DcMotorEx leftFront, leftBack, rightFront, rightBack;

    private double currentrotation = 0;
    private double integralSum = 0;
    private double lastError = 0;

    private HardwareMap hardwareMap;

    private ElapsedTime pidTimer = new ElapsedTime();

    public RotateToLimelight (HardwareMap h, Telemetry t) {

        hardwareMap = h;

        leftFront = hardwareMap.get(DcMotorEx.class, "leftfront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftback");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightfront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightback");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void dispatchTurn(LimelightData d){
        if(turnDispatched){
            return;
        }

        turnDispatched = true;

        currentrotation = d.getAngleX();

        integralSum = 0;

        lastError = 0;

        pidTimer.reset();
    }

    public void turn(LimelightData d){

        if(!turnDispatched){
            return;
        }

        currentrotation = d.getAngleX();

        double error = wrapAngle(targetrotation - currentrotation);

        if(error <= TOLERANCE_DEG){
            turnDispatched = false;
            return;
        }

        double dt = pidTimer.seconds();
        pidTimer.reset();
        if (dt <= 0) dt = 1e-3;

        integralSum += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double turnPower = (kP * error) + (kI * integralSum) + (kD * derivative);
        turnPower = clip(turnPower, -MAX_POWER, MAX_POWER);

        // Positive turnPower -> spin toward increasing rotation (CCW).
        leftFront.setPower(-turnPower);
        leftBack.setPower(-turnPower);
        rightFront.setPower(turnPower);
        rightBack.setPower(turnPower);
    }

    private double wrapAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    private double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    };

}