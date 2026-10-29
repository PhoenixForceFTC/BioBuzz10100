package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class ShooterMotor {
    final DcMotorEx shooter;

    private final Gamepad gamepad;
    public final Telemetry telemetry;

    public static Integer ticksPerRotation = 28;

    public double speed;
    public int speedRPM;


    public static double kP = 10;
    public static double kI = 2;
    public static double kD = 0;
    public static double kF = 12;

    public ShooterMotor(String shootername, Telemetry t, HardwareMap h, Gamepad g){
        shooter = h.get(DcMotorEx.class, shootername);
        telemetry = t;
        gamepad = g;
        shooter.setDirection(DcMotorEx.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setVelocityPIDFCoefficients(kP, kI, kD, kF);
    }

    public void shoot(){

        if(gamepad.dpadUpWasPressed()){
            speedRPM = Math.min(6000, speedRPM+10);
        }else if(gamepad.dpadDownWasPressed()){
            speedRPM = Math.max(0, speedRPM-10);
        }

        shooter.setVelocity(speedRPM*ticksPerRotation/60);

        telemetry.addLine("Shooter PIDF Coefficients");
        telemetry.addData("kP\t", kP);
        telemetry.addData("kI\t", kI);
        telemetry.addData("kD\t", kD);
        telemetry.addData("kF\t", kF);
    }

    public void changePIDFCoefficients(double P, double I, double D, double F){
        kP = P; kI = I; kD = D; kF = F;
        shooter.setVelocityPIDFCoefficients(kP, kI, kD, kF);
    }

}
