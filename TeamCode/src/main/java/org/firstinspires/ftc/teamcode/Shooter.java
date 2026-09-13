package org.firstinspires.ftc.teamcode;
//Imports

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//Step one
@TeleOp(name = "Shooter")
public class Shooter extends LinearOpMode {
    public DcMotorEx shooter; //To get motor's extra features
    private double power = 0;
    Telemetry _telemetry;

    public Shooter(String shooterName, Telemetry telemetry){
        this.shooter = hardwareMap.get(DcMotorEx.class, shooterName);
        this.shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        this.shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this._telemetry = telemetry;
    }

    public void run(){
        //Raises power by 0.05 each time a is pressed
        if (gamepad1.aWasPressed()) {
            power = Math.min(1.0, power + 0.05);
            //Lowers power by 0.05 each time b is pressed
        } else if (gamepad1.bWasPressed()) {
            power = Math.max(0.0, power - 0.05);
        }
        //Instant stop
        if (gamepad1.xWasPressed()) {
            power = 0;
        }
        shooter.setPower(power);
        double rpm = shooter.getVelocity() * 60 / 28; //Reads speed

        //Display speed
        _telemetry.addData("Shooter power", "%.2f", power);
        _telemetry.addData("Shooter RPM", "%.0f", rpm);
    }

    @Override
    public void runOpMode() {
        //When driver is in init
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        //Step 3
        shooter.setDirection(DcMotor.Direction.FORWARD);
        //FLOAT allows for wheel to coast down to be gentle on gears.
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //When driver presses start
        waitForStart();
        while (opModeIsActive()) {
            //Raises power by 0.05 each time a is pressed
            if (gamepad1.aWasPressed()) {
                power = Math.min(1.0, power + 0.05);
                //Lowers power by 0.05 each time b is pressed
            } else if (gamepad1.bWasPressed()) {
                power = Math.max(0.0, power - 0.05);
            }
            //Instant stop
            if (gamepad1.xWasPressed()) {
                power = 0;
            }
            shooter.setPower(power);
            double rpm = shooter.getVelocity() * 60 / 28; //Reads speed

            //Display speed
            _telemetry.addData("Shooter power", "%.2f", power);
            _telemetry.addData("Shooter RPM", "%.0f", rpm);
            _telemetry.update();
        }
    }
}

