package org.firstinspires.ftc.teamcode;
//Imports

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//Step one
@TeleOp(name = "Shooter")
public class Shooter {
    public DcMotorEx shooter; //To get motor's extra features
    private double power = 0;
    Telemetry _telemetry;
    Gamepad gamepad;
    HardwareMap hardwareMap;
    boolean isZero = false;

    public Shooter(String shooterName, Telemetry telemetry, HardwareMap h, Gamepad GameGamepad){
        this.hardwareMap = h;
        this.shooter = hardwareMap.get(DcMotorEx.class, shooterName);
        this.shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        this.shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this._telemetry = telemetry;
        this.gamepad = GameGamepad;
    }

    public void run() {
        //Raises power by 0.05 each time 'a' is pressed
        if (gamepad.aWasPressed()) {
            power = Math.min(1.0, power + 0.05);
            //Lowers power by 0.05 each time b is pressed
        } else if (gamepad.bWasPressed()) { // don't NEED else if
            power = Math.max(0.0, power - 0.05);
        }
        // might want to add an instant start. recommend: make a boolean named activated, separate to power
        //Instant stop
        if (gamepad.xWasPressed()) {
            isZero = !isZero;
        }

        if (isZero) {
            shooter.setPower(0);
        } else{
            shooter.setPower(power);
        }
        double rpm = shooter.getVelocity() * 60 / 28; //Reads speed

        //Display speed
        _telemetry.addData("Shooter power", "%.2f", power);
        _telemetry.addData("Shooter RPM", "%.0f", rpm);
    }
}

