package org.firstinspires.ftc.teamcode;
//Imports

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//Step one
public class Shooter {
    public DcMotorEx shooter1; //To get motor's extra features
    public DcMotorEx shooter2;
    private double power = 0;
    Telemetry _telemetry;
    Gamepad gamepad;
    HardwareMap hardwareMap;
    boolean isZero = false;

    public Shooter(String shooter1Name, String shooter2Name, Telemetry telemetry, HardwareMap h, Gamepad GameGamepad){
        this.hardwareMap = h;
        this.shooter1 = hardwareMap.get(DcMotorEx.class, shooter1Name);
        this.shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        this.shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.shooter2 = hardwareMap.get(DcMotorEx.class, shooter2Name);
        this.shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        this.shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
            shooter1.setPower(0);
            shooter2.setPower(0);
        } else{
            shooter1.setPower(power);
            shooter2.setPower(power);
        }
        double rpm = shooter1.getVelocity() * 60 / 28; //Reads speed

        //Display speed
        _telemetry.addData("Shooter power", "%.2f", power);
        _telemetry.addData("Shooter RPM", "%.0f", rpm);
    }
}

