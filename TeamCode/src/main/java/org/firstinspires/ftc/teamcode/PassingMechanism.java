package org.firstinspires.ftc.teamcode;
//Imports

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//Step one
public class PassingMechanism {
    public DcMotorEx passmotor; //To get motor's extra features
    private double power = 0;
    Telemetry _telemetry;
    Gamepad gamepad;
    HardwareMap hardwareMap;
    boolean isZero = false;

    public PassingMechanism(String passerName, Telemetry telemetry, HardwareMap h, Gamepad GameGamepad){
        this.hardwareMap = h;
        this.passmotor = hardwareMap.get(DcMotorEx.class, passerName);
        this.passmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.passmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.passmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this._telemetry = telemetry;
        this.gamepad = GameGamepad;
    }

    public void run() {
        //Raises power by 0.05 each time 'a' is pressed
        if (gamepad.dpadLeftWasPressed()) {
            power = Math.min(1.0, power + 0.05);
            //Lowers power by 0.05 each time b is pressed
        } else if (gamepad.dpadRightWasPressed()) { // don't NEED else if
            power = Math.max(0.0, power - 0.05);
        }
        // might want to add an instant start. recommend: make a boolean named activated, separate to power
        //Instant stop
        if (gamepad.yWasPressed()) {
            isZero = !isZero;
        }

        if (isZero) {
            passmotor.setPower(0);
        } else{
            passmotor.setPower(power);
        }
        double rpm = passmotor.getVelocity() * 60 / 28; //Reads speed

        //Display speed
        _telemetry.addData("Passer power", "%.2f", power);
        _telemetry.addData("Passer RPM", "%.0f", rpm);
    }
}