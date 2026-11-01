package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class limelightTurretAimRed {

    private Servo turretServo1;
    private Servo turretServo2;
    private Telemetry telemetryInternal;
    private Gamepad gamepad;
    Limelight limelight;
    double position = 0.5;
    double speed = 0.005;
    //private double id39dist;
    private double id31dist;
    private double id35dist;
    //private double id43dist;


    public limelightTurretAimRed(String servo1, String servo2, String limelightName, Telemetry t,
                                 HardwareMap h, Gamepad g) {
        turretServo1 = h.get(Servo.class, servo1);
        turretServo2 = h.get(Servo.class, servo2);
        limelight = new Limelight(h, t, limelightName);
        telemetryInternal = t;
        gamepad = g;
    }

    public void run() {
        ArrayList<LimelightData> results = limelight.scan();
        if(gamepad.left_stick_x > 0.08){
            position = Math.min(1, Math.max(0, position + speed));
        } else if (gamepad.left_stick_x < -0.08){
            position = Math.min(1, Math.max(0, position - speed));
        }

        if(gamepad.aWasPressed()){
            speed = Math.min(0.015, speed+0.001);
        } else if(gamepad.bWasPressed()){
            speed = Math.max(0.001, speed-0.001);
        }

        if(gamepad.xWasPressed()){
            position = 0.75;
        }

        if(gamepad.yWasPressed()){
            // default data to avoid bugs
            LimelightData id31data = new LimelightData(31, 0, 0, 100, 100, 100);
            LimelightData id35data = new LimelightData(35, 0, 0, 100, 100, 100);
            // set dist to high number so it does not attempt to shoot there if it isn't read
            id31dist = 20000;
            id35dist = 20000;
            if(!results.isEmpty()) {
                for (LimelightData datum : results) {
                    if (datum.getId() == 31) {
                        id31data = datum;
                        id31dist = datum.getDistance();
                    } else if (datum.getId() == 35) {
                        id35data = datum;
                        id35dist = datum.getDistance();
                    }
                }
                if(id31dist <= id35dist){
                    double angle = id31data.getAngleX();
                    position = Math.max(0, Math.min(1, position + angle/360)); //clamp
                }
                if(id35dist < id31dist){
                    double angle = id35data.getAngleX();
                    position = Math.max(0, Math.min(1, position + angle/360)); //clamp
                }
            }

        }

        turretServo1.setPosition(position);
        turretServo2.setPosition(position);

        telemetryInternal.addData("turret target position:\t", position);
        telemetryInternal.addData("turret speed:\t", speed);
    }
}
