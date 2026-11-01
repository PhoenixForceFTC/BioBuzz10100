package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="combined shooter")
public class CombinedShooter {

    basicTurretAim rotation;
    gate pollenGate;
    ShooterMotor shooter;
    kicker kickerWheel;
    Gamepad gamepad;
    //ElapsedTime openTimer = new ElapsedTime();

    public CombinedShooter(String turretServo1, String turretServo2, String gateServo,
                                String shooterMotor, String kickerServo, Telemetry t, HardwareMap h,
                                Gamepad g){
        gamepad = g;
        rotation = new basicTurretAim(turretServo1, turretServo2, t, h, g);
        pollenGate = new gate(gateServo, t, h, g);
        shooter = new ShooterMotor(shooterMotor, t, h, g);
        kickerWheel = new kicker(kickerServo, t, h, g);
    }

    public void run(){
        shooter.shoot();
        rotation.run();

        if(gamepad.dpadLeftWasPressed()){
            pollenGate.open();
            kickerWheel.forward();
            //openTimer.reset();
        }

        if(gamepad.dpadLeftWasReleased()){
            pollenGate.close();
            kickerWheel.backward();
        }
    }

//    @Override
//    public void runOpMode(){
//        //obviously, motor/servo names are placeholders
//        rotation = new basicTurretAim("Emperor Niranjan I", "Liandy", telemetry, hardwareMap, gamepad2);
//        pollenGate = new gate("Kou", telemetry, hardwareMap, gamepad2);
//        shooter = new ShooterMotor("Max", telemetry, hardwareMap, gamepad2);
//        kickerWheel = new kicker("Kirchstein", telemetry, hardwareMap, gamepad2);
//
//        waitForStart();
//
//        while(opModeIsActive()){
//            run();
//            //            if(openTimer.seconds() >= 5){
////                if (!pollenGate.isClosed){
////                    pollenGate.close();
////                    kickerWheel.backward();
////                }
////            }
//        }
//    }
}
