package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CombinedShooter {

    basicTurretAim rotation;
    Transfer transfer;
    gate pollenGate;
    ShooterMotor shooter;
    kicker kickerWheel;
    Gamepad gamepad;
    hoodAngle hood;
    //ElapsedTime openTimer = new ElapsedTime();



    public CombinedShooter(String turretServo1, String turretServo2, String gateServo,
                                String shooterMotor, String kickerMotor, String transferMotor,
                                Telemetry t, HardwareMap h, Gamepad g){
        gamepad = g;
        rotation = new basicTurretAim(turretServo1, turretServo2, t, h, g);
        pollenGate = new gate(gateServo, t, h, g);
        shooter = new ShooterMotor(shooterMotor, t, h, g);
        kickerWheel = new kicker(kickerMotor, t, h, g);
        transfer = new Transfer(transferMotor, t, h, g);
        hood = new hoodAngle("hood", t, h, g);
    }

    public void run(){
        shooter.shoot();
        rotation.run();
        transfer.run();
        //pollenGate.run();
        kickerWheel.run();
        hood.run();

        if(gamepad.dpadLeftWasPressed()){
            pollenGate.open();
            kickerWheel.forward();
            transfer.forward();
        } else if (gamepad.dpadLeftWasReleased()) {
            pollenGate.close();
            kickerWheel.stop();
        }

        if(gamepad.yWasPressed()){
            transfer.stop();
            kickerWheel.stop();
        }

        if(gamepad.xWasPressed()){
            transfer.forward();
            kickerWheel.forward();
        }

//        if(gamepad.dpadLeftWasPressed()){
//            pollenGate.open();
//            kickerWheel.forward();
//            //openTimer.reset();
//        }
//
//        if(gamepad.dpadLeftWasReleased()){
//            pollenGate.close();
//            kickerWheel.backward();
//        }

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
