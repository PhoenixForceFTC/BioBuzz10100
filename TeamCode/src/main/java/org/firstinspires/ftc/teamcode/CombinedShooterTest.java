package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="combined shooter")
public class CombinedShooterTest extends LinearOpMode{
    CombinedShooter shooter;

    double P = 10;
    double I = 2;
    double D = 0;
    double F = 12;
    enum PARAM{P, I, D, F}

    @Override
    public void runOpMode(){
        //obviously, motor/servo names are placeholders
        shooter = new CombinedShooter("turret_servo_1", "turret_servo_2",
                "gate", "shooter", "kicker", "transfer",
                telemetry, hardwareMap, gamepad2);
        ShooterMotorTest.PARAM currentParam = ShooterMotorTest.PARAM.P;

        waitForStart();

        while(opModeIsActive()){
            shooter.run();
            //telemetry.update();

            if(gamepad1.xWasPressed()) {
                if (currentParam == ShooterMotorTest.PARAM.P) {P += 0.5;}
                else if (currentParam == ShooterMotorTest.PARAM.I) {I += 0.5;}
                else if (currentParam == ShooterMotorTest.PARAM.D) {D += 0.5;}
                else if (currentParam == ShooterMotorTest.PARAM.F) {F += 0.5;}
            }
            if(gamepad1.yWasPressed()){
                if(currentParam == ShooterMotorTest.PARAM.P){P = Math.max(0, P-0.5);}
                else if(currentParam == ShooterMotorTest.PARAM.I){I = Math.max(0, I-0.5);}
                else if(currentParam == ShooterMotorTest.PARAM.D){D = Math.max(0, D-0.5);}
                else if(currentParam == ShooterMotorTest.PARAM.F){F = Math.max(0, F-0.5);}
            }
            if(gamepad1.bWasPressed()){
                if(currentParam == ShooterMotorTest.PARAM.P){currentParam = ShooterMotorTest.PARAM.I;}
                else if(currentParam == ShooterMotorTest.PARAM.I){currentParam = ShooterMotorTest.PARAM.D;}
                else if(currentParam == ShooterMotorTest.PARAM.D){currentParam = ShooterMotorTest.PARAM.F;}
                else if(currentParam == ShooterMotorTest.PARAM.F){currentParam = ShooterMotorTest.PARAM.P;}
            }
            shooter.shooter.changePIDFCoefficients(P, I, D, F);
            telemetry.addData("Current PIDF Param: ", currentParam);
            telemetry.update();
            //            if(openTimer.seconds() >= 5){
//                if (!pollenGate.isClosed){
//                    pollenGate.close();
//                    kickerWheel.backward();
//                }
//            }
        }
    }
}
