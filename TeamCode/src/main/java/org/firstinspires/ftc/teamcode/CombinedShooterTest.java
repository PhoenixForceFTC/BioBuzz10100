package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="combined shooter")
public class CombinedShooterTest extends LinearOpMode{
    CombinedShooter shooter;

    @Override
    public void runOpMode(){
        //obviously, motor/servo names are placeholders
        shooter = new CombinedShooter("Emperor niranjan I", "Liandy",
                "Kou", "Max", "Kirchstein",
                telemetry, hardwareMap, gamepad2);

        waitForStart();

        while(opModeIsActive()){
            shooter.run();
            //            if(openTimer.seconds() >= 5){
//                if (!pollenGate.isClosed){
//                    pollenGate.close();
//                    kickerWheel.backward();
//                }
//            }
        }
    }
}
