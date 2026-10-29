package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.ShooterMotor;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

// url for dashboard: http://192.168.43.1:8080/dash
@TeleOp(name="Basic Turret Aim Test")
public class ShooterMotorTest extends LinearOpMode{
    ShooterMotor turret;
    private final TelemetryPacket packet = new TelemetryPacket();
    int speed;

    double P = 10;
    double I = 2;
    double D = 0;
    double F = 12;
    enum PARAM{P, I, D, F}

    @Override
    public void runOpMode() {
        turret = new ShooterMotor("shooter", telemetry, hardwareMap, gamepad2);
        PARAM currentParam = PARAM.P;

        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();

        while(opModeIsActive()){
            packet.put("motorSpeed", turret.shooter.getVelocity());

            dashboard.sendTelemetryPacket(packet);

            turret.shoot();
            if(gamepad2.aWasPressed()) {
                if (currentParam == PARAM.P) {P += 0.5;}
                else if (currentParam == PARAM.I) {I += 0.5;}
                else if (currentParam == PARAM.D) {D += 0.5;}
                else if (currentParam == PARAM.F) {F += 0.5;}
            }
            if(gamepad2.bWasPressed()){
                if(currentParam == PARAM.P){P = Math.max(0, P-0.5);}
                else if(currentParam == PARAM.I){I = Math.max(0, I-0.5);}
                else if(currentParam == PARAM.D){D = Math.max(0, D-0.5);}
                else if(currentParam == PARAM.F){F = Math.max(0, F-0.5);}
            }
            if(gamepad2.xWasPressed()){
                if(currentParam == PARAM.P){currentParam = PARAM.I;}
                else if(currentParam == PARAM.I){currentParam = PARAM.D;}
                else if(currentParam == PARAM.D){currentParam = PARAM.F;}
                else if(currentParam == PARAM.F){currentParam = PARAM.P;}
            }
            turret.changePIDFCoefficients(P, I, D, F);
            telemetry.addData("Current PIDF Param: ", currentParam);
            telemetry.update();
        }
    }
}
