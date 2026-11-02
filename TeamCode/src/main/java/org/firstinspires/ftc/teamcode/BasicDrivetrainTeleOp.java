package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.RobotIn30Hours.Mecanum;

@TeleOp(name="Drivetrainwith TeleOp")
public class BasicDrivetrainTeleOp extends LinearOpMode {

    CombinedShooter shooter;
    Intake intake;
    Transfer transfer;
    Mecanum drivetrain;


    @Override
    public void runOpMode(){
        //Obviously, motor/servo names are placeholders
//        shooter = new CombinedShooter("Emperor niranjan I", "LiAndy",
//                "Kou", "Max", "Kirchstein",
//                telemetry, hardwareMap, gamepad2);
//        intake = new Intake("lowly peasant prithvi", telemetry, hardwareMap, gamepad1);
//        transfer = new Transfer("Prince Raj", telemetry, hardwareMap, gamepad2);
        shooter = new CombinedShooter("Emperor niranjan I", "LiAndy",
                "Kou", "Max", "Kirchstein",
                telemetry, hardwareMap, gamepad2);
        intake = new Intake("lowly peasant prithvi", telemetry, hardwareMap, gamepad1);
        transfer = new Transfer("Prince Raj", telemetry, hardwareMap, gamepad2);
        drivetrain = new Mecanum(hardwareMap, gamepad1, telemetry);

        waitForStart();

        while(opModeIsActive()){
            intake.run();
            shooter.run();
            transfer.run();
            drivetrain.run();
        }
    }
}
