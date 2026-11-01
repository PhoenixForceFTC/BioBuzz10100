package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.CombinedShooter;
import org.firstinspires.ftc.teamcode.Transfer;
import org.firstinspires.ftc.teamcode.Intake;

@TeleOp(name="Drivetrainless TeleOp")
public class BasicNoDrivetrainTeleOp extends LinearOpMode {

    CombinedShooter shooter;
    Intake intake;
    Transfer transfer;

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

        waitForStart();

        while(opModeIsActive()){
            intake.run();
            shooter.run();
            transfer.run();
        }
    }
}
