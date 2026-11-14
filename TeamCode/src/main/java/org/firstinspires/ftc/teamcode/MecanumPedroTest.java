package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.MecanumPedro;

@TeleOp(name="Mecanum (pedro)")
public class MecanumPedroTest extends LinearOpMode {

//    CombinedShooter shooter;
//    Intake intake;
//    Transfer transfer;
    MecanumPedro drivetrain;

    private final PoseFactory p = PoseFactory.degrees();
    private Pose initPose = p.of(0, 0, 0);

    @Override
    public void runOpMode(){
        //Obviously, motor/servo names are placeholders
//        shooter = new CombinedShooter("Emperor niranjan I", "LiAndy",
//                "Kou", "Max", "Kirchstein",
//                telemetry, hardwareMap, gamepad2);
//        intake = new Intake("lowly peasant prithvi", telemetry, hardwareMap, gamepad1);
//        transfer = new Transfer("Prince Raj", telemetry, hardwareMap, gamepad2);
//        shooter = new CombinedShooter("Emperor niranjan I", "LiAndy",
//                "Kou", "Max", "Kirchstein",
//                telemetry, hardwareMap, gamepad2);
//        intake = new Intake("lowly peasant prithvi", telemetry, hardwareMap, gamepad1);
//        transfer = new Transfer("Prince Raj", telemetry, hardwareMap, gamepad2);
        drivetrain = new MecanumPedro(telemetry, hardwareMap, gamepad1, initPose);

        waitForStart();

        while(opModeIsActive()){
//            intake.run();
//            shooter.run();
//            transfer.run();
            drivetrain.run();
            telemetry.update();
        }
    }
}
