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
        shooter = new CombinedShooter("turret_servo_1", "turret_servo_2",
                "gate_servo", "turret_motor", "transfer_motor_2",
                telemetry, hardwareMap, gamepad2);
        intake = new Intake("intake_motor", telemetry, hardwareMap, gamepad1);
        transfer = new Transfer("transfer_motor_1", telemetry, hardwareMap, gamepad2);

        waitForStart();

        while(opModeIsActive()){
            telemetry.addLine("gamepad 2 dpadup/dpaddown: increase/decrease shooter speed");
            telemetry.addLine("gamepad 2 hold dpadleft: shoot");
            telemetry.addLine("gamepad 2 left stick: turn turret");
            telemetry.addLine("gamepad 2 left stick: turn turret");
            telemetry.addLine("gamepad 2 left stick: turn turret");
            telemetry.addLine("gamepad 2 a/b: change turn speed");
            telemetry.addLine("gamepad 1 left/right bumper/y: intake forward/reverse/stop");
            telemetry.addLine("gamepad 2 left/right bumper/dpadright: transfer forward/reverse/stop");
            telemetry.addLine("gamepad 1 a: toggle drive speed");
            telemetry.addLine();
            intake.run();
            shooter.run();
            transfer.run();
            telemetry.update();
        }
    }
}
