package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.MecanumPedro;

@TeleOp(name="Drivetrainwith TeleOp")
public class BasicDrivetrainTeleOp extends LinearOpMode {

    CombinedShooter shooter;
    Intake intake;
    MecanumPedro drivetrain;
    final PoseFactory p = PoseFactory.degrees();
    Pose initialPose = p.of(72,72,0);


    @Override
    public void runOpMode(){
        //Obviously, motor/servo names are placeholders
//        shooter = new CombinedShooter("Emperor niranjan I", "LiAndy",
//                "Kou", "Max", "Kirchstein",
//                telemetry, hardwareMap, gamepad2);
//        intake = new Intake("lowly peasant prithvi", telemetry, hardwareMap, gamepad1);
//        transfer = new Transfer("Prince Raj", telemetry, hardwareMap, gamepad2);
        shooter = new CombinedShooter("turret_servo_1", "turret_servo_2",
                "gate", "shooter", "kicker",
                "transfer", telemetry, hardwareMap, gamepad2);
        intake = new Intake("intake", telemetry, hardwareMap, gamepad1);
        drivetrain = new MecanumPedro(telemetry, hardwareMap, gamepad1, initialPose);

        waitForStart();

        while(opModeIsActive()){
            intake.run();

            drivetrain.run();

            if(gamepad1.dpad_left){
                Pose closeGoal = p.of(60, 60, 0);
                Pose robot = drivetrain.getPose();
                double dx = closeGoal.x() - robot.x();
                double dy = closeGoal.y() - robot.y();
                double turretAngle = Math.atan2(dy, dx) - robot.heading();   // radians
                turretAngle = Math.atan2(Math.sin(turretAngle), Math.cos(turretAngle)); // wrap to -π..π
                double servoPos = 0.25 - turretAngle / (2*Math.PI); // servos move clockwise as position increases
                while (servoPos < 0)  servoPos += 1;
                while (servoPos >= 1) servoPos -= 1;
                shooter.rotation.moveToPosition(servoPos);
            }

            if(gamepad1.dpad_right){
                Pose farGoal = p.of(60, 84, 0);
                Pose robot = drivetrain.getPose();
                double dx = farGoal.x() - robot.x();
                double dy = farGoal.y() - robot.y();
                double turretAngle = Math.atan2(dy, dx) - robot.heading();   // radians
                turretAngle = Math.atan2(Math.sin(turretAngle), Math.cos(turretAngle)); // wrap to -π..π
                double servoPos = 0.25 - turretAngle / (2*Math.PI); // servos move clockwise as position increases
                while (servoPos < 0)  servoPos += 1;
                while (servoPos >= 1) servoPos -= 1;
                shooter.rotation.moveToPosition(servoPos);
            }
            shooter.run();

            telemetry.update();
        }
    }
}
