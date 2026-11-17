package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.MecanumPedro;
import org.firstinspires.ftc.teamcode.Relocalization;

@TeleOp(name="Drivetrainwith TeleOp")
public class BasicDrivetrainTeleOp extends LinearOpMode {

    double currentDist = 0;
    double prevDist = 0;
    CombinedShooter shooter;
    Intake intake;
    MecanumPedro drivetrain;
    Relocalization relocalization;
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
        relocalization = new Relocalization("We are the champagnes", telemetry, hardwareMap);

        relocalization.relocalize(true, drivetrain.getPose(), -0.25, drivetrain.follower);

        waitForStart();

        while(opModeIsActive()){
            intake.run();

            drivetrain.run();

            prevDist = currentDist;

            if(gamepad1.dpad_left){
                Pose closeGoal = p.of(60, 52, 0);
                Pose robot = drivetrain.getPose();
                double dx = closeGoal.x() - robot.x();
                double dy = closeGoal.y() - robot.y();
                double turretAngle = Math.atan2(dy, dx) - robot.heading();   // radians
                turretAngle = Math.atan2(Math.sin(turretAngle), Math.cos(turretAngle)); // wrap to -π..π
                double servoPos = 0.25 - turretAngle / (2*Math.PI); // servos move clockwise as position increases
                while (servoPos < 0)  servoPos += 1;
                while (servoPos >= 1) servoPos -= 1;
                shooter.rotation.moveToPosition(servoPos);
                currentDist = Math.hypot(dy, dx);
                telemetry.addData("Distance from (60, 52): ", currentDist);
            }

            if(gamepad1.dpad_right){
                Pose farGoal = p.of(60, 92, 0);
                Pose robot = drivetrain.getPose();
                double dx = farGoal.x() - robot.x();
                double dy = farGoal.y() - robot.y();
                double turretAngle = Math.atan2(dy, dx) - robot.heading();   // radians
                turretAngle = Math.atan2(Math.sin(turretAngle), Math.cos(turretAngle)); // wrap to -π..π
                double servoPos = 0.25 - turretAngle / (2*Math.PI); // servos move clockwise as position increases
                while (servoPos < 0)  servoPos += 1;
                while (servoPos >= 1) servoPos -= 1;
                shooter.rotation.moveToPosition(servoPos);
                currentDist = Math.hypot(dy, dx);
                telemetry.addData("Distance from (60, 92): ", currentDist);
            }

            if(gamepad1.dpad_up){
                Pose closeGoal = p.of(84, 52, 0);
                Pose robot = drivetrain.getPose();
                double dx = closeGoal.x() - robot.x();
                double dy = closeGoal.y() - robot.y();
                double turretAngle = Math.atan2(dy, dx) - robot.heading();   // radians
                turretAngle = Math.atan2(Math.sin(turretAngle), Math.cos(turretAngle)); // wrap to -π..π
                double servoPos = 0.25 - turretAngle / (2*Math.PI); // servos move clockwise as position increases
                while (servoPos < 0)  servoPos += 1;
                while (servoPos >= 1) servoPos -= 1;
                shooter.rotation.moveToPosition(servoPos);
                currentDist = Math.hypot(dy, dx);
                telemetry.addData("Distance from (84, 52): ", currentDist);
            }

            if(gamepad1.dpad_down){
                Pose farGoal = p.of(84, 92, 0);
                Pose robot = drivetrain.getPose();
                double dx = farGoal.x() - robot.x();
                double dy = farGoal.y() - robot.y();
                double turretAngle = Math.atan2(dy, dx) - robot.heading();   // radians
                turretAngle = Math.atan2(Math.sin(turretAngle), Math.cos(turretAngle)); // wrap to -π..π
                double servoPos = 0.25 - turretAngle / (2*Math.PI); // servos move clockwise as position increases
                while (servoPos < 0)  servoPos += 1;
                while (servoPos >= 1) servoPos -= 1;
                shooter.rotation.moveToPosition(servoPos);
                currentDist = Math.hypot(dy, dx);
                telemetry.addData("Distance from (84, 92): ", currentDist);
            }
            shooter.run();

            double turretoffset = (0.25 - shooter.rotation.position) * 2 * Math.PI;
            relocalization.relocalize(false, drivetrain.getPose(), turretoffset, drivetrain.follower);

            if(gamepad2.dpad_right || prevDist!=currentDist){
                shooter.hood.moveToPosition(Math.max(-0.00309*currentDist + 0.652, 0.5));
                shooter.shooter.speedRPM = (int) (6.22*currentDist + 2516);
            }

            telemetry.addData("currentDist", currentDist);
            telemetry.addData("dist to tag 36 (r):", relocalization.distanceToTag(36));
            telemetry.addData("dist to tag 32 (r):", relocalization.distanceToTag(32));
            telemetry.addData("dist to tag 40 (b):", relocalization.distanceToTag(40));
            telemetry.addData("dist to tag 44 (b):", relocalization.distanceToTag(44));

            telemetry.update();
        }
    }
}
