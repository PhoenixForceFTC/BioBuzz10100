package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.MecanumPedro;
import org.firstinspires.ftc.teamcode.Relocalization;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

@TeleOp(name="Drivetrainwith TeleOp")
public class BasicDrivetrainTeleOp extends LinearOpMode {

    double currentDist = 0;
    double prevDist = 0;
    CombinedShooter shooter;
    Intake intake;
    MecanumPedro drivetrain;
    LimelightRelocalizer relocalization;
    final PoseFactory p = PoseFactory.degrees();
    Pose initialPose = p.of(72,72,0);
    Limelight3A ll;


    @Override
    public void runOpMode(){
        shooter = new CombinedShooter("turret_servo_1", "turret_servo_2",
                "gate", "shooter", "kicker",
                "transfer", telemetry, hardwareMap, gamepad2);
        intake = new Intake("intake", telemetry, hardwareMap, gamepad1);
        drivetrain = new MecanumPedro(telemetry, hardwareMap, gamepad1, initialPose);
        ll = hardwareMap.get(Limelight3A.class, "We are the champagnes");
        ll.setPollRateHz(100);
        ll.pipelineSwitch(SimpleLimelightRelocalizer.PIPELINE);
        ll.start();
        relocalization = new LimelightRelocalizer(ll, new HashSet<>(Arrays.asList(32, 36, 40, 44)));
        shooter.rotation.moveToPosition(0.5);
        shooter.rotation.turretServo1.setPosition(0.5);shooter.rotation.turretServo2.setPosition(0.5);

        sleep(500); // for turret mvmt

        while(opModeInInit()){
            relocalization.update(drivetrain.follower, (-0.25) * 2 * Math.PI);
        }

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
            relocalization.update(drivetrain.follower, turretoffset);

            if(gamepad2.dpad_right || prevDist!=currentDist){
                shooter.hood.moveToPosition(Math.max(-0.00309*currentDist + 0.652, 0.5));
                shooter.shooter.speedRPM = (int) (6.22*currentDist + 2516);
            }

            telemetry.addData("currentDist", currentDist);
            telemetry.addData("Reloc", relocalization.status()); telemetry.update();

            telemetry.update();
        }
    }
}
