package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Simple BioBuzz Autonomous - just leaves starting line and shoots preloaded balls
 * No complex pathing, just basic motor control
 */
@Autonomous(name = "BioBuzz Simple Auto", group = "Autonomous")
public class RI30HAuto extends LinearOpMode {

    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotor shooterMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize drivetrain motors
        leftFront = hardwareMap.get(DcMotor.class, "leftfront");
        leftBack = hardwareMap.get(DcMotor.class, "leftback");
        rightFront = hardwareMap.get(DcMotor.class, "rightfront");
        rightBack = hardwareMap.get(DcMotor.class, "rightback");

        // Initialize shooter
        shooterMotor = hardwareMap.get(DcMotor.class, "shooter");

        // Set motor directions (adjust if your motors spin backwards)
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset and initialize encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Move forward away from starting line
            driveForward(0.6, 1500); // 60% power for 1.5 seconds

            // Shoot preloaded balls
            shootBalls();

            telemetry.addData("Status", "Complete");
            telemetry.update();
        }
    }

    /**
     * Drive forward at given power for given time
     */
    private void driveForward(double power, long timeMs) {
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);

        sleep(timeMs);
        stopDrivetrain();
    }

    /**
     * Spin up and run shooter to fire preloaded balls
     */
    private void shootBalls() {
        shooterMotor.setPower(1.0); // Full power

        // Let shooter spin up and fire balls
        sleep(2500); // Adjust based on how long your shooter takes

        shooterMotor.setPower(0); // Stop
    }

    /**
     * Stop all drivetrain motors
     */
    private void stopDrivetrain() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}