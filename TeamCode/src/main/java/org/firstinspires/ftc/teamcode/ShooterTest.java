package org.firstinspires.ftc.teamcode;
//Imports
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
//Step one
@TeleOp (name= "Shooter")
public class ShooterTest extends LinearOpMode {
    public DcMotor shooter;
    private double power = 0;

    @Override
    public void runOpMode() {
        //When driver is in init
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        //Step 3
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //When driver presses start
        waitForStart();
        //double targetRpm = 3000;
        while (opModeIsActive()) {
            if (gamepad1.a) {
                if (power <= 0.99) {
                    power += 0.01;
                }
            } else if (gamepad1.b) {
                if (power >= 0.01) {
                    power -= 0.01;
                }
            }
            shooter.setPower(power);

            telemetry.addData("Shooter power: ", power);
            idle();
        }
    }
}

