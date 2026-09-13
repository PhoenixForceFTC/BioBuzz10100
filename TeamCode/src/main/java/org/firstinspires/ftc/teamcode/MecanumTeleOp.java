package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

/**
 * Robot-configuration names:
 * <ul>
 *     <li>Control Hub 0: leftfront</li>
 *     <li>Control Hub 1: rightfront</li>
 *     <li>Control Hub 2: leftback</li>
 *     <li>Control Hub 3: rightback</li>
 *     <li>Expansion Hub 0: intake</li>
 * </ul>
 */
@TeleOp(name = "Mecanum TeleOp", group = "Drive")
public class MecanumTeleOp extends LinearOpMode {
    // Order is FL, FR, BL, BR throughout the mixing and the wheel test.

    DcMotorEx leftFront;
    DcMotorEx leftBack;
    DcMotorEx rightFront;
    DcMotorEx rightBack;
    double LFpower = 0, LBpower = 0, RFpower = 0, RBpower = 0;

    // goBILDA 435 RPM / 13.7:1 gearbox; the external 1:1 bevel adds no reduction.
    // Leave headroom below no-load speed for regulation under load; lower if needed.

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftfront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftback");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightfront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightback");

        leftFront.setPower(LFpower);
        leftBack.setPower(LBpower);
        rightFront.setPower(RFpower);
        rightBack.setPower(RBpower);

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setPower(0);
        setEncoderMode(intake);

        telemetry.addLine("Verify directions/encoder pairing with Mecanum Wheel Test first");
        telemetry.addLine("Left stick: drive/strafe | Right stick X: rotate");
        telemetry.addLine("Left trigger: intake reverse | Right trigger: forward");
        telemetry.addLine("Hold left bumper for 25% driving speed");
        telemetry.setMsTransmissionInterval(100);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // FTC gamepads report forward stick movement as negative Y.
            double drive = Math.abs(gamepad1.left_stick_y) <= 0.05 ? 0 : -gamepad1.left_stick_y;
            double strafe = Math.abs(gamepad1.left_stick_y) <= 0.05 ? 0 : gamepad1.left_stick_y;
            double turn = Math.abs(gamepad1.right_stick_x) <= 0.05 ? 0 : gamepad1.right_stick_x;
            double total = Math.abs(drive) + Math.abs(strafe) + Math.abs(turn);

            try{
                LFpower = (drive + strafe + turn)/total;
                RFpower = (drive - strafe - turn)/total;
                LBpower = (drive - strafe + turn)/total;
                RBpower = (drive + strafe - turn)/total;
            } catch (Exception e) {
                LFpower = 0;
                RFpower = 0;
                LBpower = 0;
                RBpower = 0;
            }

            leftFront.setPower(LFpower);
            leftBack.setPower(LBpower);
            rightFront.setPower(RFpower);
            rightBack.setPower(RBpower);

            // Triggers behave like full-speed directional buttons. If both are held,
            // their directions cancel so the motor stops.
            double intakePower = 0.0;
            if (gamepad1.right_trigger > 0.05) {
                intakePower += 1.0;
            }
            if (gamepad1.left_trigger > 0.05) {
                intakePower -= 1.0;
            }
            intake.setPower(intakePower);
            telemetry.addData("Intake encoder", intake.getCurrentPosition());
            telemetry.update();
            idle();
        }
    }

    private void setEncoderMode(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
