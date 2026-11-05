package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;

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
@TeleOp(name = "Test opmode", group = "Drive")
public class testopmodethingy extends LinearOpMode {
    // Order is FL, FR, BL, BR throughout the mixing and the wheel test.

    boolean fast = false;

    DcMotorEx leftFront;
    DcMotorEx leftBack;
    DcMotorEx rightFront;
    DcMotorEx rightBack;
    double LFpower = 0, LBpower = 0, RFpower = 0, RBpower = 0;

    Shooter shooter;
    Intake intake;
    Limelight limelight;
    RotateToLimelight aim;

    // picked during init with dpad left/right; decides which scoring cluster auto-aim faces.
    boolean redAlliance = true;

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

        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setPower(0);
        setEncoderMode(intakeMotor);

        shooter = new Shooter("shooter1", "shooter2", telemetry, hardwareMap, gamepad1);
        intake = new Intake("intake", telemetry, hardwareMap, gamepad1);
        limelight = new Limelight(hardwareMap, telemetry);
        aim = new RotateToLimelight();

        telemetry.setMsTransmissionInterval(100);

        while (opModeInInit()) {
            if (gamepad1.dpad_left) redAlliance = false;
            if (gamepad1.dpad_right) redAlliance = true;
            // update() clears telemetry, so these lines get added again on every pass.
            telemetry.addLine("Verify directions/encoder pairing with Mecanum Wheel Test first");
            telemetry.addLine("Left stick: drive/strafe | Right stick X: rotate");
            telemetry.addLine("Left bumper: toggle 60%/full speed, also sets intake direction");
            telemetry.addLine("Right bumper: other intake direction | Y: intake on/off");
            telemetry.addLine("A/B: shooter power up/down | X: shooter on/off");
            telemetry.addLine("Hold right trigger: auto-aim at scoring AprilTags");
            telemetry.addData("Alliance (dpad L=blue, R=red)", redAlliance ? "RED" : "BLUE");
            telemetry.update();
        }

        while (opModeIsActive()) {
            // FTC gamepads report forward stick movement as negative Y.
            double drive = Math.abs(gamepad1.left_stick_y) <= 0.05 ? 0 : -gamepad1.left_stick_y;
            double strafe = Math.abs(gamepad1.left_stick_x) <= 0.05 ? 0 : gamepad1.left_stick_x;
            double turn = Math.abs(gamepad1.right_stick_x) <= 0.05 ? 0 : gamepad1.right_stick_x;

            // auto-aim: while held, the PID replaces the driver's turn, but drive/strafe still work.
            if (gamepad1.right_trigger > 0.5) {
                ArrayList<LimelightData> tags = limelight.scan();
                for (LimelightData tag : tags) {
                    telemetry.addData("Tag " + tag.getId(), "%s  tx=%.1f°  dist=%.2fm",
                            BioBuzzTags.clusterName(tag.getId()), tag.getAngleX(), tag.getDistance());
                }
                LimelightData target = BioBuzzTags.selectAimTarget(tags, redAlliance);
                turn = aim.getTurnPower(target);
                telemetry.addData("Aim", target == null ? "no scoring tag" : (aim.isAtTarget() ? "LOCKED" : "turning"));
            } else {
                aim.reset();
            }
            // only scale down when the inputs add past 1, so small inputs (like PID output) stay small.
            double total = Math.max(1.0, Math.abs(drive) + Math.abs(strafe) + Math.abs(turn));

            if(gamepad1.leftBumperWasPressed()){
                fast = !fast;
            }

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

            if(fast){
                leftFront.setPower(LFpower);
                leftBack.setPower(LBpower);
                rightFront.setPower(RFpower);
                rightBack.setPower(RBpower);

            } else if (!fast){
                leftFront.setPower(0.6*LFpower);
                leftBack.setPower(0.6*LBpower);
                rightFront.setPower(0.6*RFpower);
                rightBack.setPower(0.6*RBpower);
            }

            shooter.run();

            // Triggers behave like full-speed directional buttons. If both are held,
            // their directions cancel so the motor stops.
            intake.run();
//            double intakePower = 0.0;
//            if (gamepad1.right_trigger > 0.05) {
//                intakePower += 1.0;
//            }
//            if (gamepad1.left_trigger > 0.05) {
//                intakePower -= 1.0;
//            }
//            intakeMotor.setPower(intakePower);
//            telemetry.addData("Intake encoder", intakeMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }

        limelight.stop();
    }

    private void setEncoderMode(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
