package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    static final String[] DRIVE_NAMES = {"leftfront", "rightfront", "leftback", "rightback"};
    // Verify each wheel's polarity with Mecanum Wheel Test.
    // Change a wheel to REVERSE if positive test power rolls that wheel backward.
    static final DcMotor.Direction[] DRIVE_DIRECTIONS = {
            DcMotor.Direction.FORWARD, DcMotor.Direction.FORWARD,
            DcMotor.Direction.FORWARD, DcMotor.Direction.REVERSE
    };
    // goBILDA 435 RPM / 13.7:1 gearbox; the external 1:1 bevel adds no reduction.
    private static final double TICKS_PER_REV = 384.5;
    private static final double MOTOR_RPM = 435.0;
    // Leave headroom below no-load speed for regulation under load; lower if needed.
    private static final double MAX_TICKS_PER_SECOND = TICKS_PER_REV * MOTOR_RPM / 60.0 * 0.75;

    @Override
    public void runOpMode() {
        DcMotorEx[] wheels = new DcMotorEx[DRIVE_NAMES.length];
        for (int i = 0; i < wheels.length; i++) {
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, DRIVE_NAMES[i]);
            wheels[i] = motor;
            motor.setPower(0);
            motor.setDirection(DRIVE_DIRECTIONS[i]);
            // Clone rather than mutate the shared SDK motor definition. Some hub
            // configurations use the generic goBILDA profile for a different gearbox.
            MotorConfigurationType motorType = motor.getMotorType().clone();
            motorType.setTicksPerRev(TICKS_PER_REV);
            motorType.setMaxRPM(MOTOR_RPM);
            motor.setMotorType(motorType);
            setEncoderMode(motor);
        }
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

        double[] targets = new double[wheels.length];
        try {
            while (opModeIsActive()) {
                // FTC gamepads report forward stick movement as negative Y.
                double drive = -deadband(gamepad1.left_stick_y);
                // Slightly compensate for the reduced lateral traction of mecanum wheels.
                double strafe = deadband(gamepad1.left_stick_x) * 1.1;
                double turn = deadband(gamepad1.right_stick_x);

                targets[0] = drive + strafe + turn;
                targets[1] = drive - strafe - turn;
                targets[2] = drive - strafe + turn;
                targets[3] = drive + strafe - turn;

                // One common scale preserves the wheel-speed ratios, including diagonals.
                double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1.0);
                double speedLimit = MAX_TICKS_PER_SECOND * (gamepad1.left_bumper ? 0.25 : 1.0);
                for (int i = 0; i < wheels.length; i++) {
                    targets[i] = targets[i] / denominator * speedLimit;
                    // The hub closes each wheel's velocity loop using its own encoder.
                    wheels[i].setVelocity(targets[i]);
                    telemetry.addData(DRIVE_NAMES[i], "target %.0f | actual %.0f ticks/s | pos %d",
                            targets[i], wheels[i].getVelocity(), wheels[i].getCurrentPosition());
                }

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
        } finally {
            for (DcMotorEx wheel : wheels) {
                wheel.setPower(0);
            }
            intake.setPower(0);
        }
    }

    private double deadband(double value) {
        return Math.abs(value) <= 0.05 ? 0.0
                : Math.copySign((Math.abs(value) - 0.05) / 0.95, value);
    }

    private void setEncoderMode(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
