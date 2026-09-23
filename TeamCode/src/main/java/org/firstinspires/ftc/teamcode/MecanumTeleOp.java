package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

/**
 * Robot-configuration names:
 * <ul>
 *     <li>Control Hub 0: leftfront</li>
 *     <li>Control Hub 1: rightfront</li>
 *     <li>Control Hub 2: leftback</li>
 *     <li>Control Hub 3: rightback</li>
 *     <li>Expansion Hub 0: intake</li>
 *     <li>Expansion Hub 1: flymotor1</li>
 *     <li>Expansion Hub 2: flymotor2</li>
 *     <li>Expansion Hub 3: kicker</li>
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
    private static final String[] SPEED_MOTOR_NAMES = {"flymotor1", "flymotor2"};
    // 6000 RPM Yellow Jacket (1:1 internal gearbox), then 8-tooth driving 14-tooth.
    private static final double SPEED_MOTOR_TICKS_PER_REV = 28.0;
    private static final double SPEED_MOTOR_RPM = 6000.0;
    private static final double OUTPUT_REVS_PER_MOTOR_REV = 8.0 / 14.0;
    private static final int OUTPUT_RPM_STEP = 100;
    // Round down to a whole step below the nominal 3428.57 RPM output limit.
    private static final int MAX_OUTPUT_RPM = (int) (SPEED_MOTOR_RPM
            * OUTPUT_REVS_PER_MOTOR_REV / OUTPUT_RPM_STEP) * OUTPUT_RPM_STEP;
    private static final double TICKS_PER_SECOND_PER_OUTPUT_RPM =
            SPEED_MOTOR_TICKS_PER_REV / (60.0 * OUTPUT_REVS_PER_MOTOR_REV);
    private static final double KICKER_MOTOR_TICKS_PER_REV = 28.0;
    private static final double KICKER_MOTOR_RPM = 435.0;
    private static final double KICKER_GEAR_RATIO = 14.0 / 8.0;
    private static final double KICKER_OUTPUT_RPM = KICKER_MOTOR_RPM * KICKER_GEAR_RATIO;
    // REV velocity PIDF uses a 32767 full-scale output and ticks/second feedback.
    // Starting gains, not a substitute for measuring/tuning the assembled flywheel.
    private static final double FLY_VELOCITY_F = 32767.0
            / (SPEED_MOTOR_RPM * SPEED_MOTOR_TICKS_PER_REV / 60.0);
    private static final double FLY_VELOCITY_P = 0.1 * FLY_VELOCITY_F;
    private static final double FLY_VELOCITY_I = 0.1 * FLY_VELOCITY_P;
    private static final double FLY_VELOCITY_D = 0.0;
    private static final double NOMINAL_VOLTAGE = 12.0;
    private static final double HIVE_TURN_GAIN = 0.018;
    private static final double HIVE_MAX_TURN = 0.35;
    private static final double POLLEN_MAX_TURN = 0.28;
    private static final double POLLEN_STOP_Y = 0.86;

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

        DcMotor kicker = hardwareMap.get(DcMotor.class, "kicker");
        kicker.setPower(0);
        kicker.setDirection(DcMotor.Direction.FORWARD);
        setEncoderMode(kicker);

        DcMotorEx[] speedMotors = new DcMotorEx[SPEED_MOTOR_NAMES.length];
        for (int i = 0; i < speedMotors.length; i++) {
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, SPEED_MOTOR_NAMES[i]);
            speedMotors[i] = motor;
            motor.setPower(0);
            // Restore original motor-lead polarity: software reversal preserves
            // the desired CCW rotation without inverting the velocity feedback.
            motor.setDirection(DcMotor.Direction.REVERSE);
            MotorConfigurationType motorType = motor.getMotorType().clone();
            motorType.setTicksPerRev(SPEED_MOTOR_TICKS_PER_REV);
            motorType.setMaxRPM(SPEED_MOTOR_RPM);
            motor.setMotorType(motorType);
            setEncoderMode(motor);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setVelocityPIDFCoefficients(FLY_VELOCITY_P, FLY_VELOCITY_I,
                    FLY_VELOCITY_D, FLY_VELOCITY_F);
        }

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // Configure pipeline 0 as 36h11 AprilTags, 3.25 in.
        PollenDetector pollenDetector = new PollenDetector();
        VisionPortal webcamPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .setCameraResolution(new Size(320, 240))
                .addProcessor(pollenDetector)
                .build();

        telemetry.addLine("Verify directions/encoder pairing with Mecanum Wheel Test first");
        telemetry.addLine("Left stick: drive/strafe | Right stick X: rotate");
        telemetry.addLine("Left trigger: intake reverse | Right trigger: forward");
        telemetry.addLine("A: kicker forward | Y: kicker reverse");
        telemetry.addLine("Hold left stick button for 25% driving speed");
        telemetry.addLine("Bumpers: right +100 / left -100 output RPM | B: stop fly motors");
        telemetry.addLine("X: toggle visible-hive turn assist | D-pad down: toggle pollen pickup");
        telemetry.addLine("Manual turning cancels hive assist; any manual driving cancels pollen pickup");
        telemetry.addLine("Fly motors: restore original power-wire polarity before running");
        telemetry.addData("Fly motor output range", "0-%d RPM", MAX_OUTPUT_RPM);
        telemetry.setMsTransmissionInterval(100);
        telemetry.update();

        double[] targets = new double[wheels.length];
        int outputRpm = 0;
        boolean previousRightBumper = gamepad1.right_bumper;
        boolean previousLeftBumper = gamepad1.left_bumper;
        double nextVoltageSample = 0.0;
        double batteryVoltage = Double.NaN;
        double appliedVelocityF = FLY_VELOCITY_F;
        boolean hiveAssist = false;
        boolean pollenAssist = false;
        boolean previousX = gamepad1.x;
        boolean previousDpadDown = gamepad1.dpad_down;
        int selectedHive = -1;
        long lastPollenFrame = -1;
        int pollenConfirmations = 0;
        double lastPollenX = Double.NaN;
        double lastPollenY = Double.NaN;
        double intakeFinishUntil = 0;
        try {
            limelight.start();
            waitForStart();
            while (opModeIsActive()) {
                if (getRuntime() >= nextVoltageSample) {
                    batteryVoltage = Double.POSITIVE_INFINITY;
                    for (VoltageSensor sensor : hardwareMap.voltageSensor) {
                        double voltage = sensor.getVoltage();
                        if (voltage > 0 && !Double.isInfinite(voltage)) {
                            batteryVoltage = Math.min(batteryVoltage, voltage);
                        }
                    }
                    // Keep nominal feedforward if no valid sensor is available.
                    double compensatedF = Double.isInfinite(batteryVoltage)
                            ? FLY_VELOCITY_F
                            : FLY_VELOCITY_F * NOMINAL_VOLTAGE / batteryVoltage;
                    if (Math.abs(compensatedF - appliedVelocityF) > 0.05) {
                        for (DcMotorEx motor : speedMotors) {
                            motor.setVelocityPIDFCoefficients(FLY_VELOCITY_P, FLY_VELOCITY_I,
                                    FLY_VELOCITY_D, compensatedF);
                        }
                        appliedVelocityF = compensatedF;
                    }
                    nextVoltageSample = getRuntime() + 0.25;
                }
                // FTC gamepads report forward stick movement as negative Y.
                double drive = -deadband(gamepad1.left_stick_y);
                // Slightly compensate for the reduced lateral traction of mecanum wheels.
                double strafe = deadband(gamepad1.left_stick_x) * 1.1;
                double turn = deadband(gamepad1.right_stick_x);

                LLResult limelightResult = limelight.getLatestResult();
                List<HiveTagTracker.Cell> visibleCells =
                        HiveTagTracker.visibleCells(limelightResult);
                boolean xPressed = gamepad1.x;
                boolean downPressed = gamepad1.dpad_down;
                if (xPressed && !previousX) {
                    hiveAssist = !hiveAssist;
                    pollenAssist = false;
                    HiveTagTracker.Cell closest = HiveTagTracker.nearestVisible(visibleCells);
                    selectedHive = closest == null ? -1 : closest.index;
                }
                if (downPressed && !previousDpadDown) {
                    pollenAssist = !pollenAssist;
                    hiveAssist = false;
                    selectedHive = -1;
                    pollenConfirmations = 0;
                }
                previousX = xPressed;
                previousDpadDown = downPressed;

                if (hiveAssist && Math.abs(turn) > 0.15) {
                    hiveAssist = false;
                }
                if (pollenAssist && (Math.abs(drive) > 0.15 || Math.abs(strafe) > 0.15
                        || Math.abs(turn) > 0.15 || gamepad1.left_trigger > 0.05)) {
                    pollenAssist = false;
                }

                HiveTagTracker.Cell hive = selectedHive < 0
                        ? HiveTagTracker.nearestVisible(visibleCells)
                        : HiveTagTracker.find(visibleCells, selectedHive);
                if (hiveAssist) {
                    turn = 0;
                    if (hive != null) {
                        selectedHive = hive.index; // Lock the cell until assist is toggled off.
                        if (Math.abs(hive.bearingDegrees) > 1.5) {
                            turn = clamp(hive.bearingDegrees * HIVE_TURN_GAIN,
                                    -HIVE_MAX_TURN, HIVE_MAX_TURN);
                            if (Math.abs(turn) < 0.08) {
                                turn = Math.copySign(0.08, turn);
                            }
                        }
                    }
                }

                PollenDetector.Detection pollen = pollenDetector.getLatest();
                boolean freshPollen = pollen != null
                        && System.nanoTime() - pollen.observedNanos < 300_000_000L;
                if (pollenAssist) {
                    drive = 0;
                    strafe = 0;
                    turn = 0;
                    if (!freshPollen) {
                        pollenConfirmations = 0;
                    } else {
                        if (pollen.frameNumber != lastPollenFrame) {
                            if (Math.abs(pollen.x - lastPollenX) < 0.16
                                    && Math.abs(pollen.y - lastPollenY) < 0.16) {
                                pollenConfirmations++;
                            } else {
                                pollenConfirmations = 1;
                            }
                            lastPollenFrame = pollen.frameNumber;
                            lastPollenX = pollen.x;
                            lastPollenY = pollen.y;
                        }
                        if (pollenConfirmations >= 3) {
                            double error = (pollen.x - 0.5) * 2;
                            if (pollen.y >= POLLEN_STOP_Y) {
                                pollenAssist = false;
                                intakeFinishUntil = getRuntime() + 0.35;
                            } else {
                                turn = clamp(error * 0.50, -POLLEN_MAX_TURN,
                                        POLLEN_MAX_TURN);
                                drive = Math.abs(error) > 0.32 ? 0
                                        : (pollen.y < 0.55 ? 0.28 : 0.18);
                            }
                        }
                    }
                }

                targets[0] = drive + strafe + turn;
                targets[1] = drive - strafe - turn;
                targets[2] = drive - strafe + turn;
                targets[3] = drive + strafe - turn;

                // One common scale preserves the wheel-speed ratios, including diagonals.
                double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1.0);
                double speedLimit = MAX_TICKS_PER_SECOND * (gamepad1.left_stick_button
                        ? 0.25 : (hiveAssist || pollenAssist ? 0.35 : 1.0));
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
                if ((pollenAssist && pollenConfirmations >= 3 && freshPollen)
                        || getRuntime() < intakeFinishUntil) {
                    intakePower = 1.0;
                }
                if (gamepad1.left_trigger > 0.05) {
                    intakeFinishUntil = 0;
                }
                intake.setPower(intakePower);

                double kickerPower = 0.0;
                if (gamepad1.a) {
                    kickerPower = 1.0;
                } else if (gamepad1.y) {
                    kickerPower = -1.0;
                }
                kicker.setPower(kickerPower);

                boolean rightBumper = gamepad1.right_bumper;
                boolean leftBumper = gamepad1.left_bumper;
                // One step per press; holding both bumpers makes no change.
                if (gamepad1.b) {
                    outputRpm = 0;
                } else if (rightBumper && !previousRightBumper && !leftBumper) {
                    outputRpm = Math.min(outputRpm + OUTPUT_RPM_STEP, MAX_OUTPUT_RPM);
                } else if (leftBumper && !previousLeftBumper && !rightBumper) {
                    outputRpm = Math.max(outputRpm - OUTPUT_RPM_STEP, 0);
                }
                previousRightBumper = rightBumper;
                previousLeftBumper = leftBumper;

                double speedTarget = outputRpm * TICKS_PER_SECOND_PER_OUTPUT_RPM;
                telemetry.addData("Fly motor target", "%d output RPM", outputRpm);
                if (Double.isInfinite(batteryVoltage)) {
                    telemetry.addLine("Battery voltage unavailable: using nominal flywheel feedforward");
                } else {
                    telemetry.addData("Battery", "%.2f V", batteryVoltage);
                }
                telemetry.addData("Fly PIDF", "P %.3f | I %.3f | D %.3f | F %.3f",
                        FLY_VELOCITY_P, FLY_VELOCITY_I, FLY_VELOCITY_D, appliedVelocityF);
                for (int i = 0; i < speedMotors.length; i++) {
                    if (outputRpm == 0) {
                        speedMotors[i].setPower(0);
                    } else {
                        // The hub runs the only PID loop; F anticipates battery voltage changes.
                        speedMotors[i].setVelocity(speedTarget);
                    }
                    double ticksPerSecond = speedMotors[i].getVelocity();
                    double actualRpm = ticksPerSecond / TICKS_PER_SECOND_PER_OUTPUT_RPM;
                    telemetry.addData(SPEED_MOTOR_NAMES[i],
                            "actual %.0f output RPM | error %+.0f RPM | %.0f ticks/s",
                            actualRpm, outputRpm - actualRpm, ticksPerSecond);
                }

                telemetry.addData("Intake encoder", intake.getCurrentPosition());
                telemetry.addData("Kicker encoder", kicker.getCurrentPosition());
                telemetry.addData("Hive assist", hiveAssist ? "ON" : "OFF");
                telemetry.addData("Pollen pickup", pollenAssist ? "ON" : "OFF");
                for (HiveTagTracker.Cell cell : visibleCells) {
                    telemetry.addData("Hive " + cell.name,
                            "IDs %s | %d tags | bearing %+.1f deg",
                            cell.ids, cell.visibleTags, cell.bearingDegrees);
                }
                if (hiveAssist) {
                    telemetry.addData("Selected hive", hive == null
                            ? "waiting for visible tags" : hive.name);
                }
                if (freshPollen) {
                    telemetry.addData("Pollen", "x %.2f y %.2f area %.0f | confirmations %d",
                            pollen.x, pollen.y, pollen.area, pollenConfirmations);
                } else if (pollenAssist) {
                    telemetry.addLine("Pollen: waiting for fresh webcam frames");
                }
                telemetry.update();
                idle();
            }
        } finally {
            for (DcMotorEx wheel : wheels) {
                wheel.setPower(0);
            }
            intake.setPower(0);
            kicker.setPower(0);
            for (DcMotorEx motor : speedMotors) {
                motor.setPower(0);
            }
            webcamPortal.close();
            pollenDetector.close();
            limelight.stop();
        }
    }

    private double deadband(double value) {
        return Math.abs(value) <= 0.05 ? 0.0
                : Math.copySign((Math.abs(value) - 0.05) / 0.95, value);
    }

    private static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(high, value));
    }

    private void setEncoderMode(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
