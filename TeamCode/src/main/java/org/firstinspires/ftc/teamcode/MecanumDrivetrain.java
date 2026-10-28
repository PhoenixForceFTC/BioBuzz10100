package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Layer 1 Subsystem: Mecanum Drivetrain using four goBILDA 5202/5203 Series Bare Motors
 * (no planetary gearbox, 1:1 ratio, 6000 RPM, 28 quadrature encoder counts per revolution).
 *
 * Plain Java class (not an OpMode) so it can be reused across standalone Test OpModes,
 * TeleOp, and Autonomous.
 */
public class MecanumDrivetrain {

    // goBILDA 5202/5203 Series Bare Motor Specifications (1:1 Ratio, no gearbox)
    public static final double BARE_MOTOR_TICKS_PER_REV = BareMotorSpecifications.ENCODER_TICKS_PER_REV;
    public static final double BARE_MOTOR_MAX_RPM = BareMotorSpecifications.CONFIGURED_FREE_SPEED_RPM;
    public static final double LATERAL_STRAFE_MULTIPLIER = 1.1;

    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;

    private double lastFrontLeftPower = 0.0;
    private double lastFrontRightPower = 0.0;
    private double lastBackLeftPower = 0.0;
    private double lastBackRightPower = 0.0;

    public MecanumDrivetrain(HardwareMap hardwareMap) {
        frontLeft = getDriveMotor(hardwareMap, "leftfront", "front_left_drive", "frontLeft");
        frontRight = getDriveMotor(hardwareMap, "rightfront", "front_right_drive", "frontRight");
        backLeft = getDriveMotor(hardwareMap, "leftback", "back_left_drive", "backLeft");
        backRight = getDriveMotor(hardwareMap, "rightback", "back_right_drive", "backRight");

        configureBareMotor(frontLeft, DcMotor.Direction.REVERSE);
        configureBareMotor(backLeft, DcMotor.Direction.REVERSE);
        configureBareMotor(frontRight, DcMotor.Direction.FORWARD);
        configureBareMotor(backRight, DcMotor.Direction.FORWARD);
    }

    /**
     * Drives the mecanum chassis in robot-centric coordinates.
     *
     * @param axial   Forward (+1.0) / Backward (-1.0) translation
     * @param lateral Right (+1.0) / Left (-1.0) strafe translation
     * @param yaw     Clockwise (+1.0) / Counter-Clockwise (-1.0) rotation
     */
    public void driveRobotCentric(double axial, double lateral, double yaw) {
        double scaledLateral = lateral * LATERAL_STRAFE_MULTIPLIER;

        double flPower = axial + scaledLateral + yaw;
        double frPower = axial - scaledLateral - yaw;
        double blPower = axial - scaledLateral + yaw;
        double brPower = axial + scaledLateral - yaw;

        double max = Math.max(
                Math.max(Math.abs(flPower), Math.abs(frPower)),
                Math.max(Math.abs(blPower), Math.abs(brPower))
        );
        if (max > 1.0) {
            flPower /= max;
            frPower /= max;
            blPower /= max;
            brPower /= max;
        }

        setWheelPowers(flPower, frPower, blPower, brPower);
    }

    /**
     * Directly sets normalized wheel powers for individual wheel testing or custom control.
     */
    public void setWheelPowers(double flPower, double frPower, double blPower, double brPower) {
        lastFrontLeftPower = flPower;
        lastFrontRightPower = frPower;
        lastBackLeftPower = blPower;
        lastBackRightPower = brPower;

        frontLeft.setPower(flPower);
        frontRight.setPower(frPower);
        backLeft.setPower(blPower);
        backRight.setPower(brPower);
    }

    /**
     * Stops all four drivetrain motors.
     */
    public void stop() {
        setWheelPowers(0.0, 0.0, 0.0, 0.0);
    }

    /**
     * Adds drivetrain power, encoder position, and velocity data to telemetry.
     * Does not call telemetry.update() so the caller OpMode can flush telemetry once per loop.
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("FL", "pwr %+.2f | pos %d | vel %.0f ticks/s",
                lastFrontLeftPower, frontLeft.getCurrentPosition(), frontLeft.getVelocity());
        telemetry.addData("FR", "pwr %+.2f | pos %d | vel %.0f ticks/s",
                lastFrontRightPower, frontRight.getCurrentPosition(), frontRight.getVelocity());
        telemetry.addData("BL", "pwr %+.2f | pos %d | vel %.0f ticks/s",
                lastBackLeftPower, backLeft.getCurrentPosition(), backLeft.getVelocity());
        telemetry.addData("BR", "pwr %+.2f | pos %d | vel %.0f ticks/s",
                lastBackRightPower, backRight.getCurrentPosition(), backRight.getVelocity());
    }

    private void configureBareMotor(DcMotorEx motor, DcMotor.Direction direction) {
        motor.setPower(0);
        motor.setDirection(direction);

        // Clone the SDK motor configuration so we set 28 ticks/rev and the shared 5550 RPM baseline
        // without mutating the shared motor type singleton.
        MotorConfigurationType motorType = motor.getMotorType().clone();
        motorType.setTicksPerRev(BARE_MOTOR_TICKS_PER_REV);
        motorType.setMaxRPM(BARE_MOTOR_MAX_RPM);
        motorType.setAchieveableMaxRPMFraction(1.0);
        motorType.setGearing(1.0);
        motor.setMotorType(motorType);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private DcMotorEx getDriveMotor(HardwareMap hwMap, String... candidateNames) {
        for (String name : candidateNames) {
            DcMotorEx motor = hwMap.tryGet(DcMotorEx.class, name);
            if (motor != null) {
                return motor;
            }
        }
        return hwMap.get(DcMotorEx.class, candidateNames[0]);
    }
}
