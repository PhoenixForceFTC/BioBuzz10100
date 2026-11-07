package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Open-loop intake on Expansion Hub motor port 1; encoder feedback is optional for callers. */
public class IntakeSubsystem {

    public static final String HW_INTAKE_MOTOR = "intake_motor";
    public static final double MOTOR_MAX_RPM = 1620.0;
    // External gearing: 10-tooth motor gear driving the 8-tooth intake gear.
    public static final double MOTOR_GEAR_TEETH = 10.0;
    public static final double INTAKE_GEAR_TEETH = 8.0;
    // Positive power turns the motor CCW when viewed facing the output shaft.
    public static final DcMotor.Direction MOTOR_DIRECTION = DcMotor.Direction.FORWARD;

    private final DcMotorEx motor;
    private final IntakeJamRecovery jamRecovery = new IntakeJamRecovery();
    private double currentAmps = Double.NaN, lastCurrentSample = Double.NEGATIVE_INFINITY;
    private double commandedPower = 0.0;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, HW_INTAKE_MOTOR);
        motor.setPower(0.0);
        motor.setDirection(MOTOR_DIRECTION);
        // Read the motor's own encoder without resetting it or enabling velocity regulation.
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setIntakePower(double power) {
        commandedPower = Range.clip(power, -1.0, 1.0);
        motor.setPower(commandedPower);
        double now = System.nanoTime()*1e-9;
        if (now-lastCurrentSample >= .2) {
            currentAmps = motor.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS);
            lastCurrentSample = now;
        }
    }

    /** Process a requested direction exactly once, using the intake's own encoder. */
    public void setIntakePowerWithJamRecovery(double now, double requestedPower) {
        setIntakePower(jamRecovery.update(now, Range.clip(requestedPower, -1, 1), getVelocityTicksPerSecond()));
    }
    public boolean isClearingJam() { return jamRecovery.isClearing(); }
    public double getCommandedPower() { return commandedPower; }

    /** Right bumper intakes; left bumper reverses. Both held stops the intake. */
    public void setBumperInputs(boolean rightBumper, boolean leftBumper) {
        double forward = rightBumper ? 1.0 : 0.0;
        double reverse = leftBumper ? 1.0 : 0.0;
        setIntakePowerWithJamRecovery(System.nanoTime() * 1e-9, forward - reverse);
    }

    public void stop() {
        jamRecovery.reset();
        commandedPower = 0; motor.setPower(0);
    }

    /** Only measures intake speed when this port has the intake motor's own encoder cable. */
    public double getVelocityTicksPerSecond() { return motor.getVelocity(); }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Intake current (A, 5 Hz)", currentAmps);
        telemetry.addData("Intake Power", "%+.2f (open loop)", commandedPower);
    }
}
