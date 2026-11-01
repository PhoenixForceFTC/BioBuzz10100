package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Axon MAX+ depollinator configured as depollinator on Expansion Hub servo port 4. */
public class DepollinatorSubsystem {

    public static final String HW_DEPOLLINATOR = "depollinator";
    public static final int MIN_PULSE_US = AxonMaxPlus.MIN_PULSE_US;
    public static final int MAX_PULSE_US = AxonMaxPlus.MAX_PULSE_US;
    // Match the existing Axon MAX+ configuration: positional mode, 355-degree programmed travel.
    public static final double SERVO_TRAVEL_DEG = AxonMaxPlus.PROGRAMMED_TRAVEL_DEG;
    public static final double HOME_ANGLE_DEG = 30.0;
    public static final double DEPLOY_TRAVEL_CW_DEG = 135.0;
    public static final double DEPLOYED_ANGLE_DEG = HOME_ANGLE_DEG + DEPLOY_TRAVEL_CW_DEG;
    // Viewed from the output shaft: program increasing PWM = CW, with no direction inversion.
    // Home/up is 30/355 = 0.08451; deployed is 165/355 = 0.46479 (135 deg CW).
    public static final Servo.Direction SERVO_DIRECTION = Servo.Direction.FORWARD;
    // Conservative SET BY HAND estimate, not measured travel or position feedback.
    public static final double SETTLE_ESTIMATE_SECONDS = 1.0;

    private final Servo servo;
    private final java.util.function.DoubleSupplier clock;
    private double commandedAt;
    private boolean deployed = false;

    public DepollinatorSubsystem(HardwareMap hardwareMap) {
        this(hardwareMap, () -> System.nanoTime() * 1e-9);
    }

    public DepollinatorSubsystem(HardwareMap hardwareMap, java.util.function.DoubleSupplier clock) {
        this.clock = clock;
        servo = hardwareMap.get(Servo.class, HW_DEPOLLINATOR);
        servo.setDirection(SERVO_DIRECTION);
        if (servo instanceof PwmControl) {
            ((PwmControl) servo).setPwmRange(new PwmControl.PwmRange(MIN_PULSE_US, MAX_PULSE_US));
        }
        home();
    }

    public void deploy() {
        commandedAt = clock.getAsDouble();
        deployed = true;
        servo.setPosition(DEPLOYED_ANGLE_DEG / SERVO_TRAVEL_DEG);
    }

    public void home() {
        commandedAt = clock.getAsDouble();
        deployed = false;
        servo.setPosition(HOME_ANGLE_DEG / SERVO_TRAVEL_DEG);
    }

    public boolean isDeployedCommanded() { return deployed; }
    public boolean isSettledEstimated() { return clock.getAsDouble() - commandedAt >= SETTLE_ESTIMATE_SECONDS; }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Depollinator Target (deg)", "%.1f (%s)",
                deployed ? DEPLOYED_ANGLE_DEG : HOME_ANGLE_DEG, deployed ? "DEPLOYED" : "HOME");
    }
}
