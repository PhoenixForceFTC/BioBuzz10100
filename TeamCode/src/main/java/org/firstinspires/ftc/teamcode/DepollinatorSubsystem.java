package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

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

    private final Servo servo;
    private boolean deployed = false;

    public DepollinatorSubsystem(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, HW_DEPOLLINATOR);
        servo.setDirection(SERVO_DIRECTION);
        if (servo instanceof ServoImplEx) {
            ((ServoImplEx) servo).setPwmRange(new PwmControl.PwmRange(MIN_PULSE_US, MAX_PULSE_US));
        }
        home();
    }

    public void deploy() {
        deployed = true;
        servo.setPosition(DEPLOYED_ANGLE_DEG / SERVO_TRAVEL_DEG);
    }

    public void home() {
        deployed = false;
        servo.setPosition(HOME_ANGLE_DEG / SERVO_TRAVEL_DEG);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Depollinator Target (deg)", "%.1f (%s)",
                deployed ? DEPLOYED_ANGLE_DEG : HOME_ANGLE_DEG, deployed ? "DEPLOYED" : "HOME");
    }
}
