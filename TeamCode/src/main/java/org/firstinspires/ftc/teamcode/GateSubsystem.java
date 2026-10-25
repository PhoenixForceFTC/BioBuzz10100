package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Shooter gate servo configured as gate_servo on Expansion Hub servo port 3. */
public class GateSubsystem {

    public static final String HW_GATE_SERVO = "gate_servo";
    public static final String SERVO_MODEL = "goBILDA Dual Mode Super Speed (2000-0025-0004)";
    // Servo must be programmed in positional mode: 300 deg over 500..2500 us.
    // https://www.gobilda.com/2000-series-dual-mode-servo-25-4-super-speed/
    public static final int MIN_PULSE_US = 500;
    public static final int MAX_PULSE_US = 2500;
    public static final double SERVO_TRAVEL_DEG = 300.0;
    public static final Servo.Direction SERVO_DIRECTION = Servo.Direction.REVERSE;
    public static final double CLOSED_ANGLE_DEG = SERVO_TRAVEL_DEG / 2.0;
    public static final double GATE_OPEN_TRAVEL_DEG = 90.0;
    // REVERSE maps logical 0.5 -> 0.2 to physical 0.5 -> 0.8: 1500 -> 2100 us, 90 deg CW.
    public static final double OPEN_ANGLE_DEG = CLOSED_ANGLE_DEG - GATE_OPEN_TRAVEL_DEG;
    public static final double TRIGGER_DEADBAND = 0.1;
    public static final double SECONDS_PER_60_DEG_AT_6V = 0.043;
    public static final double RATED_OPEN_TRAVEL_SECONDS = GATE_OPEN_TRAVEL_DEG
            / 60.0 * SECONDS_PER_60_DEG_AT_6V;
    // This delays feeding, not servo movement: setOpen sends the final target immediately.
    // Retain at least the previous 0.25 s allowance until loaded travel is measured.
    public static final double OPEN_SETTLE_MARGIN_SECONDS = 0.075;
    public static final double OPEN_SETTLE_TIME_SECONDS = Math.max(0.25,
            RATED_OPEN_TRAVEL_SECONDS + OPEN_SETTLE_MARGIN_SECONDS);

    private final ServoImplEx servo;
    private boolean open = false;
    private final ElapsedTime openTimer = new ElapsedTime();

    public GateSubsystem(HardwareMap hardwareMap) {
        servo = hardwareMap.get(ServoImplEx.class, HW_GATE_SERVO);
        servo.setDirection(SERVO_DIRECTION);
        servo.setPwmRange(new PwmControl.PwmRange(MIN_PULSE_US, MAX_PULSE_US));
        close();
    }

    public void setOpen(boolean open) {
        if (open && !this.open) {
            openTimer.reset();
        }
        this.open = open;
        double angleDeg = open ? OPEN_ANGLE_DEG : CLOSED_ANGLE_DEG;
        servo.setPosition(angleDeg / SERVO_TRAVEL_DEG);
    }

    public void close() {
        setOpen(false);
    }

    /** Estimates travel time; the servo has no physical position feedback. */
    public boolean isOpenAndSettled() {
        return open && openTimer.seconds() >= OPEN_SETTLE_TIME_SECONDS;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Gate Servo", SERVO_MODEL);
        telemetry.addData("Gate PWM Range (us)", "%d - %d | %s",
                MIN_PULSE_US, MAX_PULSE_US, SERVO_DIRECTION);
        telemetry.addData("Gate Open Travel (deg)", GATE_OPEN_TRAVEL_DEG);
        telemetry.addData("Gate Target (deg)", "%.0f (%s)",
                open ? OPEN_ANGLE_DEG : CLOSED_ANGLE_DEG, open ? "OPEN" : "CLOSED");
    }
}
