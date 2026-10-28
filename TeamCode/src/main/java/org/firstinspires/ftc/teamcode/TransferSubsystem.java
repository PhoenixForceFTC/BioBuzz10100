package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Two-stage transfer on Expansion Hub motor ports 2 and 3, without encoder cables. */
public class TransferSubsystem {

    public static final String HW_TRANSFER_MOTOR = "transfer_motor_1";
    public static final String HW_TRANSFER_MOTOR_2 = "transfer_motor_2";
    public static final String HW_FULL_SENSOR = "distance_sensor"; // Expansion Hub digital port 0
    public static final DcMotor.Direction MOTOR_DIRECTION = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction STAGE_2_MOTOR_DIRECTION = DcMotor.Direction.FORWARD;
    public static final double TRANSFER_POWER = 1.0;
    public static final double FULL_CONFIRM_TIME_SECONDS = 2.0;
    // goBILDA 3124-0001-0001 digital mode: HIGH = object within the configured threshold.
    public static final boolean FULL_SENSOR_ACTIVE_STATE = true;

    private final DcMotorEx motor;
    private final DcMotorEx stage2Motor;
    private final DigitalChannel fullSensor;
    private final TransferController controller = new TransferController(TRANSFER_POWER, FULL_CONFIRM_TIME_SECONDS);
    private boolean sensorState;
    private double current1 = Double.NaN, current2 = Double.NaN, lastCurrentSample = Double.NEGATIVE_INFINITY;
    private double commandedPower = 0.0;
    private double stage2Power = 0.0;
    private boolean feeding = false;
    private boolean loadingPaused = false;

    public TransferSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, HW_TRANSFER_MOTOR);
        motor.setPower(0.0);
        motor.setDirection(MOTOR_DIRECTION);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stage2Motor = hardwareMap.get(DcMotorEx.class, HW_TRANSFER_MOTOR_2);
        stage2Motor.setPower(0.0);
        stage2Motor.setDirection(STAGE_2_MOTOR_DIRECTION);
        stage2Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stage2Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fullSensor = hardwareMap.get(DigitalChannel.class, HW_FULL_SENSOR);
        fullSensor.setMode(DigitalChannel.Mode.INPUT);
        sensorState = fullSensor.getState();
    }

    /** Call once per active loop. The sensor's placement/threshold must indicate four balls loaded. */
    public void update() {
        update(false, false);
    }

    /**
     * Feed runs both stages regardless of the full sensor; caller must open the gate and wait its feed delay.
     * Otherwise stage 1 loads unless full or paused for aiming/gate delay; stage 2 stays stopped.
     * A confirmed full queue remains held until feeding begins, even if the sensor clears.
     */
    public void update(boolean feedBalls, boolean pauseLoading) {
        sensorState = fullSensor.getState();
        controller.update(System.nanoTime() * 1e-9, sensorState == FULL_SENSOR_ACTIVE_STATE, feedBalls, pauseLoading);
        feeding = feedBalls;
        loadingPaused = pauseLoading;
        commandedPower = controller.stage1();
        motor.setPower(commandedPower);
        stage2Power = controller.stage2();
        stage2Motor.setPower(stage2Power);
        double now = System.nanoTime()*1e-9;
        if (now-lastCurrentSample >= .2) {
            current1 = motor.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS);
            current2 = stage2Motor.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS);
            lastCurrentSample = now;
        }
    }

    public boolean isFull() {
        return controller.full();
    }

    public void stop() {
        commandedPower = 0.0;
        controller.stop();
        feeding = false;
        loadingPaused = false;
        stage2Power = 0.0;
        RobotShutdown.stopAll(() -> motor.setPower(0), () -> stage2Motor.setPower(0));
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Transfer current (A, 5 Hz)", "stage 1 %.2f | stage 2 %.2f",current1,current2);
        telemetry.addData("Transfer Mode", feeding ? "FEEDING SHOT"
                : isFull() ? "FULL: HOLDING UNTIL FEED"
                : loadingPaused ? "WAITING FOR SHOT / GATE" : "AUTOMATIC LOADING");
        telemetry.addData("Transfer Full Sensor", "%s | digital %s",
                isFull() ? "FULL" : sensorState == FULL_SENSOR_ACTIVE_STATE ? "CONFIRMING FULL" : "SPACE AVAILABLE",
                sensorState ? "HIGH" : "LOW");
        telemetry.addData("Transfer Full Confirmation (s)", "%.2f / %.2f",
                controller.confirmingSeconds(System.nanoTime() * 1e-9), FULL_CONFIRM_TIME_SECONDS);
        telemetry.addData("Transfer Stage 1 Power", "%.2f (REVERSE / CW, open loop)", commandedPower);
        telemetry.addData("Transfer Stage 2 Power", "%.2f (forward shot feed only; off while loading)", stage2Power);
    }
}
