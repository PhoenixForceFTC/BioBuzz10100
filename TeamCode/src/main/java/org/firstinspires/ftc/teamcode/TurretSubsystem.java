package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Layer 1 Subsystem: Turret & Shooter Assembly with Limelight 3A Auto-Aiming.
 *
 * <p>Hardware Architecture (Expansion Hub + Control Hub USB 3.0):
 * <ul>
 *     <li><b>Turret Rotation Servos (2x Axon MAX, Servo Mode, 1:1 with turret)</b>:
 *         Expansion Hub Servo Ports 1 & 2 (powered via goBILDA 6V Distribution Block).
 *         Config names: {@code "turret_servo_1"} and {@code "turret_servo_2"}.</li>
 *     <li><b>Turret Rotation Encoder (REV Through-Bore Encoder V1, REV-11-1271)</b>:
 *         Expansion Hub Quadrature Port 1 (8192 CPR, geared 108T Turret : 40T Encoder = 2.7x faster).
 *         Read via Motor Port 1 {@code DcMotorEx} encoder channel.
 *         Config name: {@code "turret_encoder"}.</li>
 *     <li><b>Turret Pitch / Hood Servo (1x Axon MAX, Servo Mode, 2:1 linkage with hood angle)</b>:
 *         Expansion Hub Servo Port 0 (powered via goBILDA 6V Distribution Block).
 *         Adjustable from {@code 15.0 deg} (home/default) down to {@code 0.0 deg}.
 *         Config name: {@code "turret_pitch"}.</li>
 *     <li><b>Turret Shooter / Flywheel Motor (goBILDA 5202/5203 6000 RPM Bare YellowJacket)</b>:
 *         Expansion Hub Motor Port 0 & Quadrature Port 0 (28 ticks/rev, 6000 RPM, FLOAT).
 *         Config name: {@code "turret_motor"}.</li>
 *     <li><b>Limelight 3A Vision Sensor</b>:
 *         Control Hub USB 3.0 port (via powered USB hub), mounted at {@code 30.0 deg} above horizontal.
 *         Config name: {@code "limelight"}.</li>
 * </ul>
 */
public class TurretSubsystem {

    // =========================================================================
    // HARDWARE MAP CONFIGURATION NAMES
    // =========================================================================
    public static final String HW_ROTATION_SERVO_1 = "turret_servo_1"; // Exp Hub Servo Port 1
    public static final String HW_ROTATION_SERVO_2 = "turret_servo_2"; // Exp Hub Servo Port 2
    public static final String HW_PITCH_SERVO      = "turret_pitch";   // Exp Hub Servo Port 0
    public static final String HW_TURRET_MOTOR     = "turret_motor";   // Exp Hub Motor & Quad Port 0
    public static final String HW_TURRET_ENCODER   = "turret_encoder"; // Exp Hub Motor/Quad Port 1
    public static final String HW_LIMELIGHT        = "limelight";      // Control Hub USB 3.0

    // =========================================================================
    // AXON MAX SERVO PULSE WIDTH & RANGE CONSTANTS
    // =========================================================================
    public static final int AXON_MIN_PULSE_US = 500;
    public static final int AXON_MAX_PULSE_US = 2500;
    public static final double AXON_MAX_TRAVEL_DEG = 355.0; // Default Axon MAX servo travel across 0.0..1.0

    // =========================================================================
    // TURRET ROTATION (YAW) & DEADZONE CONSTANTS (1:1 GEARING)
    // =========================================================================
    // Center (0.5 servo position) = 0.0 deg turret angle (pointing right, opposite the left deadzone).
    // Deadzone of +/-10 deg at the back-left (-180 deg / +180 deg wrap point) prevents wire wrap
    // and keeps the servos safely away from the 0.0 / 1.0 PWM discontinuity.
    public static final double ROTATION_GEAR_RATIO = 1.0; // 1:1 servo-to-turret rotation
    public static final double ROTATION_HOME_SERVO_POS = 0.5;
    public static final double ROTATION_HOME_ANGLE_DEG = 0.0;
    public static final double ROTATION_DEADZONE_HALF_DEG = 10.0; // +/- 10 deg deadzone on the left
    public static final double ROTATION_MIN_ANGLE_DEG = -180.0 + ROTATION_DEADZONE_HALF_DEG; // -170.0 deg
    public static final double ROTATION_MAX_ANGLE_DEG =  180.0 - ROTATION_DEADZONE_HALF_DEG; // +170.0 deg

    // Set to -1.0 if positive angle commands move the turret in the opposite direction of encoder ticks
    public static final double ROTATION_SERVO_DIRECTION_SIGN = 1.0;

    // Both rotation servos mesh with the turret ring in the same orientation by default;
    // flip ROTATION_SERVO_2_DIRECTION if they are mounted inverted relative to each other.
    public static final Servo.Direction ROTATION_SERVO_1_DIRECTION = Servo.Direction.FORWARD;
    public static final Servo.Direction ROTATION_SERVO_2_DIRECTION = Servo.Direction.FORWARD;

    // =========================================================================
    // REV THROUGH-BORE ENCODER V1 (QUAD PORT 1, 108T TURRET : 40T ENCODER)
    // =========================================================================
    // REV-11-1271 quadrature resolution = 8192 counts per revolution (2048 CPR * 4x).
    // Geared 108T on turret/servos : 40T on encoder => encoder spins 2.7x faster than turret.
    public static final double THROUGH_BORE_CPR = 8192.0;
    public static final double TURRET_TO_ENCODER_GEAR_RATIO = 108.0 / 40.0; // 2.7
    public static final double TURRET_TICKS_PER_REV = THROUGH_BORE_CPR * TURRET_TO_ENCODER_GEAR_RATIO; // 22,118.4 ticks/rev
    public static final double TURRET_TICKS_PER_DEGREE = TURRET_TICKS_PER_REV / 360.0; // 61.44 ticks/deg
    public static final double ENCODER_DIRECTION_SIGN = 1.0; // Flip to -1.0 if encoder counts decrease on +angle

    // Closed-loop encoder trim & servo cutout/slip correction constants
    public static final double ENCODER_TRIM_KP = 0.35;              // Integral/proportional trim gain
    public static final double MAX_ENCODER_TRIM_DEG = 25.0;         // Max servo command correction (deg)
    public static final double ENCODER_TRIM_DEADBAND_DEG = 0.35;    // Ignore tiny sub-degree encoder noise
    public static final double SERVO_CUTOUT_ERROR_THRESHOLD_DEG = 4.0; // Flag warning if |target - enc| > 4 deg

    // =========================================================================
    // TURRET PITCH / HOOD SERVO CONSTANTS (PORT 0, 2:1 LINKAGE, 15 DEG TO 0 DEG)
    // =========================================================================
    // Linked 2:1 with turret angle: 1 deg of turret pitch requires 2 deg of Axon MAX servo rotation.
    // At 15.0 deg (default/home), servo position = PITCH_SERVO_POS_AT_15_DEG (0.0).
    // Moving 15 deg of turret pitch (15 deg -> 0 deg) rotates the servo by 30.0 deg.
    public static final double PITCH_MIN_DEG = 0.0;
    public static final double PITCH_MAX_DEG = 15.0;
    public static final double PITCH_HOME_DEG = 15.0;
    public static final double PITCH_SERVO_TO_TURRET_RATIO = 2.0; // 2:1 servo-to-pitch linkage
    public static final double PITCH_SERVO_POS_AT_15_DEG = 0.0;
    public static final double PITCH_SERVO_DEG_PER_UNIT = AXON_MAX_TRAVEL_DEG; // 355.0 deg across 0.0..1.0
    public static final double PITCH_SERVO_DIRECTION_SIGN = 1.0; // +1.0 if pos increases from 15 deg -> 0 deg
    public static final Servo.Direction PITCH_SERVO_DIRECTION = Servo.Direction.FORWARD;

    // =========================================================================
    // TURRET SHOOTER / FLYWHEEL MOTOR CONSTANTS (PORT 0, 6000 RPM BARE MOTOR)
    // =========================================================================
    public static final double SHOOTER_MOTOR_TICKS_PER_REV = 28.0;
    public static final double SHOOTER_MOTOR_MAX_RPM = 6000.0;
    public static final double SHOOTER_DEFAULT_TARGET_RPM = 4200.0;
    public static final double SHOOTER_MIN_AUTO_RPM = 3200.0;
    public static final double SHOOTER_MAX_AUTO_RPM = 5600.0;
    public static final double SHOOTER_READY_TOLERANCE_RPM = 150.0;

    // =========================================================================
    // LIMELIGHT 3A MOUNTING & GOAL AUTO-AIM CONSTANTS (30 DEG MOUNT ANGLE)
    // =========================================================================
    public static final double LIMELIGHT_MOUNT_ANGLE_DEG = 30.0; // Angled 30 deg above horizontal toward goal
    public static final double LIMELIGHT_LENS_HEIGHT_INCHES = 12.0; // Tune to exact lens height above floor
    public static final double GOAL_TARGET_HEIGHT_INCHES = 36.0;    // Tune to target goal center height
    public static final int LIMELIGHT_DEFAULT_PIPELINE = 0;

    // Limelight auto-aim gains & distance interpolation bounds
    public static final double LIMELIGHT_YAW_KP = 0.65; // Fraction of tx applied per update using measured encoder yaw
    public static final double LIMELIGHT_YAW_DEADBAND_DEG = 0.5;
    public static final double AUTO_AIM_NEAR_DISTANCE_IN = 24.0; // At or closer than this -> 15 deg pitch, min RPM
    public static final double AUTO_AIM_FAR_DISTANCE_IN = 120.0; // At or farther than this -> 0 deg pitch, max RPM

    // =========================================================================
    // HARDWARE DEVICES
    // =========================================================================
    private final Servo rotationServo1;
    private final Servo rotationServo2;
    private final Servo pitchServo;
    private final DcMotorEx shooterMotor;
    private final DcMotorEx throughBoreEncoderPort;
    private final Limelight3A limelight;

    // =========================================================================
    // STATE VARIABLES
    // =========================================================================
    private int encoderZeroOffsetTicks = 0;
    private boolean hasHomed = false;
    private boolean encoderCorrectionEnabled = true;

    private double targetYawDeg = ROTATION_HOME_ANGLE_DEG;
    private double encoderTrimDeg = 0.0;
    private double lastCommandedRotationServoPos = ROTATION_HOME_SERVO_POS;

    private double targetPitchDeg = PITCH_HOME_DEG;
    private double lastCommandedPitchServoPos = PITCH_SERVO_POS_AT_15_DEG;

    private double targetShooterRpm = 0.0;

    private boolean limelightValid = false;
    private double limelightTxDeg = 0.0;
    private double limelightTyDeg = 0.0;
    private double limelightTa = 0.0;
    private double estimatedGoalDistanceInches = 0.0;

    public TurretSubsystem(HardwareMap hardwareMap) {
        rotationServo1 = getDevice(hardwareMap, Servo.class,
                HW_ROTATION_SERVO_1, "turretServo1", "turret_left", "turret1");
        rotationServo2 = getDevice(hardwareMap, Servo.class,
                HW_ROTATION_SERVO_2, "turretServo2", "turret_right", "turret2");
        pitchServo = getDevice(hardwareMap, Servo.class,
                HW_PITCH_SERVO, "turretPitch", "hood_servo", "pitch_servo");
        shooterMotor = getDevice(hardwareMap, DcMotorEx.class,
                HW_TURRET_MOTOR, "turretMotor", "shooter_motor", "shooter", "flywheel");
        throughBoreEncoderPort = getDevice(hardwareMap, DcMotorEx.class,
                HW_TURRET_ENCODER, "turretEncoder", "through_bore", "turret_enc");
        limelight = tryGetDevice(hardwareMap, Limelight3A.class,
                HW_LIMELIGHT, "limelight3a");

        configureServos();
        configureShooterMotor();
        configureThroughBoreEncoder();
        configureLimelight();

        // Home the turret servos to center (0 deg) and pitch to 15 deg on initialization,
        // and zero the REV Through-Bore Encoder relative to this home position.
        homeTurretAndZeroEncoder();
        setPitchAngleDegrees(PITCH_HOME_DEG);
    }

    // =========================================================================
    // INITIALIZATION & HOMING
    // =========================================================================

    private void configureServos() {
        configureAxonServo(rotationServo1, ROTATION_SERVO_1_DIRECTION);
        configureAxonServo(rotationServo2, ROTATION_SERVO_2_DIRECTION);
        configureAxonServo(pitchServo, PITCH_SERVO_DIRECTION);
    }

    private void configureAxonServo(Servo servo, Servo.Direction direction) {
        servo.setDirection(direction);
        if (servo instanceof ServoImplEx) {
            ((ServoImplEx) servo).setPwmRange(
                    new PwmControl.PwmRange(AXON_MIN_PULSE_US, AXON_MAX_PULSE_US));
        }
    }

    private void configureShooterMotor() {
        shooterMotor.setPower(0.0);
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);

        MotorConfigurationType motorType = shooterMotor.getMotorType().clone();
        motorType.setTicksPerRev(SHOOTER_MOTOR_TICKS_PER_REV);
        motorType.setMaxRPM(SHOOTER_MOTOR_MAX_RPM);
        motorType.setGearing(1.0);
        shooterMotor.setMotorType(motorType);

        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Flywheels should coast (FLOAT) rather than brake abruptly
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    private void configureThroughBoreEncoder() {
        // The REV Through-Bore Encoder on Expansion Hub Quadrature Port 1 is read through
        // Motor Port 1's encoder counter without energizing Motor Port 1.
        throughBoreEncoderPort.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        throughBoreEncoderPort.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void configureLimelight() {
        if (limelight != null) {
            limelight.pipelineSwitch(LIMELIGHT_DEFAULT_PIPELINE);
            limelight.start();
        }
    }

    /**
     * Commands the rotation servos to the center home position (0.5 = 0.0 deg) and zeroes
     * the REV Through-Bore Encoder at the current reading.
     */
    public void homeTurretAndZeroEncoder() {
        targetYawDeg = ROTATION_HOME_ANGLE_DEG;
        encoderTrimDeg = 0.0;
        setRawRotationServoPosition(ROTATION_HOME_SERVO_POS);
        zeroEncoderAtCurrentPosition();
        hasHomed = true;
    }

    /**
     * Re-zeroes the REV Through-Bore Encoder at the turret's current physical position
     * (treating the current turret orientation as 0.0 deg).
     */
    public void zeroEncoderAtCurrentPosition() {
        encoderZeroOffsetTicks = throughBoreEncoderPort.getCurrentPosition();
        encoderTrimDeg = 0.0;
    }

    // =========================================================================
    // MAIN LOOP UPDATE (CALL ONCE PER OPMODE LOOP)
    // =========================================================================

    /**
     * Updates Limelight vision readings and applies REV Through-Bore Encoder closed-loop
     * verification/trim to the turret rotation servos so the turret holds its exact target
     * angle even under load or servo position droop.
     */
    public void update() {
        updateLimelightData();
        applyRotationControl();
    }

    private void updateLimelightData() {
        if (limelight == null) {
            limelightValid = false;
            return;
        }

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            limelightValid = true;
            limelightTxDeg = result.getTx();
            limelightTyDeg = result.getTy();
            limelightTa = result.getTa();
            estimatedGoalDistanceInches = calculateGoalDistanceInches(limelightTyDeg);
        } else {
            limelightValid = false;
        }
    }

    private void applyRotationControl() {
        double clampedTargetDeg = clampYawAngle(targetYawDeg);
        targetYawDeg = clampedTargetDeg;

        if (encoderCorrectionEnabled && hasHomed) {
            double measuredDeg = getEncoderAngleDegrees();
            double errorDeg = clampedTargetDeg - measuredDeg;
            if (Math.abs(errorDeg) > ENCODER_TRIM_DEADBAND_DEG) {
                encoderTrimDeg = Range.clip(
                        encoderTrimDeg + errorDeg * ENCODER_TRIM_KP,
                        -MAX_ENCODER_TRIM_DEG,
                        MAX_ENCODER_TRIM_DEG
                );
            }
        } else {
            encoderTrimDeg = 0.0;
        }

        double effectiveAngleDeg = Range.clip(
                clampedTargetDeg + encoderTrimDeg,
                ROTATION_MIN_ANGLE_DEG,
                ROTATION_MAX_ANGLE_DEG
        );
        double servoPos = angleDegreesToRotationServoPos(effectiveAngleDeg);
        setRawRotationServoPosition(servoPos);
    }

    // =========================================================================
    // TURRET ROTATION (YAW) CONTROL & ENCODER TELEMETRY
    // =========================================================================

    /**
     * Sets the desired turret rotation angle in degrees, clamped to the safe travel zone
     * [{@code -170.0 deg}, {@code +170.0 deg}] (preserving the +/-10 deg left deadzone).
     */
    public void setTargetYawDegrees(double angleDeg) {
        this.targetYawDeg = clampYawAngle(angleDeg);
        applyRotationControl();
    }

    /**
     * Adjusts the desired turret rotation angle by {@code deltaDeg} degrees.
     */
    public void adjustTargetYawDegrees(double deltaDeg) {
        setTargetYawDegrees(targetYawDeg + deltaDeg);
    }

    public double getTargetYawDegrees() {
        return targetYawDeg;
    }

    /**
     * Returns the raw REV Through-Bore Encoder tick count relative to the homed zero offset.
     */
    public int getEncoderTicks() {
        return throughBoreEncoderPort.getCurrentPosition() - encoderZeroOffsetTicks;
    }

    /**
     * Returns the turret rotation angle in degrees measured by the 108:40 REV Through-Bore Encoder
     * relative to the homed 0.0 deg reference.
     */
    public double getEncoderAngleDegrees() {
        return (getEncoderTicks() / TURRET_TICKS_PER_DEGREE) * ENCODER_DIRECTION_SIGN;
    }

    /**
     * Returns the turret angular velocity in degrees/second from the REV Through-Bore Encoder.
     */
    public double getEncoderVelocityDegPerSec() {
        return (throughBoreEncoderPort.getVelocity() / TURRET_TICKS_PER_DEGREE) * ENCODER_DIRECTION_SIGN;
    }

    /**
     * Returns true if the difference between the target angle and the Through-Bore Encoder angle
     * exceeds {@link #SERVO_CUTOUT_ERROR_THRESHOLD_DEG}, indicating servo stall, slip, or cutout.
     */
    public boolean isRotationErrorHigh() {
        return Math.abs(targetYawDeg - getEncoderAngleDegrees()) > SERVO_CUTOUT_ERROR_THRESHOLD_DEG;
    }

    public void setEncoderCorrectionEnabled(boolean enabled) {
        this.encoderCorrectionEnabled = enabled;
        if (!enabled) {
            this.encoderTrimDeg = 0.0;
        }
    }

    public boolean isEncoderCorrectionEnabled() {
        return encoderCorrectionEnabled;
    }

    /**
     * Directly sets both rotation Axon MAX servos to the same raw position [0.0, 1.0].
     * Used internally and by {@link ServoZeroCalibrationOpMode}.
     */
    public void setRawRotationServoPosition(double position) {
        double clamped = Range.clip(position, 0.0, 1.0);
        lastCommandedRotationServoPos = clamped;
        rotationServo1.setPosition(clamped);
        rotationServo2.setPosition(clamped);
    }

    public double getLastCommandedRotationServoPos() {
        return lastCommandedRotationServoPos;
    }

    public static double clampYawAngle(double angleDeg) {
        return Range.clip(angleDeg, ROTATION_MIN_ANGLE_DEG, ROTATION_MAX_ANGLE_DEG);
    }

    public static double angleDegreesToRotationServoPos(double angleDeg) {
        double servoDegFromCenter = (angleDeg - ROTATION_HOME_ANGLE_DEG)
                * ROTATION_GEAR_RATIO
                * ROTATION_SERVO_DIRECTION_SIGN;
        double pos = ROTATION_HOME_SERVO_POS + (servoDegFromCenter / AXON_MAX_TRAVEL_DEG);
        return Range.clip(pos, 0.0, 1.0);
    }

    // =========================================================================
    // TURRET PITCH / HOOD SERVO CONTROL (15 DEG TO 0 DEG, 2:1 LINKAGE)
    // =========================================================================

    /**
     * Sets the turret hood/pitch angle in degrees, clamped between {@code 0.0 deg} and {@code 15.0 deg}.
     * Uses the 2:1 Axon MAX linkage ratio (1 deg of turret pitch = 2 deg of servo rotation).
     */
    public void setPitchAngleDegrees(double pitchDeg) {
        targetPitchDeg = Range.clip(pitchDeg, PITCH_MIN_DEG, PITCH_MAX_DEG);
        double servoPos = pitchDegreesToServoPos(targetPitchDeg);
        setRawPitchServoPosition(servoPos);
    }

    public void adjustPitchAngleDegrees(double deltaDeg) {
        setPitchAngleDegrees(targetPitchDeg + deltaDeg);
    }

    public double getTargetPitchDegrees() {
        return targetPitchDeg;
    }

    /**
     * Directly sets the pitch Axon MAX servo on Port 0 to a raw position [0.0, 1.0].
     */
    public void setRawPitchServoPosition(double position) {
        double clamped = Range.clip(position, 0.0, 1.0);
        lastCommandedPitchServoPos = clamped;
        pitchServo.setPosition(clamped);
    }

    public double getLastCommandedPitchServoPos() {
        return lastCommandedPitchServoPos;
    }

    public static double pitchDegreesToServoPos(double pitchDeg) {
        double clampedPitch = Range.clip(pitchDeg, PITCH_MIN_DEG, PITCH_MAX_DEG);
        // Delta from the 15.0 deg starting position: moving from 15 deg down to 0 deg
        // moves the 2:1 servo by (15 - pitchDeg) * 2.0 degrees.
        double turretDeltaFrom15Deg = PITCH_HOME_DEG - clampedPitch;
        double servoRotationDeg = turretDeltaFrom15Deg * PITCH_SERVO_TO_TURRET_RATIO;
        double pos = PITCH_SERVO_POS_AT_15_DEG
                + PITCH_SERVO_DIRECTION_SIGN * (servoRotationDeg / PITCH_SERVO_DEG_PER_UNIT);
        return Range.clip(pos, 0.0, 1.0);
    }

    // =========================================================================
    // SHOOTER / FLYWHEEL MOTOR CONTROL (PORT 0, 6000 RPM BARE YELLOWJACKET)
    // =========================================================================

    /**
     * Commands the 6000 RPM turret shooter motor to a closed-loop velocity in RPM.
     */
    public void setShooterRpm(double rpm) {
        targetShooterRpm = Range.clip(rpm, -SHOOTER_MOTOR_MAX_RPM, SHOOTER_MOTOR_MAX_RPM);
        double ticksPerSec = (targetShooterRpm / 60.0) * SHOOTER_MOTOR_TICKS_PER_REV;
        shooterMotor.setVelocity(ticksPerSec);
    }

    /**
     * Commands the shooter motor using open-loop normalized power [-1.0, 1.0].
     */
    public void setShooterPower(double power) {
        double clamped = Range.clip(power, -1.0, 1.0);
        targetShooterRpm = clamped * SHOOTER_MOTOR_MAX_RPM;
        shooterMotor.setPower(clamped);
    }

    /**
     * Stops the shooter motor (coasts down because ZeroPowerBehavior is FLOAT).
     */
    public void stopShooter() {
        targetShooterRpm = 0.0;
        shooterMotor.setPower(0.0);
    }

    public double getShooterCurrentRpm() {
        return (shooterMotor.getVelocity() / SHOOTER_MOTOR_TICKS_PER_REV) * 60.0;
    }

    public double getTargetShooterRpm() {
        return targetShooterRpm;
    }

    public boolean isShooterAtSpeed() {
        return targetShooterRpm > 0.0
                && Math.abs(getShooterCurrentRpm() - targetShooterRpm) <= SHOOTER_READY_TOLERANCE_RPM;
    }

    // =========================================================================
    // LIMELIGHT 3A AUTO-AIMING & DISTANCE-BASED PITCH / RPM
    // =========================================================================

    /**
     * Uses the Limelight 3A (angled 30 deg up toward the goal) to:
     * <ol>
     *     <li>Track goal yaw using {@code tx} combined with the Through-Bore Encoder's current angle.</li>
     *     <li>Estimate horizontal distance from {@code ty} + the 30 deg mount angle and automatically
     *         interpolate the turret hood angle ({@code 15 deg} to {@code 0 deg}) and flywheel RPM
     *         (when {@code autoPitchAndRpm} is true).</li>
     * </ol>
     *
     * @param autoPitchAndRpm true to also adjust the 15-0 deg pitch servo and shooter target RPM from ty
     * @param spinShooter     true to spin the shooter motor at the distance-computed RPM
     * @return true if a valid Limelight target was tracked this loop
     */
    public boolean autoAimWithLimelight(boolean autoPitchAndRpm, boolean spinShooter) {
        if (!limelightValid) {
            return false;
        }

        // 1. Yaw tracking: Limelight tx is horizontal offset in degrees (+right, -left).
        // Combine current measured turret angle with proportional tx step and clamp to +/-170 deg.
        if (Math.abs(limelightTxDeg) > LIMELIGHT_YAW_DEADBAND_DEG) {
            double currentMeasuredYaw = getEncoderAngleDegrees();
            // Subtract or add tx depending on turret positive rotation convention;
            // using -tx when +yaw is counter-clockwise (or flip ROTATION_SERVO_DIRECTION_SIGN).
            double desiredYaw = currentMeasuredYaw - (limelightTxDeg * LIMELIGHT_YAW_KP);
            setTargetYawDegrees(desiredYaw);
        }

        // 2. Distance-based Pitch (15 deg near -> 0 deg far) and Flywheel RPM
        if (autoPitchAndRpm && estimatedGoalDistanceInches > 0.0) {
            double t = Range.clip(
                    (estimatedGoalDistanceInches - AUTO_AIM_NEAR_DISTANCE_IN)
                            / (AUTO_AIM_FAR_DISTANCE_IN - AUTO_AIM_NEAR_DISTANCE_IN),
                    0.0,
                    1.0
            );
            // Near shot = 15 deg pitch; Far shot = 0 deg pitch (flatter trajectory / adjustable)
            double computedPitchDeg = PITCH_MAX_DEG - t * (PITCH_MAX_DEG - PITCH_MIN_DEG);
            setPitchAngleDegrees(computedPitchDeg);

            if (spinShooter) {
                double computedRpm = SHOOTER_MIN_AUTO_RPM
                        + t * (SHOOTER_MAX_AUTO_RPM - SHOOTER_MIN_AUTO_RPM);
                setShooterRpm(computedRpm);
            }
        } else if (spinShooter) {
            setShooterRpm(SHOOTER_DEFAULT_TARGET_RPM);
        }

        return true;
    }

    /**
     * Computes planar distance (in inches) to the goal using the Limelight's 30.0 deg upward tilt
     * and the vertical target offset {@code ty}:
     * {@code distance = (goalHeight - lensHeight) / tan(30 deg + ty)}.
     */
    public static double calculateGoalDistanceInches(double tyDegrees) {
        double totalElevationDeg = LIMELIGHT_MOUNT_ANGLE_DEG + tyDegrees;
        if (totalElevationDeg <= 1.0 || totalElevationDeg >= 89.0) {
            return 0.0;
        }
        double heightDiffInches = Math.max(1.0, GOAL_TARGET_HEIGHT_INCHES - LIMELIGHT_LENS_HEIGHT_INCHES);
        return heightDiffInches / Math.tan(Math.toRadians(totalElevationDeg));
    }

    public boolean hasLimelightTarget() {
        return limelightValid;
    }

    public double getLimelightTxDegrees() {
        return limelightTxDeg;
    }

    public double getLimelightTyDegrees() {
        return limelightTyDeg;
    }

    public double getEstimatedGoalDistanceInches() {
        return estimatedGoalDistanceInches;
    }

    public void setLimelightPipeline(int pipelineIndex) {
        if (limelight != null) {
            limelight.pipelineSwitch(pipelineIndex);
        }
    }

    /**
     * Sets all three turret Axon MAX servos (rotation 1, rotation 2, pitch) to 0.0 for horn installation.
     */
    public void setAllServosToZeroPosition() {
        encoderCorrectionEnabled = false;
        setRawRotationServoPosition(0.0);
        setRawPitchServoPosition(0.0);
    }

    /**
     * Stops the shooter motor and Limelight polling when the OpMode ends.
     */
    public void stop() {
        stopShooter();
        if (limelight != null) {
            limelight.stop();
        }
    }

    // =========================================================================
    // TELEMETRY
    // =========================================================================

    public void addTelemetry(Telemetry telemetry) {
        double encDeg = getEncoderAngleDegrees();
        double yawError = targetYawDeg - encDeg;

        telemetry.addData("Turret Yaw (deg)",
                "Target: %+.1f deg | Enc: %+.2f deg | Err: %+.2f deg%s",
                targetYawDeg, encDeg, yawError,
                isRotationErrorHigh() ? " [WARN: SLIP/CUTOUT]" : "");
        telemetry.addData("Turret Yaw Servo/Enc",
                "ServoPos: %.3f | Trim: %+.2f deg | EncTicks: %d (%.0f deg/s)",
                lastCommandedRotationServoPos, encoderTrimDeg, getEncoderTicks(), getEncoderVelocityDegPerSec());
        telemetry.addData("Turret Pitch (2:1)",
                "Target: %.1f deg (range 15-0 deg) | ServoPos: %.3f",
                targetPitchDeg, lastCommandedPitchServoPos);
        telemetry.addData("Shooter (Port 0)",
                "RPM: %.0f / %.0f (%s) | Vel: %.0f ticks/s",
                getShooterCurrentRpm(), targetShooterRpm,
                isShooterAtSpeed() ? "READY" : "SPINNING",
                shooterMotor.getVelocity());

        if (limelight != null) {
            LLStatus status = limelight.getStatus();
            if (limelightValid) {
                telemetry.addData("Limelight (30 deg)",
                        "VALID | tx: %+.2f deg | ty: %+.2f deg | ta: %.2f%% | Dist: %.1f in",
                        limelightTxDeg, limelightTyDeg, limelightTa, estimatedGoalDistanceInches);
            } else {
                telemetry.addData("Limelight (30 deg)",
                        "NO TARGET | Pipe: %d | FPS: %d",
                        status.getPipelineIndex(), (int) status.getFps());
            }
        } else {
            telemetry.addData("Limelight", "Not configured in HardwareMap");
        }
    }

    private <T> T getDevice(HardwareMap hwMap, Class<T> clazz, String... candidateNames) {
        T device = tryGetDevice(hwMap, clazz, candidateNames);
        if (device != null) {
            return device;
        }
        return hwMap.get(clazz, candidateNames[0]);
    }

    private <T> T tryGetDevice(HardwareMap hwMap, Class<T> clazz, String... candidateNames) {
        for (String name : candidateNames) {
            T device = hwMap.tryGet(clazz, name);
            if (device != null) {
                return device;
            }
        }
        return null;
    }
}
