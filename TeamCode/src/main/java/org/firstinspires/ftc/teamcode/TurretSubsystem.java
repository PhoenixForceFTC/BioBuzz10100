package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
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

import java.util.List;

/**
 * Layer 1 Subsystem: Turret & Shooter Assembly with Limelight 3A Auto-Aiming.
 *
 * <p>Hardware Architecture (Expansion Hub + Control Hub USB 3.0):
 * <ul>
 *     <li><b>Turret Rotation Servos (2x Axon MAX+, Servo Mode, 1:1 with turret)</b>:
 *         Expansion Hub Servo Ports 1 & 2 (powered via goBILDA 6V Distribution Block).
 *         Config names: {@code "turret_servo_1"} and {@code "turret_servo_2"}.</li>
 *     <li><b>Turret Rotation Encoder (REV Through-Bore Encoder V1, REV-11-1271)</b>:
 *         Expansion Hub Quadrature Port 1 (8192 CPR, geared 108T Turret : 40T Encoder = 2.7x faster).
 *         Read via Motor Port 1 {@code DcMotorEx} encoder channel.
 *         Config name: {@code "turret_encoder"}.</li>
 *     <li><b>Turret Pitch / Hood Servo (1x Axon MAX+, Servo Mode, 2:1 linkage with hood angle)</b>:
 *         Expansion Hub Servo Port 0 (powered via goBILDA 6V Distribution Block).
 *         Home is {@code 0.5}, the highest launch angle (+15 deg above horizontal).
 *         Counter-clockwise servo travel lowers launch angle to -15 deg (30 deg hood travel).
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
    public static final int ROTATION_SERVO_1_PORT = 1;
    public static final int ROTATION_SERVO_2_PORT = 2;
    public static final int PITCH_SERVO_PORT = 0;
    public static final String HW_TURRET_MOTOR     = "turret_motor";   // Exp Hub Motor & Quad Port 0
    public static final String HW_TURRET_ENCODER   = "turret_encoder"; // Exp Hub Motor/Quad Port 1
    public static final String HW_LIMELIGHT        = "limelight";      // Control Hub USB 3.0

    // =========================================================================
    // AXON MAX+ SERVO PULSE WIDTH & RANGE CONSTANTS
    // =========================================================================
    public static final int AXON_MIN_PULSE_US = AxonMaxPlus.MIN_PULSE_US;
    public static final int AXON_MAX_PULSE_US = AxonMaxPlus.MAX_PULSE_US;
    public static final double AXON_MAX_TRAVEL_DEG = AxonMaxPlus.PROGRAMMED_TRAVEL_DEG; // Programmed travel across 0.0..1.0

    // =========================================================================
    // TURRET ROTATION (YAW) & DEADZONE CONSTANTS (1:1 GEARING)
    // =========================================================================
    // Robot headings are clockwise: front=0, right=90, back=180, left=270 deg.
    // Center (0.5) is home at robot heading 90 deg, internal servo-relative yaw 0 deg.
    // Internal yaw stays counterclockwise for field geometry; it is NOT a robot heading.
    // Deadzone of +/-10 deg at the left (robot headings 260..280, internal yaw +/-180) prevents wire wrap
    // and keeps the servos safely away from the 0.0 / 1.0 PWM discontinuity.
    public static final double ROTATION_GEAR_RATIO = 1.0; // 1:1 servo-to-turret rotation
    public static final double ROTATION_HOME_SERVO_POS = 0.5;
    public static final double ROTATION_HOME_ROBOT_HEADING_DEG = 90.0;
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
    public static final double MAX_ENCODER_TRIM_DEG = 25.0;         // Max servo command correction (deg)
    public static final double ENCODER_TRIM_DEADBAND_DEG = 0.35;    // Ignore tiny sub-degree encoder noise
    public static final double SERVO_CUTOUT_ERROR_THRESHOLD_DEG = 4.0; // Flag warning if |target - enc| > 4 deg

    // =========================================================================
    // HOOD TRAVEL FROM HOME (PORT 0, 2:1 LINKAGE): 0..30 DEG => LAUNCH +15..-15 DEG
    // =========================================================================
    // Home position 0.5 = zero hood travel = +15 deg launch angle above horizontal.
    // Linked 2:1 with turret angle: 1 deg of turret pitch requires 2 deg of Axon MAX+ servo rotation
    // (or set PITCH_SERVO_TO_TURRET_RATIO = 1.0 if measuring servo shaft degrees directly).
    // Looking at the front gear face of the Axon MAX+ servo, decreasing PWM from 0.5 (1500 us -> 500 us)
    // rotates the servo horn counter-clockwise (CCW). Flip PITCH_SERVO_DIRECTION_SIGN to +1.0 if your
    // Axon programmer has direction inversion enabled.
    public static final double PITCH_MIN_DEG = 0.0;
    public static final double PITCH_MAX_DEG = 30.0;
    public static final double LAUNCH_HOME_ANGLE_DEG = 15.0;
    public static final double PITCH_HOME_DEG = 0.0;
    public static final double PITCH_SERVO_TO_TURRET_RATIO = 2.0; // 60 deg servo CCW = 30 deg hood travel
    public static final double PITCH_HOME_SERVO_POS = 0.5;        // Highest launch angle
    public static final double PITCH_SERVO_POS_AT_0_DEG = PITCH_HOME_SERVO_POS;
    public static final double PITCH_SERVO_DEG_PER_UNIT = AXON_MAX_TRAVEL_DEG; // 355.0 deg across 0.0..1.0
    public static final double PITCH_SERVO_DIRECTION_SIGN = -1.0; // -1.0 rotates CCW from 0.5 when looking at front gear face
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
    public static final double SHOT_READY_YAW_TOLERANCE_DEG = 1.0;
    public static final double SHOT_READY_PITCH_TOLERANCE_DEG = 0.25;
    public static final double HOOD_SETTLE_TIME_SECONDS = 0.3;

    // =========================================================================
    // LIMELIGHT 3A MOUNTING & BIOBUZZ HIVE / APRILTAG FIELD CONSTANTS
    // =========================================================================
    public static final double LIMELIGHT_MOUNT_ANGLE_DEG = 30.0;     // Angled 30 deg above horizontal toward goal
    public static final double LIMELIGHT_LENS_HEIGHT_MM = 335.0;
    public static final double LIMELIGHT_LENS_HEIGHT_INCHES = LIMELIGHT_LENS_HEIGHT_MM / 25.4;
    public static final int LIMELIGHT_DEFAULT_PIPELINE = 0;

    // Robot-frame turret pivot offset; the rotating camera offset is separate in ShootingConfiguration.
    public static final double TURRET_OFFSET_FORWARD_IN = ShootingConfiguration.TURRET_PIVOT_FORWARD_MM / 25.4;
    public static final double TURRET_OFFSET_LEFT_IN = ShootingConfiguration.TURRET_PIVOT_LEFT_MM / 25.4;

    // Math uses CCW radians: right-facing home is -90 deg (clockwise robot heading 90 deg).
    public static final double TURRET_ZERO_ROBOT_HEADING_OFFSET_DEG = -90.0;

    // Legacy example geometry. Must be independently checked before calibration approval.
    // Team planning field frame (must match route and tag survey): (0, 0) at field center,
    // +X toward Blue alliance (right), -X toward Red alliance (left),
    // +Y toward Far / Scoring Table side, -Y toward Audience side.
    public static final double FIELD_SIZE_INCHES = 144.0;
    public static final double HIVE_CENTER_TO_CENTER_INCHES = 25.5;
    public static final double RED_HIVE_PIVOT_X_IN = -HIVE_CENTER_TO_CENTER_INCHES / 2.0;  // -12.75 in
    public static final double BLUE_HIVE_PIVOT_X_IN = +HIVE_CENTER_TO_CENTER_INCHES / 2.0; // +12.75 in
    public static final double HIVE_PIVOT_Y_IN = 0.0;
    public static final double HIVE_PIVOT_Z_IN = 43.95;
    public static final double HIVE_TILT_DEG = 30.0;
    public static final double HIVE_CELL_GAP_IN = 18.84;
    public static final double HIVE_CELL_DEPTH_IN = 12.04;
    public static final double HIVE_OPENING_WIDTH_IN = 20.0;
    public static final double HIVE_OPENING_SHOULDER_HEIGHT_IN = 7.61;
    public static final double HIVE_OPENING_PEAK_HEIGHT_IN = 14.0;
    public static final double HIVE_MOUTH_BOTTOM_Z_IN = 53.5;

    // Raised Hive Cell opening center offset from Hive pivot:
    // Along cell axis a2 = 18.84/2 + 12.04 = 21.46 in at 30 deg tilt:
    // Mouth bottom is at (y = +/- 14.23 in, z = 53.50 in);
    // Opening center (~7.0 in up the 30 deg mouth plane) is at (y = +/- 17.73 in, z = 59.56 in).
    public static final double HIVE_RAISED_MOUTH_CENTER_Y_OFFSET_IN = 17.73;
    public static final double HIVE_RAISED_MOUTH_CENTER_Z_IN = 59.56;
    public static final double GOAL_TARGET_HEIGHT_INCHES = HIVE_RAISED_MOUTH_CENTER_Z_IN;

    // BIOBUZZ AprilTag IDs on each Hive Cell (4-tag clusters on the underside of each cell):
    // Red Far (Scoring Side): 30..33 | Red Audience: 34..37
    // Blue Audience: 38..41          | Blue Far (Scoring Side): 42..45
    public static final int RED_FAR_TAG_MIN = 30;
    public static final int RED_FAR_TAG_MAX = 33;
    public static final int RED_AUDIENCE_TAG_MIN = 34;
    public static final int RED_AUDIENCE_TAG_MAX = 37;
    public static final int BLUE_AUDIENCE_TAG_MIN = 38;
    public static final int BLUE_AUDIENCE_TAG_MAX = 41;
    public static final int BLUE_FAR_TAG_MIN = 42;
    public static final int BLUE_FAR_TAG_MAX = 45;

    public enum Alliance {
        RED,
        BLUE
    }

    public enum HiveSide {
        FAR,      // Scoring table side (+Y): Tags 30-33 (Red) or 42-45 (Blue)
        AUDIENCE  // Audience side (-Y): Tags 34-37 (Red) or 38-41 (Blue)
    }

    public static final long LIMELIGHT_MAX_STALENESS_MS = 150;
    public static final double HOME_TRAVEL_SECONDS = 1.5, HOME_STABLE_SECONDS = .35;
    public static final double HOME_TIMEOUT_SECONDS = 5, HOME_STABLE_WINDOW_DEG = .3;

    // =========================================================================
    // HARDWARE DEVICES
    // =========================================================================
    private final Servo rotationServo1;
    private final Servo rotationServo2;
    private final Servo pitchServo;
    private final DcMotorEx shooterMotor;
    private final DcMotorEx throughBoreEncoderPort;
    private final Limelight3A limelight;
    private final boolean visionEnabled;

    private final TurretHomingController homing = new TurretHomingController(HOME_TRAVEL_SECONDS, HOME_STABLE_SECONDS, HOME_TIMEOUT_SECONDS, HOME_STABLE_WINDOW_DEG);
    private final TurretControl control = new TurretControl();
    private int rawTicks, encoderZeroOffsetTicks;
    private double encoderVelocity, currentRpm, targetYawDeg, targetPitchDeg, targetShooterRpm;
    private double encoderTrimDeg, lastCommandedRotationServoPos = .5, lastCommandedPitchServoPos = .5;
    private boolean encoderCorrectionEnabled = true;
    private Alliance activeAlliance = Alliance.RED;
    private HiveSide activeHiveSide = HiveSide.FAR;
    private LLResult cachedResult;
    private LLResultTypes.FiducialResult cachedTag;
    private String trackingStatus = "Automatic aiming requires ShotCoordinator";

    public TurretSubsystem(HardwareMap hardwareMap) {
        this(hardwareMap, true);
    }

    /** Manual mechanism diagnostics can omit the camera entirely. */
    public TurretSubsystem(HardwareMap hardwareMap, boolean enableVision) {
        visionEnabled = enableVision;
        rotationServo1 = getTurretServo(hardwareMap, ROTATION_SERVO_1_PORT,
                HW_ROTATION_SERVO_1, "turretServo1", "turret_left", "turret1");
        rotationServo2 = getTurretServo(hardwareMap, ROTATION_SERVO_2_PORT,
                HW_ROTATION_SERVO_2, "turretServo2", "turret_right", "turret2");
        pitchServo = getTurretServo(hardwareMap, PITCH_SERVO_PORT,
                HW_PITCH_SERVO, "turretPitch", "hood_servo", "pitch_servo");
        shooterMotor = getDevice(hardwareMap, DcMotorEx.class,
                HW_TURRET_MOTOR, "turretMotor", "shooter_motor", "shooter", "flywheel");
        throughBoreEncoderPort = getDevice(hardwareMap, DcMotorEx.class,
                HW_TURRET_ENCODER, "turretEncoder", "through_bore", "turret_enc",
                IntakeSubsystem.HW_INTAKE_MOTOR);
        limelight = enableVision ? tryGetDevice(hardwareMap, Limelight3A.class,
                HW_LIMELIGHT, "limelight3a") : null;

        configureServos();
        configureShooterMotor();
        // Intake owns the shared encoder channel mode/direction; do not configure/reset it here.
        configureLimelight();

        // Begin travel now; update during INIT and after an early START.
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

    private void configureLimelight() {
        if (limelight != null) {
            limelight.pipelineSwitch(LIMELIGHT_DEFAULT_PIPELINE);
            limelight.start();
        }
    }

    /** Begins non-blocking estimated homing; zero is captured only after travel and stability. */
    public void homeTurretAndZeroEncoder() {
        targetYawDeg = 0; control.reset();
        homing.start(System.nanoTime() * 1e-9);
        setRawRotationServoPosition(ROTATION_HOME_SERVO_POS);
    }
    /** Diagnostic relative zero deliberately invalidates automatic firing until rehomed. */
    public void zeroEncoderAtCurrentPosition() {
        encoderZeroOffsetTicks = rawTicks; control.reset(); homing.invalidate();
    }
    public void update() {
        double now = System.nanoTime() * 1e-9;
        rawTicks = throughBoreEncoderPort.getCurrentPosition();
        encoderVelocity = throughBoreEncoderPort.getVelocity() / TURRET_TICKS_PER_DEGREE * ENCODER_DIRECTION_SIGN;
        currentRpm = shooterMotor.getVelocity() * 60 / SHOOTER_MOTOR_TICKS_PER_REV;
        boolean wasReady = homing.ready();
        homing.update(now, rawTicks, rawTicks / TURRET_TICKS_PER_DEGREE * ENCODER_DIRECTION_SIGN);
        if (!wasReady && homing.ready()) encoderZeroOffsetTicks = homing.zeroTicks();
        encoderTrimDeg = control.update(now, targetYawDeg - getEncoderAngleDegrees(), encoderVelocity,
                homing.ready() && encoderCorrectionEnabled, targetPitchDeg,
                ROTATION_MIN_ANGLE_DEG - targetYawDeg, ROTATION_MAX_ANGLE_DEG - targetYawDeg);
        if (homing.ready()) {
            setRawRotationServoPosition(angleDegreesToRotationServoPos(clampYawAngle(targetYawDeg + encoderTrimDeg)));
        } else {
            setRawRotationServoPosition(ROTATION_HOME_SERVO_POS);
        }
        cachedResult = limelight == null ? null : limelight.getLatestResult();
        cachedTag = null;
        if (cachedResult != null && cachedResult.isValid() && cachedResult.getStaleness() >= 0
                && cachedResult.getStaleness() <= LIMELIGHT_MAX_STALENESS_MS) {
            for (LLResultTypes.FiducialResult tag : cachedResult.getFiducialResults()) {
                if (tag != null && getHiveSideForTag(activeAlliance, tag.getFiducialId()) == activeHiveSide
                        && Double.isFinite(tag.getTargetXDegrees()) && Double.isFinite(tag.getTargetYDegrees())
                        && (cachedTag == null || tag.getFiducialId() < cachedTag.getFiducialId())) cachedTag = tag;
            }
        }
    }
    public boolean isHomed() { return homing.ready(); }
    public String getHomingStatus() { return homing.state() + ": " + homing.reason(); }
    public LLResult getVisionResult() { return cachedResult; }
    public LLResultTypes.FiducialResult getVisionTag() { return cachedTag; }
    public void setTrackingStatus(String status) { trackingStatus = status; }
    public void resetTrackingTrim() { control.reset(); }
    public void setTargetYawDegrees(double deg) { if (Double.isFinite(deg)) targetYawDeg = clampYawAngle(deg); }
    public void adjustTargetYawDegrees(double delta) { setTargetYawDegrees(targetYawDeg + delta); }
    /** GP1 auto-aim owns yaw while held; releasing manual/automatic control captures actual position. */
    public void updateManualYaw(double stickX, double dt, boolean automatic) {
        if (!isHomed()) return;
        setTargetYawDegrees(control.manualYawTarget(getEncoderAngleDegrees(), targetYawDeg, stickX, dt, automatic));
    }
    public double getRobotHeadingDegrees() { return TurretControl.yawToRobotHeadingDegrees(getEncoderAngleDegrees()); }
    public double getTargetRobotHeadingDegrees() { return TurretControl.yawToRobotHeadingDegrees(targetYawDeg); }
    public double getTargetYawDegrees() { return targetYawDeg; }
    public int getEncoderTicks() { return rawTicks - encoderZeroOffsetTicks; }
    public double getEncoderAngleDegrees() { return getEncoderTicks() / TURRET_TICKS_PER_DEGREE * ENCODER_DIRECTION_SIGN; }
    public double getEncoderVelocityDegPerSec() { return encoderVelocity; }
    public boolean isRotationErrorHigh() { return Math.abs(targetYawDeg - getEncoderAngleDegrees()) > 4; }
    public void setEncoderCorrectionEnabled(boolean enabled) {
        if (encoderCorrectionEnabled != enabled) control.reset();
        encoderCorrectionEnabled = enabled;
    }
    public boolean isEncoderCorrectionEnabled() { return encoderCorrectionEnabled; }
    public void setRawRotationServoPosition(double pos) {
        lastCommandedRotationServoPos = Range.clip(pos, 0, 1);
        rotationServo1.setPosition(lastCommandedRotationServoPos); rotationServo2.setPosition(lastCommandedRotationServoPos);
    }
    public double getLastCommandedRotationServoPos() { return lastCommandedRotationServoPos; }
    public static double clampYawAngle(double deg) { return Range.clip(deg, ROTATION_MIN_ANGLE_DEG, ROTATION_MAX_ANGLE_DEG); }
    public static double angleDegreesToRotationServoPos(double deg) {
        return Range.clip(.5 + deg * ROTATION_GEAR_RATIO * ROTATION_SERVO_DIRECTION_SIGN / AXON_MAX_TRAVEL_DEG, 0, 1);
    }
    /** Commands hood travel down from home (0..30 deg), not elevation above horizontal. */
    public void setPitchAngleDegrees(double deg) {
        if (!Double.isFinite(deg)) return;
        targetPitchDeg = Range.clip(deg, PITCH_MIN_DEG, PITCH_MAX_DEG);
        setRawPitchServoPosition(pitchDegreesToServoPos(targetPitchDeg));
    }
    public void adjustPitchAngleDegrees(double delta) { setPitchAngleDegrees(targetPitchDeg + delta); }
    public double getTargetPitchDegrees() { return targetPitchDeg; }
    /** Commanded geometric exit elevation; this is not a measured projectile trajectory. */
    public static double hoodTravelToLaunchAngleDegrees(double hoodTravelDeg) {
        return LAUNCH_HOME_ANGLE_DEG - Range.clip(hoodTravelDeg, PITCH_MIN_DEG, PITCH_MAX_DEG);
    }
    public void setRawPitchServoPosition(double pos) {
        lastCommandedPitchServoPos = Range.clip(pos, 0, 1); pitchServo.setPosition(lastCommandedPitchServoPos);
    }
    public double getLastCommandedPitchServoPos() { return lastCommandedPitchServoPos; }
    public static double pitchDegreesToServoPos(double deg) {
        return Range.clip(PITCH_HOME_SERVO_POS + PITCH_SERVO_DIRECTION_SIGN * Range.clip(deg, PITCH_MIN_DEG, PITCH_MAX_DEG)
                * PITCH_SERVO_TO_TURRET_RATIO / PITCH_SERVO_DEG_PER_UNIT, 0, 1);
    }
    public boolean isHoodTracking() { return control.hoodError(targetPitchDeg) <= SHOT_READY_PITCH_TOLERANCE_DEG; }
    public void setShooterRpm(double rpm) {
        if (!Double.isFinite(rpm)) { stopShooter(); return; }
        targetShooterRpm = Range.clip(rpm, -SHOOTER_MOTOR_MAX_RPM, SHOOTER_MOTOR_MAX_RPM);
        shooterMotor.setVelocity(targetShooterRpm / 60 * SHOOTER_MOTOR_TICKS_PER_REV);
    }
    public void setShooterPower(double power) {
        targetShooterRpm = Range.clip(power, -1, 1) * SHOOTER_MOTOR_MAX_RPM; shooterMotor.setPower(Range.clip(power, -1, 1));
    }
    public void stopShooter() { targetShooterRpm = 0; shooterMotor.setPower(0); }
    public double getShooterCurrentRpm() { return currentRpm; }
    public double getTargetShooterRpm() { return targetShooterRpm; }
    public boolean isShooterAtSpeed() { return targetShooterRpm > 0 && Math.abs(currentRpm - targetShooterRpm) <= SHOOTER_READY_TOLERANCE_RPM; }
    public void setTargetHiveCell(Alliance alliance, HiveSide side) {
        activeAlliance = alliance == null ? Alliance.RED : alliance; activeHiveSide = side == null ? HiveSide.FAR : side;
    }
    public Alliance toggleAlliance() { setTargetHiveCell(activeAlliance == Alliance.RED ? Alliance.BLUE : Alliance.RED, activeHiveSide); return activeAlliance; }
    public HiveSide toggleHiveSide() { setTargetHiveCell(activeAlliance, activeHiveSide == HiveSide.FAR ? HiveSide.AUDIENCE : HiveSide.FAR); return activeHiveSide; }
    public Alliance getActiveAlliance() { return activeAlliance; }
    public HiveSide getActiveHiveSide() { return activeHiveSide; }
    public static HiveSide getHiveSideForTag(Alliance alliance, int id) {
        if (alliance == Alliance.RED) {
            if (id >= 30 && id <= 33) return HiveSide.FAR;
            if (id >= 34 && id <= 37) return HiveSide.AUDIENCE;
        } else if (alliance == Alliance.BLUE) {
            if (id >= 42 && id <= 45) return HiveSide.FAR;
            if (id >= 38 && id <= 41) return HiveSide.AUDIENCE;
        }
        return null;
    }
    static LLResultTypes.FiducialResult selectAllianceFiducial(Alliance alliance, List<LLResultTypes.FiducialResult> tags) {
        if (tags == null) return null;
        for (LLResultTypes.FiducialResult tag : tags) if (tag != null && getHiveSideForTag(alliance, tag.getFiducialId()) != null) return tag;
        return null;
    }
    public static double getHiveOpeningCenterXInches(Alliance alliance) { return alliance == Alliance.BLUE ? BLUE_HIVE_PIVOT_X_IN : RED_HIVE_PIVOT_X_IN; }
    public static double getHiveOpeningCenterYInches(HiveSide side) { return side == HiveSide.AUDIENCE ? -17.73 : 17.73; }
    public static double normalizeAngleDegrees(double deg) { return Math.toDegrees(RobotMath.wrap(Math.toRadians(deg))); }
    public void setLimelightPipeline(int pipeline) { if (limelight != null) limelight.pipelineSwitch(pipeline); }
    public void setAllServosToZeroPosition() {
        homing.invalidate(); control.reset(); encoderCorrectionEnabled = false;
        targetPitchDeg = 0; setRawRotationServoPosition(.5); setRawPitchServoPosition(.5);
    }
    public void stop() { stopShooter(); if (limelight != null) limelight.stop(); }
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Turret home", getHomingStatus());
        if (visionEnabled) {
            telemetry.addData("Limelight", limelight == null ? "NOT CONFIGURED" : "ENABLED");
            telemetry.addData("Selected goal", "%s / %s | tag %d", activeAlliance, activeHiveSide, cachedTag == null ? -1 : cachedTag.getFiducialId());
        }
        telemetry.addData("Turret robot heading (deg CW)", "%.2f / %.2f | rate %.1f deg/s", getRobotHeadingDegrees(), getTargetRobotHeadingDegrees(), -encoderVelocity);
        telemetry.addData("Turret encoder PID correction (deg CCW)", encoderTrimDeg);
        telemetry.addData("Hood command (deg)", "%.2f | estimated tracking %s (no feedback)", targetPitchDeg, isHoodTracking());
        telemetry.addData("Launch elevation command (deg above horizontal)", hoodTravelToLaunchAngleDegrees(targetPitchDeg));
        telemetry.addData("Flywheel (RPM)", "%.0f / %.0f", currentRpm, targetShooterRpm);
        if (visionEnabled) telemetry.addData("Shot tracking", trackingStatus);
    }

    private Servo getTurretServo(HardwareMap hwMap, int expectedPort, String... candidateNames) {
        Servo namedServo = tryGetDevice(hwMap, Servo.class, candidateNames);
        if (namedServo != null) {
            return namedServo;
        }

        Servo portMatch = null;
        for (Servo candidate : hwMap.getAll(Servo.class)) {
            if (candidate.getPortNumber() == expectedPort) {
                if (portMatch != null) {
                    throw new IllegalStateException("More than one Servo is configured on port "
                            + expectedPort + "; name the turret servo " + candidateNames[0]
                            + " so HardwareMap selection is unambiguous.");
                }
                portMatch = candidate;
            }
        }
        if (portMatch != null) {
            return portMatch;
        }

        StringBuilder configured = new StringBuilder();
        for (Servo candidate : hwMap.getAll(Servo.class)) {
            if (configured.length() > 0) {
                configured.append(", ");
            }
            configured.append(hwMap.getNamesOf(candidate))
                    .append(" [").append(candidate.getController().getConnectionInfo())
                    .append(", port ").append(candidate.getPortNumber()).append("]");
        }
        throw new IllegalArgumentException("No configured Servo named " + candidateNames[0]
                + " or on expected port " + expectedPort
                + ". Configured servos: " + (configured.length() == 0 ? "none" : configured));
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
