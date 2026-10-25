package org.firstinspires.ftc.teamcode;

/** Direct positional servo control with external encoder backup, plus manual control and hood estimate. */
public final class TurretControl {
    // Initial gains require tuning on the loaded turret. D uses measured velocity to avoid setpoint kick.
    public static final double YAW_KP = .25, YAW_KI_PER_SECOND = .5, YAW_KD_SECONDS = .02;
    public static final double MAX_TRIM_DEG = 25, TRIM_DEADBAND_DEG = .35;
    // Let the servo's internal position loop handle travel; apply backup feedback only at low speed.
    public static final double TRIM_SETTLED_VELOCITY_DEG_PER_SECOND = 3;
    public static final double BACKUP_ACTIVATION_ERROR_DEG = 1;
    public static final double BACKUP_ERROR_PERSISTENCE_SECONDS = .25;
    // Axon MAX+ at 6 V: https://docs.axon-robotics.com/archive/max (rated 0.115 s / 60 deg).
    public static final double AXON_MAX_PLUS_SECONDS_PER_60_DEG_AT_6V = AxonMaxPlus.SECONDS_PER_60_DEG_AT_6V;
    // This limits only the position estimate, not the servo command. The hood receives its target directly.
    // Rated travel estimate with the 2:1 linkage; loaded hood speed requires measurement.
    public static final double HOOD_TRAVEL_DEG_PER_SECOND = AxonMaxPlus.DEGREES_PER_SECOND_AT_6V
            / TurretSubsystem.PITCH_SERVO_TO_TURRET_RATIO;
    public static final double MAX_YAW_DEG_PER_SECOND = AxonMaxPlus.DEGREES_PER_SECOND_AT_6V
            / TurretSubsystem.ROTATION_GEAR_RATIO;
    public static final double MANUAL_STICK_DEADBAND = .08;
    public static final double MAX_UPDATE_SECONDS = .1;
    private double integral, yawTrimDeg, estimatedHood, lastTime = Double.NaN;
    private double lastYawTargetDeg = Double.NaN, backupErrorSince = Double.NaN, backupErrorSign;
    private boolean backupActive;
    private boolean previousManual, previousAutomatic;
    public void reset() {
        integral = 0; yawTrimDeg = 0; backupActive = false;
        lastYawTargetDeg = Double.NaN; backupErrorSince = Double.NaN;
    }
    /** The servos handle normal travel/hold. Only persistent settled errors engage external feedback. */
    public double updateYawCommand(double now, double targetYawDeg, double measuredYawDeg,
                                   double measuredVelocityDegPerSecond, boolean trimEnabled, double hoodCommandDeg) {
        double target = TurretSubsystem.clampYawAngle(targetYawDeg);
        double error = target - measuredYawDeg;
        boolean slow = Math.abs(measuredVelocityDegPerSecond) <= TRIM_SETTLED_VELOCITY_DEG_PER_SECOND;
        if (target != lastYawTargetDeg || !trimEnabled || (slow && Math.abs(error) <= TRIM_DEADBAND_DEG)) {
            backupActive = false; backupErrorSince = Double.NaN;
        }
        lastYawTargetDeg = target;
        if (!backupActive && trimEnabled) {
            if (!slow || Math.abs(error) <= BACKUP_ACTIVATION_ERROR_DEG) backupErrorSince = Double.NaN;
            else if (Double.isNaN(backupErrorSince) || Math.signum(error) != backupErrorSign) {
                backupErrorSince = now; backupErrorSign = Math.signum(error);
            } else if (now - backupErrorSince >= BACKUP_ERROR_PERSISTENCE_SECONDS) backupActive = true;
        }
        double low = TurretSubsystem.ROTATION_MIN_ANGLE_DEG - target;
        double high = TurretSubsystem.ROTATION_MAX_ANGLE_DEG - target;
        double heldTrim = trimEnabled ? Math.max(low, Math.min(high, yawTrimDeg)) : 0;
        double heldIntegral = trimEnabled ? Math.max(low, Math.min(high, integral)) : 0;
        boolean correcting = backupActive && slow;
        update(now, error, measuredVelocityDegPerSecond, correcting, hoodCommandDeg, low, high);
        if (!correcting) {
            // Retain learned position offset, without live PID changes while moving or holding normally.
            yawTrimDeg = heldTrim;
            integral = slow && Math.abs(error) <= TRIM_DEADBAND_DEG ? heldTrim : heldIntegral;
        }
        return TurretSubsystem.clampYawAngle(target + yawTrimDeg);
    }
    public boolean isBackupCorrectionActive() { return backupActive; }
    public double getYawTrimDegrees() { return yawTrimDeg; }
    public double update(double now, double yawErrorDeg, boolean trimEnabled, double hoodCommandDeg) {
        return update(now, yawErrorDeg, 0, trimEnabled, hoodCommandDeg, -MAX_TRIM_DEG, MAX_TRIM_DEG);
    }
    public double update(double now, double yawErrorDeg, double measuredVelocityDegPerSecond,
                         boolean trimEnabled, double hoodCommandDeg, double minimumTrimDeg, double maximumTrimDeg) {
        double dt = Double.isNaN(lastTime) ? 0 : Math.max(0, Math.min(MAX_UPDATE_SECONDS, now - lastTime));
        lastTime = now;
        double trim = 0;
        if (trimEnabled) {
            double low = Math.max(-MAX_TRIM_DEG, minimumTrimDeg), high = Math.min(MAX_TRIM_DEG, maximumTrimDeg);
            boolean slow = Math.abs(measuredVelocityDegPerSecond) <= TRIM_SETTLED_VELOCITY_DEG_PER_SECOND;
            if (Math.abs(yawErrorDeg) <= TRIM_DEADBAND_DEG && slow) {
                // Preserve the successful command instead of dropping P and kicking the turret back out.
                trim = Math.max(low, Math.min(high, yawTrimDeg));
                integral = trim;
            } else {
                // Continuous deadband avoids a proportional step at its edge.
                double error = Math.copySign(Math.max(0, Math.abs(yawErrorDeg) - TRIM_DEADBAND_DEG), yawErrorDeg);
                double pd = YAW_KP * error - YAW_KD_SECONDS * measuredVelocityDegPerSecond;
                double candidate = integral;
                if (slow) {
                    // Use the full error here so residual bias enters the hold band in finite time.
                    candidate = RobotMath.clip(integral + yawErrorDeg * YAW_KI_PER_SECOND * dt, MAX_TRIM_DEG);
                }
                // Do not accumulate error into either correction limit or the mechanical travel limit.
                double output = pd + candidate;
                if (!((output > high && error > 0) || (output < low && error < 0))) integral = candidate;
                trim = Math.max(low, Math.min(high, pd + integral));
            }
        } else integral = 0;
        yawTrimDeg = trim;
        estimatedHood += RobotMath.clip(hoodCommandDeg - estimatedHood, HOOD_TRAVEL_DEG_PER_SECOND * dt);
        return trim;
    }
    public double hoodError(double commandDeg) { return Math.abs(commandDeg - estimatedHood); }
    public static double manualDelta(double input, double dt) { return input * MAX_YAW_DEG_PER_SECOND * Math.max(0, Math.min(dt, MAX_UPDATE_SECONDS)); }
    /** Positive stick is clockwise; internal yaw is counterclockwise from right-facing home. */
    public double manualYawTarget(double measuredYawDeg, double targetYawDeg, double stickX, double dt, boolean automatic) {
        boolean moving = !automatic && Math.abs(stickX) > MANUAL_STICK_DEADBAND;
        if (automatic) {
            previousAutomatic = true; previousManual = false;
            return targetYawDeg;
        }
        if (previousAutomatic || (previousManual && !moving)) {
            targetYawDeg = measuredYawDeg;
            reset();
        }
        previousAutomatic = false; previousManual = moving;
        return moving ? targetYawDeg - manualDelta(RobotMath.clip(stickX, 1), dt) : targetYawDeg;
    }
    public static double yawToRobotHeadingDegrees(double yawDeg) {
        return ((90 - yawDeg) % 360 + 360) % 360;
    }
    /** Signed, continuous servo yaw: front=+90, right=0, back=-90, left=+/-180. */
    public static double robotHeadingToYawDegrees(double headingDeg) {
        return Math.toDegrees(RobotMath.wrap(Math.toRadians(90 - headingDeg)));
    }
}
