package org.firstinspires.ftc.teamcode;

/** Hold-to-run bench controls with the competition timed feed/recovery cycle. */
public final class IntakeShooterTestController {
    private final FiringCoordinator firing = ShootingConfiguration.firing();
    private boolean armed, spinFlywheel, loadIndex, reverseIntake;

    public static final double INTAKE_JAM_MIN_VELOCITY_TICKS_PER_SECOND = 20.0;
    public static final double INTAKE_JAM_DETECT_SECONDS = 0.20;
    public static final double INTAKE_JAM_CLEAR_SECONDS = 0.50;
    public static final double INTAKE_JAM_DIRECTION_SECONDS = 0.125;
    private double previousIntakeRequest;
    private double intakeStalledSince = Double.NaN, intakeClearingSince = Double.NaN;

    public static final double BENCH_YAW_TOLERANCE_DEG = 4.0;
    /** Preserve the independently requested bench readiness tolerance. */
    public static boolean isYawSettled(double errorDeg, double velocityDegPerSecond) {
        return RobotMath.finite(errorDeg, velocityDegPerSecond)
                && Math.abs(errorDeg) <= BENCH_YAW_TOLERANCE_DEG
                && Math.abs(velocityDegPerSecond) <= TurretSubsystem.YAW_VERIFY_MAX_VELOCITY_DEG_PER_SECOND;
    }

    public void update(double now, boolean load, boolean reverse, boolean spin, boolean feed,
                       boolean stop, boolean homed, boolean ready) {
        if (stop) armed = false;
        else if (!load && !reverse && !spin && !feed) armed = true;

        spinFlywheel = armed && !stop && spin;
        boolean inhibit = !armed || stop || reverse;
        String rejection = !homed ? "Waiting for turret home" : "Waiting for flywheel";
        // Requalify readiness between timed feeds; retain the bench trigger/home interlocks.
        firing.update(now, armed && feed, false, inhibit, !spinFlywheel,
                homed, ready, ready, rejection);
        loadIndex = armed && !stop && load && !reverse && !feed;
        reverseIntake = armed && !stop && reverse && !load;
    }

    public boolean armed() { return armed; }
    public boolean spinFlywheel() { return spinFlywheel; }
    public boolean loadIndex() { return loadIndex; }
    public boolean feedTransfer(boolean gateReady) { return firing.feeding() && gateReady; }
    public double intakePower(boolean full) {
        return reverseIntake ? -1.0 : loadIndex ? (full ? -1.0 : 1.0) : 0.0;
    }

    /** Bench-only encoder jam recovery. Call once per loop after updating the controls. */
    public double updateIntakePower(double now, boolean full, double velocityTicksPerSecond) {
        double requestedPower = intakePower(full);
        if (requestedPower == 0 || requestedPower != previousIntakeRequest) {
            intakeStalledSince = intakeClearingSince = Double.NaN;
        }
        previousIntakeRequest = requestedPower;
        if (requestedPower == 0) return 0;

        if (isIntakeClearing()) {
            double elapsed = now - intakeClearingSince;
            if (elapsed < INTAKE_JAM_CLEAR_SECONDS) {
                int phase = (int) (elapsed / INTAKE_JAM_DIRECTION_SECONDS);
                return phase % 2 == 0 ? -requestedPower : requestedPower;
            }
            intakeClearingSince = Double.NaN;
            // Give the resumed direction a fresh stall window before retrying a persistent jam.
            intakeStalledSince = Double.NaN;
        }
        if (Math.abs(velocityTicksPerSecond) >= INTAKE_JAM_MIN_VELOCITY_TICKS_PER_SECOND) {
            intakeStalledSince = Double.NaN;
        } else {
            if (Double.isNaN(intakeStalledSince)) intakeStalledSince = now;
            if (now - intakeStalledSince >= INTAKE_JAM_DETECT_SECONDS) {
                intakeClearingSince = now;
                return -requestedPower;
            }
        }
        return requestedPower;
    }

    public boolean isIntakeClearing() { return !Double.isNaN(intakeClearingSince); }
    public FiringCoordinator firing() { return firing; }
}
