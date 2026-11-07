package org.firstinspires.ftc.teamcode;

/** Nonblocking encoder-based recovery, shared by isolated, combined and autonomous intake users. */
public final class IntakeJamRecovery {
    public static final double MIN_VELOCITY_TICKS_PER_SECOND = 20.0;
    public static final double DETECT_SECONDS = .20;
    public static final double CLEAR_SECONDS = .50;
    public static final double DIRECTION_SECONDS = .125;
    private double previousRequest;
    private double stalledSince = Double.NaN, clearingSince = Double.NaN;

    public double update(double now, double requestedPower, double velocityTicksPerSecond) {
        if (requestedPower == 0 || requestedPower != previousRequest) reset();
        previousRequest = requestedPower;
        if (requestedPower == 0) return 0;
        if (isClearing()) {
            double elapsed = now - clearingSince;
            if (elapsed < CLEAR_SECONDS) {
                int phase = (int) (elapsed / DIRECTION_SECONDS);
                return phase % 2 == 0 ? -requestedPower : requestedPower;
            }
            clearingSince = stalledSince = Double.NaN;
        }
        // Missing/nonfinite feedback cannot establish a stall.
        if (!Double.isFinite(velocityTicksPerSecond)
                || Math.abs(velocityTicksPerSecond) >= MIN_VELOCITY_TICKS_PER_SECOND) {
            stalledSince = Double.NaN;
        } else {
            if (Double.isNaN(stalledSince)) stalledSince = now;
            if (now - stalledSince >= DETECT_SECONDS) {
                clearingSince = now;
                return -requestedPower;
            }
        }
        return requestedPower;
    }
    public boolean isClearing() { return !Double.isNaN(clearingSince); }
    public void reset() { previousRequest = 0; stalledSince = clearingSince = Double.NaN; }
}
