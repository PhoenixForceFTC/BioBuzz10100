package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Measures how far the turret still trails a moving auto-aim bearing. Telemetry only: it never changes the
 * aim. Spin the chassis at a steady rate with auto aim held; the residual lag is the least-squares fit of
 * (bearing now - measured yaw) against measured turret rate, so a correct AIM_LEAD_SECONDS reads about 0 ms.
 * Holds, the rear dead zone and the long-way unwind are excluded by the rate and error limits.
 */
public final class TurretLagEstimator {
    public static final double MIN_TURRET_RATE_DEG_PER_SECOND = 60, MAX_TRACKING_ERROR_DEG = 20;
    public static final double MEMORY_SECONDS = 1.0;
    private double sumErrorRate, sumRateSquared, lastTime = Double.NaN, lastErrorDeg = Double.NaN, peakChassisRate;
    private int samples;

    public void reset() {
        sumErrorRate = sumRateSquared = peakChassisRate = 0; samples = 0;
        lastTime = lastErrorDeg = Double.NaN;
    }

    /** Degrees use the turret's internal (CCW) yaw; rates are deg/s. */
    public void update(double now, double requiredYawDeg, double measuredYawDeg,
                       double turretRateDegPerSecond, double chassisRateDegPerSecond) {
        if (Double.isFinite(chassisRateDegPerSecond)) peakChassisRate = Math.max(peakChassisRate, Math.abs(chassisRateDegPerSecond));
        if (!RobotMath.finite(now, requiredYawDeg, measuredYawDeg, turretRateDegPerSecond)) { lastErrorDeg = Double.NaN; return; }
        double dt = Double.isNaN(lastTime) ? 0 : Math.max(0, Math.min(.1, now - lastTime));
        lastTime = now;
        double error = TurretSubsystem.normalizeAngleDegrees(requiredYawDeg - measuredYawDeg);
        lastErrorDeg = error;
        if (Math.abs(turretRateDegPerSecond) < MIN_TURRET_RATE_DEG_PER_SECOND || Math.abs(error) > MAX_TRACKING_ERROR_DEG) return;
        double keep = Math.exp(-dt / MEMORY_SECONDS);
        sumErrorRate = keep * sumErrorRate + error * turretRateDegPerSecond;
        sumRateSquared = keep * sumRateSquared + turretRateDegPerSecond * turretRateDegPerSecond;
        samples++;
    }
    /** Positive: the turret still lags (raise the lead); negative: it leads too much. NaN before any sample. */
    public double residualLagSeconds() { return samples == 0 || !(sumRateSquared > 0) ? Double.NaN : sumErrorRate / sumRateSquared; }
    public double suggestedLeadSeconds(double currentLeadSeconds) {
        double residual = residualLagSeconds();
        return Double.isFinite(residual) ? Math.max(0, Math.min(TurretControl.MAX_AIM_LEAD_SECONDS, currentLeadSeconds + residual)) : Double.NaN;
    }
    public double lastErrorDeg() { return lastErrorDeg; }
    public double peakChassisRateDegPerSecond() { return peakChassisRate; }
    public int samples() { return samples; }

    public void addTelemetry(Telemetry telemetry, double currentLeadSeconds) {
        telemetry.addData("Turret tracking error now (deg)", "%.2f | shot tolerance %.1f", lastErrorDeg, ShotReadiness.YAW_TOLERANCE_DEG);
        telemetry.addData("Turret lag (ms): residual / aim lead / suggested lead", "%.0f / %.0f / %.0f | %d moving samples",
                1000 * residualLagSeconds(), 1000 * currentLeadSeconds, 1000 * suggestedLeadSeconds(currentLeadSeconds), samples);
        telemetry.addData("Peak chassis turn rate while aiming (deg/s)", "%.0f", peakChassisRate);
    }
}
