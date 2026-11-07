package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Competition TeleOp: while auto aim is held, caps the chassis turn command so the turret can keep pointing
 * at the goal. The stick-to-turn-rate gain is learned from Pinpoint heading rate, so battery sag and loaded
 * speed are followed without a second constant. Released, the stick passes through unchanged.
 */
public final class AutoAimSpinLimiter {
    /**
     * STARTING VALUE, UNMEASURED: about 57% of the turret's rated 522 deg/s no-load speed, leaving margin for
     * load, translation-driven bearing change and catch-up. Set below Test: Turret's measured peak rate
     * (full RS-X) and record that measurement here.
     */
    public static final double MAX_CHASSIS_RATE_DEG_PER_SECOND = 300;
    /**
     * Full-stick turn rate assumed before any Pinpoint learning. Deliberately above the ~420-460 deg/s no-load
     * estimate (2.6 m/s wheels, ~14-in lever), so an unlearned robot starts under the cap and learns down.
     */
    public static final double INITIAL_FULL_STICK_RATE_DEG_PER_SECOND = 600;
    public static final double LEARNING_FILTER_SECONDS = .15;
    public static final double MIN_LEARNING_STICK = .15, MIN_LEARNING_RATE_DEG_PER_SECOND = 30;
    /**
     * A higher gain is always learned (prevents exceeding the cap). A lower one only while the turn is steady
     * (small heading-rate change, unchanged command), so spin-up acceleration cannot loosen the cap.
     */
    public static final double STEADY_ACCELERATION_DEG_PER_SECOND_SQUARED = 300, STEADY_STICK_CHANGE = .02;
    public static final double MIN_STICK_LIMIT = .05;
    private double fullStickRate = INITIAL_FULL_STICK_RATE_DEG_PER_SECOND, lastStick, stickBefore, lastTime = Double.NaN;
    private double previousRateDeg = Double.NaN;
    private double stickLimit = 1;
    private boolean limiting;

    /**
     * @param yawStick clockwise-positive drive turn command
     * @param measuredRadPerSecond Pinpoint heading rate, CCW positive (NaN when unavailable)
     * @param aiming apply the cap this loop
     */
    public double limit(double now, double yawStick, double measuredRadPerSecond, boolean aiming) {
        double dt = Double.isNaN(lastTime) ? 0 : Math.max(0, Math.min(.1, now - lastTime));
        lastTime = now;
        // Clockwise stick turns the heading negative; learn only when the measured turn follows the command.
        double rateDeg = Double.isFinite(measuredRadPerSecond) ? Math.toDegrees(measuredRadPerSecond) : Double.NaN;
        if (Double.isFinite(rateDeg) && Math.abs(lastStick) >= MIN_LEARNING_STICK && -lastStick * rateDeg > 0
                && Math.abs(rateDeg) >= MIN_LEARNING_RATE_DEG_PER_SECOND) {
            double gain = Math.abs(rateDeg) / Math.abs(lastStick);
            boolean steady = dt > 0 && Double.isFinite(previousRateDeg)
                    && Math.abs(rateDeg - previousRateDeg) / dt <= STEADY_ACCELERATION_DEG_PER_SECOND_SQUARED
                    && Math.abs(lastStick - stickBefore) <= STEADY_STICK_CHANGE;
            if (gain > fullStickRate || steady)
                fullStickRate += (gain - fullStickRate) * -Math.expm1(-dt / LEARNING_FILTER_SECONDS);
        }
        previousRateDeg = rateDeg;
        stickBefore = lastStick;
        stickLimit = Math.max(MIN_STICK_LIMIT, Math.min(1, MAX_CHASSIS_RATE_DEG_PER_SECOND / fullStickRate));
        double out = Double.isFinite(yawStick) ? RobotMath.clip(yawStick, 1) : 0;
        limiting = aiming && Math.abs(out) > stickLimit;
        if (aiming) out = RobotMath.clip(out, stickLimit);
        lastStick = out;
        return out;
    }
    public boolean limiting() { return limiting; }
    public double stickLimit() { return stickLimit; }
    public double learnedFullStickRateDegPerSecond() { return fullStickRate; }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Aim turn cap (deg/s)", "%.0f | stick limit %.2f | learned full stick %.0f deg/s%s",
                MAX_CHASSIS_RATE_DEG_PER_SECOND, stickLimit, fullStickRate, limiting ? " | LIMITING" : "");
    }
}
