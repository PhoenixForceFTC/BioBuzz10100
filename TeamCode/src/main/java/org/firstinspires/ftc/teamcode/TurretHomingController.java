package org.firstinspires.ftc.teamcode;

/** Relative-encoder home estimate: travel, then a stable window. A stationary stall is undetectable. */
public final class TurretHomingController {
    public enum State { UNHOMED, MOVING_HOME, SETTLING, READY, FAULT }
    public final double travelSeconds, stableSeconds, timeoutSeconds, stableWindowDeg;
    private State state = State.UNHOMED;
    private double started, stableSince, minimum, maximum;
    private int zeroTicks;
    private String reason = "Not homed";
    public TurretHomingController(double travelSeconds, double stableSeconds,
                                  double timeoutSeconds, double stableWindowDeg) {
        if (!RobotMath.finite(travelSeconds, stableSeconds, timeoutSeconds, stableWindowDeg)
                || travelSeconds <= 0 || stableSeconds <= 0 || timeoutSeconds <= travelSeconds + stableSeconds
                || stableWindowDeg <= 0) throw new IllegalArgumentException("Invalid homing configuration");
        this.travelSeconds = travelSeconds; this.stableSeconds = stableSeconds;
        this.timeoutSeconds = timeoutSeconds; this.stableWindowDeg = stableWindowDeg;
    }
    public void start(double now) { started = now; state = State.MOVING_HOME; reason = "Travel interval"; }
    public void invalidate() { state = State.UNHOMED; reason = "Manual zero/override requires rehome"; }
    public void update(double now, int rawTicks, double rawAngleDeg) {
        if (state == State.UNHOMED || state == State.READY || state == State.FAULT) return;
        if (!RobotMath.finite(now, rawAngleDeg) || now < started || now - started >= timeoutSeconds) {
            state = State.FAULT; reason = "Home settling timeout / invalid encoder"; return;
        }
        if (state == State.MOVING_HOME && now - started >= travelSeconds) {
            state = State.SETTLING; stableSince = now; minimum = maximum = rawAngleDeg;
            reason = "Waiting for stable encoder window";
        }
        if (state == State.SETTLING) {
            minimum = Math.min(minimum, rawAngleDeg); maximum = Math.max(maximum, rawAngleDeg);
            if (maximum - minimum > stableWindowDeg) {
                stableSince = now; minimum = maximum = rawAngleDeg;
            } else if (now - stableSince >= stableSeconds) {
                zeroTicks = rawTicks; state = State.READY; reason = "Estimated home (no absolute sensor)";
            }
        }
    }
    public boolean commandsHome() { return state == State.MOVING_HOME || state == State.SETTLING; }
    public boolean ready() { return state == State.READY; }
    public State state() { return state; }
    public String reason() { return reason; }
    public int zeroTicks() { return zeroTicks; }
}
