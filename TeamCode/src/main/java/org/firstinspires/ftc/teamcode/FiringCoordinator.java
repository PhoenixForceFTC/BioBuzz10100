package org.firstinspires.ftc.teamcode;

/** Shared timed feed attempts. No launched-ball count can be inferred from the full sensor. */
public final class FiringCoordinator {
    public enum State { IDLE, PREPARING, GATE_OPENING, FEEDING, RECOVERY, INHIBITED, FAULT }
    public final double gateTravelSeconds, feedToExitSeconds, feedWindowSeconds, recoverySeconds, stableSeconds;
    private State state = State.IDLE;
    private double entered, stableSince = Double.NaN;
    private boolean gateOpen, feeding, committed;
    private boolean previousManual;
    private int attempts;
    private String reason = "Idle";
    public FiringCoordinator(double gateTravelSeconds, double feedToExitSeconds, double feedWindowSeconds,
                             double recoverySeconds, double stableSeconds) {
        if (!RobotMath.finite(gateTravelSeconds, feedToExitSeconds, feedWindowSeconds, recoverySeconds, stableSeconds)
                || gateTravelSeconds <= 0 || feedToExitSeconds <= 0 || feedWindowSeconds <= 0
                || recoverySeconds <= 0 || stableSeconds <= 0) throw new IllegalArgumentException("Invalid firing timing");
        this.gateTravelSeconds = gateTravelSeconds; this.feedToExitSeconds = feedToExitSeconds;
        this.feedWindowSeconds = feedWindowSeconds; this.recoverySeconds = recoverySeconds; this.stableSeconds = stableSeconds;
    }
    public void update(double now, boolean auto, boolean manual, boolean inhibit, boolean flywheelStop,
                       boolean hardValid, boolean tightTracking, boolean looseTracking, String rejection) {
        gateOpen = feeding = false;
        if (previousManual && !manual) enter(State.PREPARING, now, "Manual override released: requalify automatic readiness");
        previousManual = manual;
        if (committed && now - entered > feedToExitSeconds && state != State.FEEDING) committed = false;
        if (inhibit) { enter(State.INHIBITED, now, "Operator feed inhibit"); return; }
        // Explicit manual feed bypasses shot readiness, including flywheel stop. Inhibit always wins.
        boolean requested = manual || auto;
        if (!requested) { enter(State.IDLE, now, "Idle"); return; }
        if (!manual && (!hardValid || flywheelStop)) {
            enter(State.INHIBITED, now, flywheelStop ? "Flywheel stop" : rejection); return;
        }
        if (state == State.IDLE || state == State.INHIBITED || state == State.FAULT) enter(State.PREPARING, now, "Preparing");
        if (state == State.RECOVERY) {
            if (now - entered >= recoverySeconds) enter(State.PREPARING, now, "Flywheel recovery");
            return;
        }
        if (!manual && (state == State.GATE_OPENING || state == State.FEEDING) && !looseTracking) {
            enter(State.RECOVERY, now, "Tracking / RPM dropped; committed ball may still exit"); return;
        }
        if (state == State.PREPARING) {
            if (manual || tightTracking) {
                if (Double.isNaN(stableSince)) stableSince = now;
                if (manual || now - stableSince >= stableSeconds) enter(State.GATE_OPENING, now, "Gate command-to-feed delay");
            } else { stableSince = Double.NaN; reason = "Waiting for stable turret / hood / RPM"; }
        }
        if (state == State.GATE_OPENING) {
            gateOpen = true;
            if (now - entered >= gateTravelSeconds) { enter(State.FEEDING, now, "Timed feed attempt"); attempts++; }
        }
        if (state == State.FEEDING) {
            gateOpen = feeding = committed = true;
            if (!manual && now - entered >= feedWindowSeconds) {
                enter(State.RECOVERY, now, "Timed recovery"); gateOpen = feeding = false;
            }
        }
    }
    private void enter(State next, double now, String reason) {
        if (state != next) { state = next; entered = now; stableSince = Double.NaN; }
        this.reason = reason;
    }
    public void stop(double now) { gateOpen = feeding = false; enter(State.IDLE, now, "Stopped; a committed ball cannot be recalled"); }
    public State state() { return state; }
    public String reason() { return reason; }
    public boolean gateOpen() { return gateOpen; }
    public boolean feeding() { return feeding; }
    public boolean ballMayBeCommitted() { return committed; }
    public int feedAttempts() { return attempts; }
}
