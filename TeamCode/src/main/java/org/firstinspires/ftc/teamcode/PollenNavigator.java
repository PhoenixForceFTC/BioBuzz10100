package org.firstinspires.ftc.teamcode;

/** Test-only collection decisions. Pedro follows the requested pose; this class never computes motor powers. */
public class PollenNavigator {
    public static final double AUTO_TIME_LIMIT_SECONDS = 28.0;
    public static final double HARDWARE_WAIT_SECONDS = 4.0;
    public static final double SEARCH_TIME_LIMIT_SECONDS = 4.0;
    public static final double SEARCH_TURN_LIMIT_DEG = 90.0;
    public static final double APPROACH_TIME_LIMIT_SECONDS = 7.0;
    public static final double TARGET_LOST_GRACE_SECONDS = 0.35;
    public static final double MAX_TARGET_AGE_SECONDS = 0.2;
    public static final double MAX_TOTAL_TRAVEL_MM = 2400.0;
    public static final double DRIVE_ALIGNMENT_DEG = 12.0;
    public static final double PICKUP_ALIGNMENT_DEG = 4.0;
    // Tune from the robot center to the floor contact just before pollen leaves camera view.
    public static final double PICKUP_START_FORWARD_MM = PollenConfiguration.diagnostic().pickupStartMm;
    public static final double PICKUP_CONFIRM_SECONDS = 0.12;
    public static final int PICKUP_CONFIRM_FRAMES = 3;
    public static final double PICKUP_TRAVEL_MM = PollenConfiguration.diagnostic().pickupTravelMm;
    public static final double PICKUP_TIME_LIMIT_SECONDS = 0.9;
    public static final double INTAKE_SETTLE_SECONDS = 0.4;

    public enum State { WAITING_FOR_HARDWARE, SEARCHING, APPROACHING, COLLECTING, SETTLING, COMPLETE, TIMED_OUT, FAULT }
    public enum Motion { STOP, HOLD, FOLLOW }
    private Motion motion = Motion.STOP;
    private double goalXMm, goalYMm, goalHeadingDeg;
    private double searchXMm, searchYMm, pickupXMm, pickupYMm;

    private final double pickupStartMm, pickupTravelMm;
    public PollenNavigator() { this(PollenConfiguration.diagnostic()); }
    public PollenNavigator(PollenConfiguration config) { pickupStartMm = config.pickupStartMm; pickupTravelMm = config.pickupTravelMm; }
    public void stop(long now) { stopCommands(); enter(State.COMPLETE, now, "Stopped"); started = false; }
    private State state = State.WAITING_FOR_HARDWARE;
    private String status = "Waiting for START";
    private boolean started;
    private long startNanos;
    private long stateStartNanos;
    private long lostSinceNanos;
    private long closeSinceNanos;
    private long lastCloseCaptureNanos, lastTargetCaptureNanos;
    private int closeFrameCount;
    private double lastXMm;
    private double lastYMm;
    private double totalTravelMm;
    private double pickupStartTravelMm;
    private double referenceHeadingDeg;
    private double intakePower;
    private int pickupAttempts;

    public void start(long nowNanos, double xMm, double yMm) {
        started = true;
        startNanos = nowNanos;
        lastXMm = xMm;
        lastYMm = yMm;
        totalTravelMm = 0.0;
        pickupAttempts = 0; lastTargetCaptureNanos = 0;
        enter(State.WAITING_FOR_HARDWARE, nowNanos, "Waiting for camera and Pinpoint");
        stopCommands();
    }

    /** hardwareReady requires STREAMING fresh camera frames and READY Pinpoint data. */
    public void update(long nowNanos, PollenTarget target, double xMm, double yMm,
                       double headingDeg, boolean hardwareReady, boolean queueFull) {
        stopCommands();
        if (!started || isFinished()) {
            return;
        }
        if (queueFull) {
            enter(State.COMPLETE, nowNanos, "Queue full");
            return;
        }
        if (secondsSince(nowNanos, startNanos) >= AUTO_TIME_LIMIT_SECONDS) {
            enter(State.TIMED_OUT, nowNanos, "Autonomous time limit");
            return;
        }
        if (!hardwareReady) {
            if (state != State.WAITING_FOR_HARDWARE || secondsSince(nowNanos, stateStartNanos) >= HARDWARE_WAIT_SECONDS) {
                enter(State.FAULT, nowNanos, "Camera or Pinpoint unavailable / stale");
            }
            return;
        }
        if (!Double.isFinite(xMm) || !Double.isFinite(yMm) || !Double.isFinite(headingDeg)) {
            enter(State.FAULT, nowNanos, "Invalid odometry pose");
            return;
        }
        if (state == State.WAITING_FOR_HARDWARE) {
            // Calibration may have returned NaNs at START; anchor travel only after READY.
            lastXMm = xMm;
            lastYMm = yMm;
            beginSearch(nowNanos, xMm, yMm, headingDeg);
        }
        totalTravelMm += Math.hypot(xMm - lastXMm, yMm - lastYMm);
        lastXMm = xMm;
        lastYMm = yMm;
        if (totalTravelMm >= MAX_TOTAL_TRAVEL_MM) {
            enter(State.COMPLETE, nowNanos, "Travel limit reached");
            return;
        }
        if (target != null && (secondsSince(nowNanos, target.captureTimeNanos) < 0.0
                || secondsSince(nowNanos, target.captureTimeNanos) > MAX_TARGET_AGE_SECONDS)) {
            target = null;
        }
        if (target != null) {
            if (target.captureTimeNanos < lastTargetCaptureNanos) target = null;
            else lastTargetCaptureNanos = target.captureTimeNanos;
        }
        if (state == State.SEARCHING) {
            if (target == null) {
                if (secondsSince(nowNanos, stateStartNanos) >= SEARCH_TIME_LIMIT_SECONDS
                        || Math.abs(normalizeDegrees(headingDeg - referenceHeadingDeg)) >= SEARCH_TURN_LIMIT_DEG) {
                    enter(State.TIMED_OUT, nowNanos, "No pollen in bounded search");
                } else {
                    request(Motion.HOLD, searchXMm, searchYMm, referenceHeadingDeg - SEARCH_TURN_LIMIT_DEG);
                }
                return;
            }
            enter(State.APPROACHING, nowNanos, "Tracking pollen");
        }
        if (state == State.APPROACHING) {
            if (secondsSince(nowNanos, stateStartNanos) >= APPROACH_TIME_LIMIT_SECONDS) {
                enter(State.TIMED_OUT, nowNanos, "Approach time limit");
                return;
            }
            if (target == null) {
                resetCloseConfirmation();
                if (lostSinceNanos == 0) {
                    lostSinceNanos = nowNanos;
                }
                status = "Target lost: stopped";
                if (secondsSince(nowNanos, lostSinceNanos) >= TARGET_LOST_GRACE_SECONDS) {
                    beginSearch(nowNanos, xMm, yMm, headingDeg);
                }
                return;
            }
            lostSinceNanos = 0;
            status = "Tracking pollen";
            intakePower = 1.0;
            double targetHeadingDeg = headingDeg - target.bearingDeg;
            double distanceErrorMm = target.forwardMm - pickupStartMm;
            // Align in place before requesting a straight approach in the observed direction.
            double travelMm = Math.abs(target.bearingDeg) <= DRIVE_ALIGNMENT_DEG ? Math.max(0, distanceErrorMm) : 0;
            double headingRad = Math.toRadians(targetHeadingDeg);
            request(travelMm > 0 ? Motion.FOLLOW : Motion.HOLD,
                    xMm + travelMm * Math.cos(headingRad), yMm + travelMm * Math.sin(headingRad), targetHeadingDeg);
            if (distanceErrorMm <= 0.0 && Math.abs(target.bearingDeg) <= PICKUP_ALIGNMENT_DEG) {
                if (closeSinceNanos == 0) {
                    closeSinceNanos = target.captureTimeNanos;
                }
                if (target.captureTimeNanos > lastCloseCaptureNanos) {
                    closeFrameCount++;
                    lastCloseCaptureNanos = target.captureTimeNanos;
                }
                if (closeFrameCount >= PICKUP_CONFIRM_FRAMES
                        && secondsSince(target.captureTimeNanos, closeSinceNanos) >= PICKUP_CONFIRM_SECONDS) {
                    enter(State.COLLECTING, nowNanos, "Bounded pickup through camera blind spot");
                    pickupStartTravelMm = totalTravelMm;
                    referenceHeadingDeg = headingDeg;
                    pickupXMm = xMm + pickupTravelMm * Math.cos(Math.toRadians(headingDeg));
                    pickupYMm = yMm + pickupTravelMm * Math.sin(Math.toRadians(headingDeg));
                    request(Motion.FOLLOW, pickupXMm, pickupYMm, referenceHeadingDeg);
                }
            } else {
                resetCloseConfirmation();
            }
            return;
        }
        if (state == State.COLLECTING) {
            intakePower = 1.0;
            if (totalTravelMm - pickupStartTravelMm >= pickupTravelMm
                    || secondsSince(nowNanos, stateStartNanos) >= PICKUP_TIME_LIMIT_SECONDS) {
                pickupAttempts++;
                enter(State.SETTLING, nowNanos, "Finishing intake; pickup is not sensor-confirmed");
            } else {
                request(Motion.FOLLOW, pickupXMm, pickupYMm, referenceHeadingDeg);
            }
            return;
        }
        if (state == State.SETTLING) {
            intakePower = 1.0;
            if (secondsSince(nowNanos, stateStartNanos) >= INTAKE_SETTLE_SECONDS) {
                beginSearch(nowNanos, xMm, yMm, headingDeg);
                intakePower = 0.0;
            }
        }
    }

    private void beginSearch(long nowNanos, double xMm, double yMm, double headingDeg) {
        enter(State.SEARCHING, nowNanos, "Searching for pollen");
        referenceHeadingDeg = headingDeg;
        searchXMm = xMm; searchYMm = yMm;
    }

    private void enter(State next, long nowNanos, String message) {
        state = next;
        stateStartNanos = nowNanos;
        lostSinceNanos = 0;
        resetCloseConfirmation();
        status = message;
    }

    private void resetCloseConfirmation() {
        closeSinceNanos = 0;
        lastCloseCaptureNanos = 0;
        closeFrameCount = 0;
    }

    private void stopCommands() { motion = Motion.STOP; intakePower = 0.0; }
    private void request(Motion motion, double xMm, double yMm, double headingDeg) {
        this.motion = motion; goalXMm = xMm; goalYMm = yMm; goalHeadingDeg = headingDeg;
    }
    private static double secondsSince(long nowNanos, long thenNanos) { return (nowNanos - thenNanos) * 1e-9; }
    private static double normalizeDegrees(double angleDeg) { return ((angleDeg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0; }
    public State getState() { return state; }
    public String getStatus() { return status; }
    public Motion getMotion() { return motion; }
    public double getGoalXMm() { return goalXMm; }
    public double getGoalYMm() { return goalYMm; }
    public double getGoalHeadingDeg() { return goalHeadingDeg; }
    public double getIntakePower() { return intakePower; }
    public double getTotalTravelMm() { return totalTravelMm; }
    public int getPickupAttempts() { return pickupAttempts; }
    public boolean isFinished() { return state == State.COMPLETE || state == State.TIMED_OUT || state == State.FAULT; }
}
