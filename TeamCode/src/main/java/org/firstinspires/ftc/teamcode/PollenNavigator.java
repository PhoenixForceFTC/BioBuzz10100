package org.firstinspires.ftc.teamcode;

/** Non-blocking autonomous coordinator. Owns no hardware; outputs robot-centric commands. */
public class PollenNavigator {
    public static final double AUTO_TIME_LIMIT_SECONDS = 28.0;
    public static final double HARDWARE_WAIT_SECONDS = 4.0;
    public static final double SEARCH_TIME_LIMIT_SECONDS = 4.0;
    public static final double SEARCH_TURN_LIMIT_DEG = 90.0;
    public static final double SEARCH_YAW_POWER = 0.12;
    public static final double APPROACH_TIME_LIMIT_SECONDS = 7.0;
    public static final double TARGET_LOST_GRACE_SECONDS = 0.35;
    public static final double MAX_TARGET_AGE_SECONDS = 0.2;
    public static final double MAX_TOTAL_TRAVEL_MM = 2400.0;
    public static final double YAW_KP_PER_DEG = 0.018;
    public static final double MAX_YAW_POWER = 0.22;
    public static final double DRIVE_KP_PER_MM = 0.0007;
    public static final double MAX_DRIVE_POWER = 0.28;
    public static final double MIN_DRIVE_POWER = 0.10;
    public static final double DRIVE_ALIGNMENT_DEG = 12.0;
    public static final double PICKUP_ALIGNMENT_DEG = 4.0;
    // Tune from the robot center to the floor contact just before pollen leaves camera view.
    public static final double PICKUP_START_FORWARD_MM = PollenConfiguration.diagnostic().pickupStartMm;
    public static final double PICKUP_CONFIRM_SECONDS = 0.12;
    public static final int PICKUP_CONFIRM_FRAMES = 3;
    public static final double PICKUP_POWER = 0.16;
    public static final double PICKUP_TRAVEL_MM = PollenConfiguration.diagnostic().pickupTravelMm;
    public static final double PICKUP_TIME_LIMIT_SECONDS = 0.9;
    public static final double INTAKE_SETTLE_SECONDS = 0.4;

    public enum State { WAITING_FOR_HARDWARE, SEARCHING, APPROACHING, COLLECTING, SETTLING, COMPLETE, TIMED_OUT, FAULT }

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
    private double axial;
    private double yaw;
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
            beginSearch(nowNanos, headingDeg);
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
                    yaw = SEARCH_YAW_POWER;
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
                    beginSearch(nowNanos, headingDeg);
                }
                return;
            }
            lostSinceNanos = 0;
            status = "Tracking pollen";
            intakePower = 1.0;
            yaw = clip(target.bearingDeg * YAW_KP_PER_DEG, MAX_YAW_POWER);
            double distanceErrorMm = target.forwardMm - pickupStartMm;
            if (Math.abs(target.bearingDeg) <= DRIVE_ALIGNMENT_DEG && distanceErrorMm > 0.0) {
                axial = Math.min(MAX_DRIVE_POWER, Math.max(MIN_DRIVE_POWER, distanceErrorMm * DRIVE_KP_PER_MM));
            }
            if (distanceErrorMm <= 0.0 && Math.abs(target.bearingDeg) <= PICKUP_ALIGNMENT_DEG) {
                axial = 0.0;
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
                axial = PICKUP_POWER;
                // Pinpoint heading is CCW positive; drivetrain yaw is CW positive.
                yaw = clip(normalizeDegrees(headingDeg - referenceHeadingDeg) * YAW_KP_PER_DEG, MAX_YAW_POWER);
            }
            return;
        }
        if (state == State.SETTLING) {
            intakePower = 1.0;
            if (secondsSince(nowNanos, stateStartNanos) >= INTAKE_SETTLE_SECONDS) {
                beginSearch(nowNanos, headingDeg);
                intakePower = 0.0;
            }
        }
    }

    private void beginSearch(long nowNanos, double headingDeg) {
        enter(State.SEARCHING, nowNanos, "Searching for pollen");
        referenceHeadingDeg = headingDeg;
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

    private void stopCommands() { axial = 0.0; yaw = 0.0; intakePower = 0.0; }
    private static double secondsSince(long nowNanos, long thenNanos) { return (nowNanos - thenNanos) * 1e-9; }
    private static double clip(double value, double max) { return Math.max(-max, Math.min(max, value)); }
    private static double normalizeDegrees(double angleDeg) { return ((angleDeg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0; }
    public State getState() { return state; }
    public String getStatus() { return status; }
    public double getAxialPower() { return axial; }
    public double getYawPower() { return yaw; }
    public double getIntakePower() { return intakePower; }
    public double getTotalTravelMm() { return totalTravelMm; }
    public int getPickupAttempts() { return pickupAttempts; }
    public boolean isFinished() { return state == State.COMPLETE || state == State.TIMED_OUT || state == State.FAULT; }
}
