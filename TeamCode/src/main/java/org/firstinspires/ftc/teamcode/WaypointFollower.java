package org.firstinspires.ftc.teamcode;

/** Non-blocking proportional pose navigation with arrival, progress and timeout checks. */
public final class WaypointFollower {
    public static final double MAX_POSE_AGE_SECONDS = .1, ARRIVAL_DISTANCE_METERS = .04;
    public static final double ARRIVAL_HEADING_RAD = Math.toRadians(3), PROGRESS_METERS = .01, STALL_SECONDS = 1.5;
    public static final double DRIVE_GAIN_PER_METER = 1.3, MAX_DRIVE_POWER = .35;
    public static final double HEADING_GAIN_PER_RAD = .8, MAX_YAW_POWER = .25;
    public static final double PROGRESS_HEADING_METERS_PER_RAD = .15;
    public enum State { IDLE, DRIVING, ARRIVED, FAULT }
    private State state = State.IDLE;
    private AutonomousStrategy strategy;
    private AutonomousStrategy.Waypoint[] route;
    private int index;
    private double started, progressTime, bestError;
    private double axial, lateral, yaw;
    private String reason = "Idle";
    public void start(double now, AutonomousStrategy strategy, AutonomousStrategy.Waypoint[] route) {
        this.strategy = strategy; this.route = route.clone(); index = 0; started = progressTime = now;
        bestError = Double.POSITIVE_INFINITY; state = State.DRIVING; reason = "Navigating"; clear();
    }
    public void update(double now, MotionSample p) {
        clear();
        if (state != State.DRIVING) return;
        if (p == null || !p.fieldReferenced || !p.fresh(now, MAX_POSE_AGE_SECONDS)) { fail("Localization unavailable"); return; }
        if (now - started >= strategy.navigationSeconds) { fail("Route timeout"); return; }
        AutonomousStrategy.Waypoint w = route[index];
        if (!strategy.segmentClear(p.x, p.y, w.x, w.y)) { fail("Current segment violates clearance"); return; }
        double dx = w.x - p.x, dy = w.y - p.y, headingError = RobotMath.wrap(w.heading - p.heading);
        double distance = Math.hypot(dx, dy);
        if (distance <= ARRIVAL_DISTANCE_METERS && Math.abs(headingError) <= ARRIVAL_HEADING_RAD) {
            if (++index == route.length) { state = State.ARRIVED; reason = "Arrived"; }
            else { progressTime = now; bestError = Double.POSITIVE_INFINITY; }
            return;
        }
        double error = distance + PROGRESS_HEADING_METERS_PER_RAD * Math.abs(headingError);
        if (error < bestError - PROGRESS_METERS) { bestError = error; progressTime = now; }
        if (now - progressTime > STALL_SECONDS) { fail("Navigation stalled"); return; }
        double forward = dx * Math.cos(p.heading) + dy * Math.sin(p.heading);
        double left = -dx * Math.sin(p.heading) + dy * Math.cos(p.heading);
        double magnitude = Math.max(1, Math.hypot(forward, left) * DRIVE_GAIN_PER_METER / MAX_DRIVE_POWER);
        axial = forward * DRIVE_GAIN_PER_METER / magnitude; lateral = -left * DRIVE_GAIN_PER_METER / magnitude;
        yaw = -RobotMath.clip(headingError * HEADING_GAIN_PER_RAD, MAX_YAW_POWER); // drivetrain is clockwise/right positive.
    }
    private void fail(String reason) { state = State.FAULT; this.reason = reason; clear(); }
    private void clear() { axial = lateral = yaw = 0; }
    public void stop() { state = State.IDLE; clear(); }
    public State state() { return state; }
    public String reason() { return reason; }
    public double axial() { return axial; }
    public double lateral() { return lateral; }
    public double yaw() { return yaw; }
}
