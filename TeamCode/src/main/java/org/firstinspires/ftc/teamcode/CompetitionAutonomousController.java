package org.firstinspires.ftc.teamcode;

/** Match sequencing reuses the collection controller and shared firing path. Outputs stop every update. */
public final class CompetitionAutonomousController {
    public static final double STATIONARY_SPEED_METERS_PER_SECOND = .03, STATIONARY_TURN_RAD_PER_SECOND = .03;
    public static final double COLLECTION_LOOKAHEAD_METERS = .3;
    public enum State { IDLE, PRELOAD, TO_COLLECTION, COLLECTING, TO_SCORING, SCORING, PARKING, COMPLETE, FAULT }
    private final AutonomousStrategy strategy;
    private final PollenNavigator collection;
    private final WaypointFollower navigation = new WaypointFollower();
    private State state = State.IDLE;
    private double start, entered, axial, lateral, yaw, intake;
    private boolean fire, loading;
    private int repeats;
    private String reason = "Waiting for START";
    public CompetitionAutonomousController(AutonomousStrategy strategy, PollenConfiguration config) {
        this.strategy = strategy; collection = new PollenNavigator(config);
    }
    public void start(double now) {
        clear(); start = entered = now; repeats = 0;
        if (strategy == null || !strategy.approved) { enter(State.FAULT, now, "No approved autonomous strategy"); return; }
        if (strategy.preload) enter(State.PRELOAD, now, "Optional preload scoring");
        else travel(State.TO_COLLECTION, now, strategy.collection());
    }
    public void update(double now, MotionSample pose, PollenTarget target, boolean collectionReady,
                       boolean queueFull, boolean stopRequested) {
        clear();
        if (stopRequested) { stop(now); return; }
        if (state == State.IDLE || finished()) return;
        if (now - start >= strategy.matchSeconds) { stop(now); reason = "Global match deadline"; return; }
        if (pose == null || !pose.fieldReferenced || !pose.fresh(now, .1)) {
            enter(State.FAULT, now, "Localization unavailable: stop fallback"); navigation.stop(); return;
        }
        if (!strategy.segmentClear(pose.x, pose.y, pose.x, pose.y)) {
            enter(State.FAULT, now, "Current pose violates robot clearance"); navigation.stop(); return;
        }
        if (state != State.PARKING && now - start >= strategy.matchSeconds - strategy.parkReserveSeconds) {
            travel(State.PARKING, now, strategy.parking());
        }
        if (state == State.TO_COLLECTION || state == State.TO_SCORING || state == State.PARKING) {
            navigation.update(now, pose);
            if (navigation.state() == WaypointFollower.State.FAULT) {
                if (state == State.PARKING) enter(State.FAULT, now, navigation.reason());
                else travel(State.PARKING, now, strategy.parking());
                return;
            }
            if (navigation.state() == WaypointFollower.State.ARRIVED) {
                if (state == State.PARKING) { enter(State.COMPLETE, now, "Parked"); return; }
                if (state == State.TO_COLLECTION) {
                    enter(State.COLLECTING, now, "Bounded collection"); collection.start((long) (now * 1e9), pose.x * 1000, pose.y * 1000);
                } else enter(State.SCORING, now, "Stationary scoring window");
                return;
            }
            axial = navigation.axial(); lateral = navigation.lateral(); yaw = navigation.yaw();
            fire = state == State.TO_SCORING && navigation.movingShotsAllowed();
            return;
        }
        if (state == State.COLLECTING) {
            collection.update((long) (now * 1e9), target, pose.x * 1000, pose.y * 1000,
                    Math.toDegrees(pose.heading), collectionReady, queueFull);
            if (collection.isFinished() || now - entered >= strategy.collectionSeconds) {
                collection.stop((long) (now * 1e9)); travel(State.TO_SCORING, now, strategy.scoring()); return;
            }
            axial = collection.getAxialPower(); yaw = collection.getYawPower(); intake = collection.getIntakePower(); loading = true;
            // Stop before a bounded collection step would enter a keep-out; no obstacle avoidance is attempted.
            double lookahead = axial > 0 ? COLLECTION_LOOKAHEAD_METERS : 0;
            if (!strategy.segmentClear(pose.x, pose.y, pose.x + lookahead * Math.cos(pose.heading), pose.y + lookahead * Math.sin(pose.heading)))
                travel(State.PARKING, now, strategy.parking());
            return;
        }
        if (state == State.PRELOAD || state == State.SCORING) {
            // Otherwise stationary shots only. Never steer in response to a failed shot solution.
            fire = Math.hypot(pose.vx, pose.vy) <= STATIONARY_SPEED_METERS_PER_SECOND && Math.abs(pose.omega) <= STATIONARY_TURN_RAD_PER_SECOND;
            if (now - entered >= strategy.scoreSeconds) {
                fire = false;
                if (state == State.PRELOAD || repeats++ < strategy.repeats) travel(State.TO_COLLECTION, now, strategy.collection());
                else travel(State.PARKING, now, strategy.parking());
            }
        }
    }
    private void travel(State next, double now, AutonomousStrategy.Waypoint[] route) {
        enter(next, now, next.toString()); navigation.start(now, strategy, route);
    }
    private void enter(State next, double now, String message) { state = next; entered = now; reason = message; clear(); }
    private void clear() { axial = lateral = yaw = intake = 0; fire = loading = false; }
    public void stop(double now) { collection.stop((long) (now * 1e9)); navigation.stop(); enter(State.COMPLETE, now, "STOP"); }
    public boolean finished() { return state == State.COMPLETE || state == State.FAULT; }
    public State state() { return state; }
    public String reason() { return reason; }
    public double axial() { return axial; }
    public double lateral() { return lateral; }
    public double yaw() { return yaw; }
    public double intake() { return intake; }
    public boolean fireRequested() { return fire; }
    public boolean loading() { return loading; }
}
