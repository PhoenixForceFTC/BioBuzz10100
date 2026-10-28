package org.firstinspires.ftc.teamcode;

/** Approved routes must be supplied by the team; synthetic tests are never competition coordinates. */
public final class AutonomousStrategy {
    public static final class Waypoint {
        public final double x, y, heading;
        public Waypoint(double x, double y, double heading) {
            if (!RobotMath.finite(x, y, heading)) throw new IllegalArgumentException("Invalid waypoint");
            this.x = x; this.y = y; this.heading = heading;
        }
    }
    /** Axis-aligned keep-out box in meters; inflated by the robot clearance radius. */
    public static final class Obstacle {
        public final double minX, minY, maxX, maxY;
        public Obstacle(double minX, double minY, double maxX, double maxY) {
            if (!RobotMath.finite(minX, minY, maxX, maxY) || minX >= maxX || minY >= maxY) throw new IllegalArgumentException("Invalid keep-out");
            this.minX = minX; this.minY = minY; this.maxX = maxX; this.maxY = maxY;
        }
    }
    public final String name;
    public final boolean approved, preload;
    public final TurretSubsystem.Alliance alliance;
    public final Waypoint start;
    public final GoalTarget target;
    public final double robotRadius, matchSeconds, parkReserveSeconds, navigationSeconds, collectionSeconds, scoreSeconds;
    public final int repeats;
    private final Waypoint[] collection, scoring, parking;
    private final Obstacle[] obstacles;
    public AutonomousStrategy(String name, boolean approved, TurretSubsystem.Alliance alliance, Waypoint start,
                              GoalTarget target, Waypoint[] collection, Waypoint[] scoring, Waypoint[] parking,
                              Obstacle[] obstacles, double robotRadius, double matchSeconds, double parkReserveSeconds,
                              double navigationSeconds, double collectionSeconds, double scoreSeconds, boolean preload, int repeats) {
        if (name == null || name.isEmpty() || alliance == null || start == null || target == null || target.alliance != alliance
                || collection == null || scoring == null || parking == null || obstacles == null
                || collection.length == 0 || scoring.length == 0 || parking.length == 0
                || !RobotMath.finite(robotRadius, matchSeconds, parkReserveSeconds, navigationSeconds, collectionSeconds, scoreSeconds)
                || robotRadius <= 0 || robotRadius >= 1 || matchSeconds <= 0 || matchSeconds > 30
                || parkReserveSeconds <= 0 || parkReserveSeconds >= matchSeconds || navigationSeconds <= 0
                || collectionSeconds <= 0 || scoreSeconds <= 0 || repeats < 0 || repeats > 3)
            throw new IllegalArgumentException("Invalid autonomous strategy / time budget");
        this.name = name; this.approved = approved; this.alliance = alliance; this.start = start; this.target = target;
        this.collection = collection.clone(); this.scoring = scoring.clone(); this.parking = parking.clone(); this.obstacles = obstacles.clone();
        this.robotRadius = robotRadius; this.matchSeconds = matchSeconds; this.parkReserveSeconds = parkReserveSeconds;
        this.navigationSeconds = navigationSeconds; this.collectionSeconds = collectionSeconds; this.scoreSeconds = scoreSeconds;
        this.preload = preload; this.repeats = repeats;
        for (Obstacle o : this.obstacles) if (o == null) throw new IllegalArgumentException("Null obstacle");
        checkRoute(start, this.collection); checkRoute(this.collection[this.collection.length - 1], this.scoring);
        checkRoute(this.scoring[this.scoring.length - 1], this.parking);
        if (repeats > 0) checkRoute(this.scoring[this.scoring.length - 1], this.collection);
    }
    private void checkRoute(Waypoint from, Waypoint[] route) {
        for (Waypoint to : route) {
            if (to == null || !segmentClear(from.x, from.y, to.x, to.y)) throw new IllegalArgumentException("Route violates field / clearance constraints");
            from = to;
        }
    }
    public Waypoint[] collection() { return collection.clone(); }
    public Waypoint[] scoring() { return scoring.clone(); }
    public Waypoint[] parking() { return parking.clone(); }
    public boolean segmentClear(double x0, double y0, double x1, double y1) {
        double edge = FieldGeometry.FIELD_HALF_WIDTH_METERS - robotRadius;
        if (!RobotMath.finite(x0, y0, x1, y1) || Math.abs(x0) > edge || Math.abs(y0) > edge || Math.abs(x1) > edge || Math.abs(y1) > edge) return false;
        for (Obstacle o : obstacles) {
            double lo = 0, hi = 1;
            double[] origin = {x0, y0}, delta = {x1 - x0, y1 - y0};
            double[] minimum = {o.minX - robotRadius, o.minY - robotRadius}, maximum = {o.maxX + robotRadius, o.maxY + robotRadius};
            for (int axis = 0; axis < 2; axis++) {
                if (Math.abs(delta[axis]) < 1e-9) {
                    if (origin[axis] < minimum[axis] || origin[axis] > maximum[axis]) { lo = 2; break; }
                } else {
                    double a = (minimum[axis] - origin[axis]) / delta[axis], b = (maximum[axis] - origin[axis]) / delta[axis];
                    lo = Math.max(lo, Math.min(a, b)); hi = Math.min(hi, Math.max(a, b));
                }
            }
            if (lo <= hi) return false;
        }
        return true;
    }
    public static final AutonomousStrategy RED_ROUTE = null, BLUE_ROUTE = null;
    public static AutonomousStrategy configured(TurretSubsystem.Alliance alliance) {
        // Deliberately no invented field routes. Supply validated red/blue data here after approval.
        return alliance == TurretSubsystem.Alliance.RED ? RED_ROUTE : BLUE_ROUTE;
    }
}
