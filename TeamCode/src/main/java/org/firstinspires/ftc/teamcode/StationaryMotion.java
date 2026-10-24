package org.firstinspires.ftc.teamcode;

/** Shared, time-qualified noise envelope. A raw excursion revokes qualification immediately. */
public final class StationaryMotion {
    public static final double SPEED_METERS_PER_SECOND = .03, TURN_RAD_PER_SECOND = .03;
    public static final double TURRET_RAD_PER_SECOND = Math.toRadians(3), STABLE_SECONDS = .15;
    private double since = Double.NaN, last = Double.NaN, speed, turn, turret;
    private boolean qualified;
    private long referenceId = Long.MIN_VALUE;
    public boolean update(double now, MotionSample p, double turretRate) {
        if (p == null || !p.fresh(now,.1) || !Double.isFinite(turretRate)) { reset(); return false; }
        if (p.referenceId != referenceId) { reset(); referenceId = p.referenceId; }
        double dt = now-last;
        if (!Double.isFinite(last) || dt < 0 || dt > .1) { since = Double.NaN; speed = turn = turret = 0; }
        last = now;
        double rawSpeed = Math.hypot(p.vx,p.vy), rawTurn = Math.abs(p.omega), rawTurret = Math.abs(turretRate);
        double alpha = dt > 0 && dt <= .1 ? 1-Math.exp(-dt/.05) : 1;
        speed += alpha*(rawSpeed-speed); turn += alpha*(rawTurn-turn); turret += alpha*(rawTurret-turret);
        boolean bounded = rawSpeed <= SPEED_METERS_PER_SECOND && rawTurn <= TURN_RAD_PER_SECOND
                && rawTurret <= TURRET_RAD_PER_SECOND && speed <= SPEED_METERS_PER_SECOND
                && turn <= TURN_RAD_PER_SECOND && turret <= TURRET_RAD_PER_SECOND;
        if (!bounded) since = Double.NaN;
        else if (Double.isNaN(since)) since = now;
        qualified = bounded && now-since >= STABLE_SECONDS;
        return qualified;
    }
    public boolean qualified() { return qualified; }
    public void reset() { since = last = Double.NaN; qualified = false; }
    public MotionSample stationarySample(MotionSample p) {
        if (!qualified) return p;
        return new MotionSample(p.time,p.x,p.y,p.heading,0,0,0,0,0,p.uncertainty,p.ready,p.fieldReferenced,p.status).withReference(p.referenceId);
    }
}
