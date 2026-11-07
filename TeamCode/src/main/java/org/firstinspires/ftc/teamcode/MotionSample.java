package org.firstinspires.ftc.teamcode;

/** Immutable bulk sample. Velocity is in the same fixed frame as position, never robot relative. */
public final class MotionSample {
    public final double time, x, y, heading, vx, vy, omega, acceleration, angularAcceleration, uncertainty;
    public final boolean ready, fieldReferenced;
    public final long referenceId;
    public final String status;
    public MotionSample(double time, double x, double y, double heading, double vx, double vy,
                        double omega, double acceleration, double uncertainty,
                        boolean ready, boolean fieldReferenced, String status) {
        this(time, x, y, heading, vx, vy, omega, acceleration, 0, uncertainty, ready, fieldReferenced, status);
    }
    public MotionSample(double time, double x, double y, double heading, double vx, double vy,
                        double omega, double acceleration, double angularAcceleration, double uncertainty,
                        boolean ready, boolean fieldReferenced, String status) {
        this(time,x,y,heading,vx,vy,omega,acceleration,angularAcceleration,uncertainty,ready,fieldReferenced,status,0);
    }
    private MotionSample(double time, double x, double y, double heading, double vx, double vy,
                        double omega, double acceleration, double angularAcceleration, double uncertainty,
                        boolean ready, boolean fieldReferenced, String status, long referenceId) {
        this.referenceId = referenceId;
        this.time = time; this.x = x; this.y = y; this.heading = heading;
        this.vx = vx; this.vy = vy; this.omega = omega; this.acceleration = acceleration;
        this.angularAcceleration = angularAcceleration;
        this.uncertainty = uncertainty; this.ready = ready; this.fieldReferenced = fieldReferenced;
        this.status = status;
    }
    public boolean fresh(double now, double maxAge) {
        return ready && RobotMath.finite(time, x, y, heading, vx, vy, omega, acceleration, angularAcceleration, uncertainty)
                && uncertainty >= 0 && acceleration >= 0 && angularAcceleration >= 0 && now >= time && now - time <= maxAge;
    }
    public MotionSample shifted(double dx, double dy, double uncertainty) {
        return new MotionSample(time, x + dx, y + dy, heading, vx, vy, omega, acceleration, angularAcceleration,
                uncertainty, ready, fieldReferenced, status, referenceId);
    }
    /**
     * Constant-velocity estimate of this pose {@code seconds} later (fixed-frame velocity, Pinpoint heading
     * rate). Time, rates and provenance are unchanged so freshness still describes the measured sample.
     */
    public MotionSample led(double seconds) {
        if (!(seconds > 0) || !RobotMath.finite(seconds, x, y, heading, vx, vy, omega)) return this;
        return new MotionSample(time, x + vx * seconds, y + vy * seconds, RobotMath.wrap(heading + omega * seconds),
                vx, vy, omega, acceleration, angularAcceleration, uncertainty, ready, fieldReferenced, status, referenceId);
    }
    public MotionSample withReference(long id) {
        return new MotionSample(time,x,y,heading,vx,vy,omega,acceleration,angularAcceleration,uncertainty,ready,fieldReferenced,status,id);
    }
}
