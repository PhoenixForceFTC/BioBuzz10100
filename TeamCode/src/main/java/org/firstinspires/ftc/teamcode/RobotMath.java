package org.firstinspires.ftc.teamcode;

/** Planning uses meters, seconds, radians. Robot: +X forward, +Y left; yaw counterclockwise.
 * Team field: center origin, +X toward blue, +Y toward scoring table. Surveys/routes must use it.
 */
public final class RobotMath {
    private RobotMath() { }
    public static double wrap(double a) { return Math.atan2(Math.sin(a), Math.cos(a)); }
    public static double clip(double x, double limit) { return Math.max(-limit, Math.min(limit, x)); }
    public static boolean finite(double... values) {
        for (double value : values) if (!Double.isFinite(value)) return false;
        return true;
    }
    public static final class Vec {
        public final double x, y;
        public Vec(double x, double y) { this.x = x; this.y = y; }
        public Vec plus(Vec b) { return new Vec(x + b.x, y + b.y); }
        public Vec minus(Vec b) { return new Vec(x - b.x, y - b.y); }
        public Vec rotate(double a) { return new Vec(x * Math.cos(a) - y * Math.sin(a), x * Math.sin(a) + y * Math.cos(a)); }
        public double norm() { return Math.hypot(x, y); }
        public double angle() { return Math.atan2(y, x); }
    }
}
