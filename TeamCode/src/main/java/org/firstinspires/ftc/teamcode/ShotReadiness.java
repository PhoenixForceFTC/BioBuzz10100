package org.firstinspires.ftc.teamcode;

/** Competition angle readiness deliberately has no chassis/turret velocity requirement. */
public final class ShotReadiness {
    private ShotReadiness() { }
    public static final double YAW_TOLERANCE_DEG = 3.0;
    public static boolean angleReady(double errorDeg) {
        return Double.isFinite(errorDeg) && Math.abs(errorDeg) <= YAW_TOLERANCE_DEG;
    }
}
