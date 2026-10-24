package org.firstinspires.ftc.teamcode;

/** Immutable observation shared by the camera thread and the autonomous loop. */
public final class PollenTarget {
    public final long captureTimeNanos;
    public final double centerXPx;
    public final double centerYPx;
    public final double areaPx;
    public final double contactXPx, contactYPx;
    public final double forwardMm;
    public final double rightMm;
    public final double bearingDeg; // Positive is right / clockwise, matching the drivetrain.

    private PollenTarget(long captureTimeNanos, double centerXPx, double centerYPx,
                         double areaPx, double forwardMm, double rightMm, double contactXPx, double contactYPx) {
        this.captureTimeNanos = captureTimeNanos;
        this.centerXPx = centerXPx;
        this.centerYPx = centerYPx;
        this.areaPx = areaPx; this.contactXPx = contactXPx; this.contactYPx = contactYPx;
        this.forwardMm = forwardMm;
        this.rightMm = rightMm;
        bearingDeg = Math.toDegrees(Math.atan2(rightMm, forwardMm));
    }

    public PollenTarget withContactPixels(double x, double y) {
        return new PollenTarget(captureTimeNanos,centerXPx,centerYPx,areaPx,forwardMm,rightMm,x,y);
    }

    /**
     * Intersects the ray through the blob's lowest contour point with the floor.
     * Camera axes: +X right, +Y down, +Z optical forward; pitch is positive downward.
     * Assumes an upright camera facing forward, with negligible roll and yaw.
     * This is an estimate of floor contact, not a measured ball-center range.
     */
    public static PollenTarget fromFloorContact(long captureTimeNanos,
            double centerXPx, double centerYPx, double contactXPx, double contactYPx,
            double areaPx, double fxPx, double fyPx, double cxPx, double cyPx,
            double heightMm, double downPitchDeg, double forwardOffsetMm, double rightOffsetMm) {
        if (!RobotMath.finite(contactXPx, contactYPx, cxPx, cyPx, downPitchDeg, forwardOffsetMm, rightOffsetMm)
                || !Double.isFinite(fxPx) || !Double.isFinite(fyPx) || fxPx <= 0.0 || fyPx <= 0.0
                || !Double.isFinite(heightMm) || heightMm <= 0.0
                || !Double.isFinite(areaPx) || areaPx <= 0.0
                || !Double.isFinite(centerXPx) || !Double.isFinite(centerYPx)) {
            return null;
        }
        double imageRight = (contactXPx - cxPx) / fxPx;
        double imageDown = (contactYPx - cyPx) / fyPx;
        double pitchRad = Math.toRadians(downPitchDeg);
        double rayForward = Math.cos(pitchRad) - imageDown * Math.sin(pitchRad);
        double rayDown = Math.sin(pitchRad) + imageDown * Math.cos(pitchRad);
        if (!Double.isFinite(rayDown) || rayDown <= 0.0 || rayForward <= 0.0) {
            return null; // At/above the horizon or behind the camera.
        }
        double scaleMm = heightMm / rayDown;
        double forwardMm = scaleMm * rayForward + forwardOffsetMm;
        double rightMm = scaleMm * imageRight + rightOffsetMm;
        if (!Double.isFinite(forwardMm) || !Double.isFinite(rightMm) || forwardMm <= 0.0) {
            return null;
        }
        return new PollenTarget(captureTimeNanos, centerXPx, centerYPx, areaPx, forwardMm, rightMm, contactXPx, contactYPx);
    }
}
