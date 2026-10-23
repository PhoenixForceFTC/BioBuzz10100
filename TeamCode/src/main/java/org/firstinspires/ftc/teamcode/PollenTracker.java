package org.firstinspires.ftc.teamcode;

import java.util.List;

/** Camera-independent frame filtering and continuity, shared with the C270 adapter. */
public final class PollenTracker {
    public static final class Detection {
        public final double x, y, contactX, contactY, area, density, aspect, circularity;
        public final boolean clipped;
        public Detection(double x, double y, double contactX, double contactY, double area,
                         double density, double aspect, double circularity, boolean clipped) {
            this.x = x; this.y = y; this.contactX = contactX; this.contactY = contactY; this.area = area;
            this.density = density; this.aspect = aspect; this.circularity = circularity; this.clipped = clipped;
        }
    }
    private final PollenConfiguration config;
    private PollenTarget last;
    private long lastFrame = Long.MIN_VALUE;
    private String rejection = "No frame";
    public PollenTracker(PollenConfiguration config) { this.config = config; }
    public PollenTarget select(long timestamp, List<Detection> detections, double fx, double fy, double cx, double cy) {
        if (timestamp <= lastFrame) { rejection = "Duplicate / out-of-order pollen frame"; return null; }
        lastFrame = timestamp;
        boolean lock = last != null && timestamp >= last.captureTimeNanos && (timestamp - last.captureTimeNanos) * 1e-9 <= .4;
        PollenTarget best = null; double bestMotion = Double.POSITIVE_INFINITY;
        rejection = "No eligible blob";
        for (Detection d : detections) {
            if (!RobotMath.finite(d.x, d.y, d.contactX, d.contactY, d.area, d.density, d.aspect, d.circularity)) { rejection = "Invalid contour"; continue; }
            if (d.clipped) { rejection = "Clipped contact / blind spot"; continue; }
            if (d.area < 60 || d.area > 16000 || d.density < .65 || d.aspect > 1.8 || d.circularity < .45) { rejection = "Blob shape / area rejected"; continue; }
            PollenTarget candidate = PollenTarget.fromFloorContact(timestamp, d.x, d.y, d.contactX, d.contactY, d.area,
                    fx, fy, cx, cy, config.heightMm, config.pitchDeg, config.forwardMm, config.rightMm);
            if (candidate == null || candidate.forwardMm > 2200) { rejection = "Invalid floor ray / outside range"; continue; }
            double motion = lock ? Math.hypot(candidate.centerXPx - last.centerXPx, candidate.centerYPx - last.centerYPx) : 0;
            if (lock && motion > 65) { rejection = "Outside continuity gate"; continue; }
            if (best == null || (lock ? motion < bestMotion : candidate.areaPx > best.areaPx)) { best = candidate; bestMotion = motion; }
        }
        if (best != null) { last = best; rejection = "Tracking"; }
        return best;
    }
    public String rejection() { return rejection; }
    public static boolean fresh(long capture, long now) { return capture != 0 && now >= capture && (now - capture) * 1e-9 <= .2; }
}
