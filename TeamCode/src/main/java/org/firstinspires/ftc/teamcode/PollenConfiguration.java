package org.firstinspires.ftc.teamcode;

/** Camera/pickup model and provenance; zero offsets are valid measurements when marked measured. */
public final class PollenConfiguration {
    // Team-confirmed C270 mount: upright, straight forward, optical axis below horizontal.
    public static final double CAMERA_HEIGHT_MM = 195;
    public static final double CAMERA_PITCH_DOWN_DEG = 10;
    public static final double CAMERA_FORWARD_MM = 248;
    public static final double CAMERA_RIGHT_MM = 0;
    public final double heightMm, pitchDeg, forwardMm, rightMm, horizontalFovDeg;
    public final double fx, fy, cx, cy;
    private double[] distortion = new double[0];
    public double[] distortionCoefficients() { return distortion.clone(); }
    public PollenConfiguration withDistortion(double... coefficients) {
        PollenConfiguration copy = new PollenConfiguration(heightMm,pitchDeg,forwardMm,rightMm,horizontalFovDeg,fx,fy,cx,cy,
                mountMeasured,intrinsicsMeasured,pickupMeasured,colorThresholdsMeasured,allowApproximateAutonomous,
                hueMin,hueMax,saturationMin,valueMin,pickupStartMm,pickupTravelMm);
        new PollenCameraModel(320,240,fx,fy,cx,cy,coefficients);
        copy.distortion = coefficients.clone();
        if (copy.autonomousEligible(false) && !PollenCameraModel.approximate(copy).pickupVisible(copy))
            throw new IllegalArgumentException("Distorted camera pickup transition outside visible floor region");
        return copy;
    }
    public final boolean mountMeasured, intrinsicsMeasured, pickupMeasured, colorThresholdsMeasured, allowApproximateAutonomous;
    public final double hueMin, hueMax, saturationMin, valueMin, pickupStartMm, pickupTravelMm;
    public PollenConfiguration(double heightMm, double pitchDeg, double forwardMm, double rightMm,
                               double horizontalFovDeg, double fx, double fy, double cx, double cy,
                               boolean mountMeasured, boolean intrinsicsMeasured, boolean pickupMeasured,
                               boolean colorThresholdsMeasured, boolean allowApproximateAutonomous, double hueMin, double hueMax,
                               double saturationMin, double valueMin, double pickupStartMm, double pickupTravelMm) {
        if (!RobotMath.finite(heightMm, pitchDeg, forwardMm, rightMm, horizontalFovDeg,
                fx, fy, cx, cy, hueMin, hueMax, saturationMin, valueMin, pickupStartMm, pickupTravelMm)
                || heightMm <= 0 || pitchDeg <= -80 || pitchDeg >= 80 || horizontalFovDeg <= 1 || horizontalFovDeg >= 170
                || fx < 0 || fy < 0 || cx < 0 || cy < 0 || (intrinsicsMeasured && (fx <= 0 || fy <= 0))
                || hueMin < 0 || hueMax > 180 || hueMin >= hueMax || saturationMin < 0 || saturationMin > 255
                || valueMin < 0 || valueMin > 255 || pickupStartMm <= 0 || pickupTravelMm <= 0)
            throw new IllegalArgumentException("Invalid pollen camera / color / pickup configuration");
        this.heightMm = heightMm; this.pitchDeg = pitchDeg; this.forwardMm = forwardMm; this.rightMm = rightMm;
        this.horizontalFovDeg = horizontalFovDeg; this.fx = fx; this.fy = fy; this.cx = cx; this.cy = cy;
        this.mountMeasured = mountMeasured; this.intrinsicsMeasured = intrinsicsMeasured;
        this.pickupMeasured = pickupMeasured; this.allowApproximateAutonomous = allowApproximateAutonomous;
        this.colorThresholdsMeasured = colorThresholdsMeasured;
        this.hueMin = hueMin; this.hueMax = hueMax; this.saturationMin = saturationMin; this.valueMin = valueMin;
        this.pickupStartMm = pickupStartMm; this.pickupTravelMm = pickupTravelMm;
        if (mountMeasured && pickupMeasured && colorThresholdsMeasured && (intrinsicsMeasured || allowApproximateAutonomous)
                && !PollenCameraModel.approximate(this).pickupVisible(this))
            throw new IllegalArgumentException("Enabled pickup transition is outside visible floor region");
    }
    public boolean autonomousEligible(boolean sdkCalibration) {
        return mountMeasured && pickupMeasured && colorThresholdsMeasured && (intrinsicsMeasured || sdkCalibration || allowApproximateAutonomous);
    }
    public String status(boolean sdkCalibration) {
        if (!mountMeasured) return "Mount UNMEASURED: diagnostic range only";
        if (!pickupMeasured) return "Pickup geometry UNMEASURED";
        if (!colorThresholdsMeasured) return "Color thresholds UNMEASURED";
        if (!intrinsicsMeasured && !sdkCalibration && !allowApproximateAutonomous) return "Approximate intrinsics: autonomous disabled";
        return intrinsicsMeasured ? "Measured intrinsics" : sdkCalibration ? "SDK intrinsics" : "APPROXIMATE: autonomous explicitly enabled";
    }
    public static PollenConfiguration diagnostic() {
        // Mount is confirmed; intrinsics, color thresholds and pickup still need calibration.
        return new PollenConfiguration(CAMERA_HEIGHT_MM, CAMERA_PITCH_DOWN_DEG,
                CAMERA_FORWARD_MM, CAMERA_RIGHT_MM, 60, 0, 0, 0, 0,
                true, false, false, false, false, 20, 40, 80, 80, 650, 450);
    }
}
