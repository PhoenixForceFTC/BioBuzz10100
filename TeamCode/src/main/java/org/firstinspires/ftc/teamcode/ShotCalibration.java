package org.firstinspires.ftc.teamcode;

import java.util.TreeMap;

/** Hardware-free calibration notebook. Distances are horizontal muzzle-to-opening ranges. */
public final class ShotCalibration {
    public static final double METERS_PER_INCH = .0254;
    private static final class Average {
        int count;
        double rpm, hoodDeg;
        void add(double targetRpm, double hoodTravelDeg) {
            count++;
            rpm += (targetRpm - rpm) / count;
            hoodDeg += (hoodTravelDeg - hoodDeg) / count;
        }
    }
    private final TreeMap<Double, Average> averages = new TreeMap<>();
    private final TreeMap<Double, ShotProfile.Row> rows = new TreeMap<>();
    private double distanceInches = 24, rpm = 3200, hoodDeg = 0;

    public double distanceInches() { return distanceInches; }
    public double rpm() { return rpm; }
    public double hoodDeg() { return hoodDeg; }
    public int hits() {
        Average average = averages.get(distanceInches);
        return average == null ? 0 : average.count;
    }
    public ShotProfile.Row currentAverage() { return rows.get(distanceInches); }
    public int size() { return rows.size(); }

    /** Candidate changes preserve all previous confirmed hits at each exact entered distance. */
    public void configure(double inches, double rpm, double hoodDeg) {
        if (!Double.isFinite(inches) || inches <= 0 || !Double.isFinite(rpm) || rpm <= 0
                || rpm > 6000 || !Double.isFinite(hoodDeg) || hoodDeg < TurretSubsystem.PITCH_MIN_DEG
                || hoodDeg > TurretSubsystem.PITCH_MAX_DEG)
            throw new IllegalArgumentException("Invalid distance, RPM or hood angle");
        distanceInches = inches; this.rpm = rpm; this.hoodDeg = hoodDeg;
    }

    /** Called once after the operator rates a completed feed attempt. Only confirmed hits contribute. */
    public void record(boolean hit) {
        if (hit) {
            Average average = averages.get(distanceInches);
            if (average == null) {
                average = new Average();
                averages.put(distanceInches, average);
            }
            average.add(rpm, hoodDeg);
            rows.put(distanceInches, new ShotProfile.Row(distanceInches * METERS_PER_INCH,
                    average.hoodDeg, average.rpm, Double.NaN));
        }
    }

    public ShotProfile profile(String geometryId) {
        return new ShotProfile(geometryId, true, rows());
    }
    /** Sorted snapshot for continuous Driver Hub telemetry; no storage is read or written. */
    public ShotProfile.Row[] rows() {
        return rows.values().toArray(new ShotProfile.Row[0]);
    }
}
