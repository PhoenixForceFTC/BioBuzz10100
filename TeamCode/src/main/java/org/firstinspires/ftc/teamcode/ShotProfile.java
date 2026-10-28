package org.firstinspires.ftc.teamcode;

/** Measured settings: RPM is flywheel output RPM; hoodDeg is 0..30 deg downward travel from home. */
public final class ShotProfile {
    public static final class Row {
        public final double distance, hoodDeg, rpm;
        public Row(double distance, double hoodDeg, double rpm) {
            this.distance = distance; this.hoodDeg = hoodDeg; this.rpm = rpm;
        }
    }
    public final String geometryId;
    public final boolean calibrated;
    private final Row[] rows;
    public ShotProfile(String geometryId, boolean calibrated, Row... rows) {
        if (geometryId == null || geometryId.isEmpty() || rows == null || rows.length < 2)
            throw new IllegalArgumentException("Profile needs geometry and at least two rows");
        this.geometryId = geometryId; this.calibrated = calibrated; this.rows = rows.clone();
        double previous = 0;
        for (Row row : this.rows) {
            if (row == null || !RobotMath.finite(row.distance, row.hoodDeg, row.rpm)
                    || row.distance <= previous || row.hoodDeg < TurretSubsystem.PITCH_MIN_DEG
                    || row.hoodDeg > TurretSubsystem.PITCH_MAX_DEG
                    || row.rpm <= 0 || row.rpm > ShooterSpeed.FLYWHEEL_MAX_TARGET_RPM)
                throw new IllegalArgumentException("Invalid profile row / coverage ordering");
            previous = row.distance;
        }
    }
    public Row sample(double distance) {
        if (!Double.isFinite(distance) || distance < rows[0].distance || distance > rows[rows.length - 1].distance) return null;
        for (int i = 1; i < rows.length; i++) {
            Row a = rows[i - 1], b = rows[i];
            if (distance <= b.distance) {
                double t = (distance - a.distance) / (b.distance - a.distance);
                return new Row(distance, a.hoodDeg + t * (b.hoodDeg - a.hoodDeg),
                        a.rpm + t * (b.rpm - a.rpm));
            }
        }
        return null;
    }
    public static ShotProfile example() {
        // Keep unmeasured examples below the configured target ceiling as the motor baseline changes.
        double ratio = ShooterSpeed.FLYWHEEL_REVS_PER_MOTOR_REV
                * BareMotorSpecifications.CONFIGURED_FREE_SPEED_RPM / BareMotorSpecifications.NOMINAL_RPM;
        return new ShotProfile("UNMEASURED", false,
                new Row(.6096, 2, 3200 * ratio), new Row(.9144, 4.5, 3550 * ratio),
                new Row(1.2192, 7, 3900 * ratio), new Row(1.524, 9.5, 4250 * ratio),
                new Row(1.8288, 11.5, 4650 * ratio), new Row(2.1336, 13.5, 5050 * ratio),
                new Row(2.4384, 15, 5450 * ratio));
    }
}
