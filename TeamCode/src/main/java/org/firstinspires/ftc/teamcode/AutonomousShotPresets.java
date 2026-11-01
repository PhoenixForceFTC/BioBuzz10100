package org.firstinspires.ftc.teamcode;

/** Fixed robot-local setpoints. No measured competition presets exist yet. */
public final class AutonomousShotPresets {
    public static final class Preset {
        public final String name, provenance;
        public final double yawDeg, hoodTravelDeg, outputRpm;
        public final boolean measured;
        public Preset(String name, double yawDeg, double hoodTravelDeg, double outputRpm,
                      boolean measured, String provenance) {
            AutonomousCommands.range("preset yawDeg", yawDeg, TurretSubsystem.ROTATION_MIN_ANGLE_DEG, TurretSubsystem.ROTATION_MAX_ANGLE_DEG);
            AutonomousCommands.range("preset hoodTravelDeg", hoodTravelDeg, 0, TurretSubsystem.PITCH_MAX_DEG);
            AutonomousCommands.range("preset outputRpm", outputRpm, 1, TurretSubsystem.SHOOTER_MAX_RPM);
            if (name == null || name.isEmpty() || provenance == null || provenance.isEmpty())
                throw new IllegalArgumentException("Preset requires name and provenance");
            this.name = name; this.yawDeg = yawDeg; this.hoodTravelDeg = hoodTravelDeg;
            this.outputRpm = outputRpm; this.measured = measured; this.provenance = provenance;
        }
    }
    public static final Preset DIAGNOSTIC = new Preset("diagnostic-example", 0, 0, 3000,
            false, "UNMEASURED example for bench preparation; automatic feed inhibited");
    private AutonomousShotPresets() { }
    public static Preset require(String name) {
        if (DIAGNOSTIC.name.equals(name)) return DIAGNOSTIC;
        throw new IllegalArgumentException("Unknown shot preset '" + name + "'; configured: diagnostic-example (UNMEASURED)");
    }
}
