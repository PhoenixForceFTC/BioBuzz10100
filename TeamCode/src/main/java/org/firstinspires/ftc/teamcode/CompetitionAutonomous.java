package org.firstinspires.ftc.teamcode;

import org.horizon36596.zenith.ivy.AutoFromFile;
import org.horizon36596.zenith.ivy.ZenithRobot;

/** Match entry points load team-authored Zenith assets using the shared Pedro/Ivy robot. */
public abstract class CompetitionAutonomous extends AutoFromFile {
    protected abstract TurretSubsystem.Alliance alliance();
    @Override protected String autoName() { return alliance().name().toLowerCase(java.util.Locale.ROOT) + "-match"; }
    @Override protected ZenithRobot createRobot() { return new BioBuzzRobot(alliance()); }
}
