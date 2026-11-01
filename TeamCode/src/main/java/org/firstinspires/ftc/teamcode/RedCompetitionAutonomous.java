package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Auto: Red Match (Route Required)", group = "Competition")
@Disabled // Match files generate their own single selectable entry.
public final class RedCompetitionAutonomous extends CompetitionAutonomous {
    @Override protected TurretSubsystem.Alliance alliance() { return TurretSubsystem.Alliance.RED; }
}
