package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Auto: Blue Match (Route Required)", group = "Competition")
@Disabled // Match files generate their own single selectable entry.
public final class BlueCompetitionAutonomous extends CompetitionAutonomous {
    @Override protected TurretSubsystem.Alliance alliance() { return TurretSubsystem.Alliance.BLUE; }
}
