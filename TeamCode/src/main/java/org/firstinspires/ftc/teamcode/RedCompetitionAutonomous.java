package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto: Red Match (Route Required)", group = "Competition")
public final class RedCompetitionAutonomous extends CompetitionAutonomous {
    @Override protected TurretSubsystem.Alliance alliance() { return TurretSubsystem.Alliance.RED; }
}
