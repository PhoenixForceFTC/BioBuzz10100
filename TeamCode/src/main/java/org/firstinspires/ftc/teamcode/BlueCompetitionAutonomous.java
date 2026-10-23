package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto: Blue Match (Route Required)", group = "Competition")
public final class BlueCompetitionAutonomous extends CompetitionAutonomous {
    @Override protected TurretSubsystem.Alliance alliance() { return TurretSubsystem.Alliance.BLUE; }
}
