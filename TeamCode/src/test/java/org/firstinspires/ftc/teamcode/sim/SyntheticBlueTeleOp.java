package org.firstinspires.ftc.teamcode.sim;

import org.firstinspires.ftc.teamcode.TurretSubsystem;

/** Same explicitly artificial shot measurements, with the real blue alliance selection. */
public class SyntheticBlueTeleOp extends SyntheticRedTeleOp {
    @Override protected TurretSubsystem.Alliance getAlliance() { return TurretSubsystem.Alliance.BLUE; }
}
