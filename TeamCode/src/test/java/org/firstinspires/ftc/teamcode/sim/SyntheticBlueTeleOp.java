package org.firstinspires.ftc.teamcode.sim;

import org.firstinspires.ftc.teamcode.*;

/** Same explicitly artificial shot measurements, with the real blue alliance selection. */
public class SyntheticBlueTeleOp extends BlueMecanumTeleOp {
    @Override protected ShotCoordinator createShotCoordinator(TurretSubsystem turret) {
        return SyntheticRedTeleOp.syntheticCoordinator(turret,getAlliance());
    }
}
