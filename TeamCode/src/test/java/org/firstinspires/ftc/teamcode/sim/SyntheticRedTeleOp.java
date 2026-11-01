package org.firstinspires.ftc.teamcode.sim;

import org.firstinspires.ftc.teamcode.*;

/** TEST ONLY, never annotated or packaged: exercises inherited coordination with artificial shot rows. */
public class SyntheticRedTeleOp extends RedMecanumTeleOp {
    @Override protected ShotCoordinator createShotCoordinator(TurretSubsystem turret) {
        GoalTarget target = TeleOpFieldReference.target(getAlliance(), TurretSubsystem.HiveSide.AUDIENCE);
        ShotProfile synthetic = new ShotProfile(target.geometryId, true,
                new ShotProfile.Row(.2, 5, 2800), new ShotProfile.Row(5, 5, 2800));
        return new ShotCoordinator(turret, target, ShootingConfiguration.MOUNT, synthetic,
                ShootingConfiguration.firing(), true, ShotCoordinator.AimMode.TELEOP_ODOMETRY);
    }
}
