package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.RobotMath.Vec;

final class ControllerFixtures {
    static MotionSample pose(double t, double x, double y, double h, double vx, double vy, double omega) {
        return new MotionSample(t, x, y, h, vx, vy, omega, 0, .02, true, true, "Synthetic");
    }
    static GoalTarget goal(double x, double y) {
        return new GoalTarget(TurretSubsystem.Alliance.RED, TurretSubsystem.HiveSide.FAR,
                GoalTarget.HiveState.RAISED, x, y, 1.5, true, "synthetic", new GoalTarget.Tag(30, x, y, 1), new GoalTarget.Tag(31, x + .1, y, 1.1));
    }
    static ShooterMount mount() { return new ShooterMount(new Vec(0, 0), new Vec(0, 0), new Vec(0, 0), 0, 0, .2, .3, true); }
    static ShotProfile profile() {
        // Independent synthetic distance/hood/RPM lookup table.
        return new ShotProfile("synthetic", true, new ShotProfile.Row(.2, 2, 3000), new ShotProfile.Row(5, 12, 3400));
    }
    static ShotPlanner planner(ShooterMount mount) { return new ShotPlanner(mount); }
}
