package org.firstinspires.ftc.teamcode;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.algorithm.ForesightV3;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;

/** Numerical guard for the pinned snapshot's zero-distance hold normalization. */
public final class PedroForesight extends ForesightV3 {
    public static final double HOLD_NORMALIZATION_EPSILON_IN = 1e-4;
    public PedroForesight(ForesightConfig config) { super(config); }

    @Override public DrivePowers calculateHold(Drivetrain drivetrain, Pose target, MotionState state,
                                                boolean useScaling, double deltaTime) {
        Pose projected = state.pose().plus(getBrakeDisplacement(state.twist(), state.pose().heading()));
        if (target.distance(projected) < HOLD_NORMALIZATION_EPSILON_IN) {
            // ForesightV3 divides this displacement by its length before its deadband check.
            // This offset is far below minCorrectionDistance; Pedro still produces zero translation.
            target = new Pose(projected.x() + HOLD_NORMALIZATION_EPSILON_IN, projected.y(), target.heading());
        }
        return super.calculateHold(drivetrain, target, state, useScaling, deltaTime);
    }
}
