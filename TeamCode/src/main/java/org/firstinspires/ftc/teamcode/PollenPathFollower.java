package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import java.util.function.BiFunction;

/** Converts the test planner's targets into Pedro paths/holds without restarting every camera frame. */
public final class PollenPathFollower {
    public static final double REPLAN_INTERVAL_SECONDS = .2;
    public static final double REPLAN_DISTANCE_IN = 3;
    public static final double HOLD_DISTANCE_IN = .5;
    public static final double REPLAN_HEADING_RAD = Math.toRadians(5);
    private final Follower follower;
    private final BiFunction<Path, Double, Path> atSpeed;
    private final Runnable immediateStop;
    private PollenNavigator.Motion lastMotion = PollenNavigator.Motion.STOP;
    private Pose lastGoal;
    private long lastPlan;

    public PollenPathFollower(PedroDrive drive) { this(drive.follower(), drive::atSpeed, drive::stop); }
    PollenPathFollower(Follower follower, BiFunction<Path, Double, Path> atSpeed, Runnable immediateStop) {
        this.follower = follower; this.atSpeed = atSpeed; this.immediateStop = immediateStop;
    }
    public void update(PollenNavigator navigator, long now) {
        update(navigator.getMotion(), new Pose(navigator.getGoalXMm() / PedroConfiguration.MM_PER_INCH,
                navigator.getGoalYMm() / PedroConfiguration.MM_PER_INCH,
                Math.toRadians(navigator.getGoalHeadingDeg())), now);
    }
    void update(PollenNavigator.Motion motion, Pose goal, long now) {
        if (motion == PollenNavigator.Motion.STOP) { stop(); return; }
        double remaining = goal.distance(follower.pose());
        if (motion == PollenNavigator.Motion.HOLD || remaining <= HOLD_DISTANCE_IN) follower.hold(goal);
        else if (motion != lastMotion || lastGoal == null || !follower.following()
                || ((now - lastPlan) * 1e-9 >= REPLAN_INTERVAL_SECONDS && (goal.distance(lastGoal) >= REPLAN_DISTANCE_IN
                || Math.abs(RobotMath.wrap(goal.heading() - lastGoal.heading())) >= REPLAN_HEADING_RAD))) {
            follower.follow(atSpeed.apply(Paths.line(follower.pose(), goal).linear(follower.pose().heading(), goal.heading()),
                    PedroConfiguration.DEFAULT_PATH_SPEED_FRACTION));
            lastGoal = goal; lastPlan = now;
        }
        lastMotion = motion;
    }
    public void stop() {
        immediateStop.run(); lastMotion = PollenNavigator.Motion.STOP; lastGoal = null; lastPlan = 0;
    }
}
