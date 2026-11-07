package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;

//usage: works out how far the shooter is from pointing at our goal, using only the Pinpoint odometry.
// create it during init, call start() once when the match starts, and update() every loop.
// it doesn't touch any motors. pass getHeadingErrorDeg() to RotateToLimelight to get a turn power.
// see TeamCode/ODOMETRY_AIM_NOTES.md for setup and testing.

public class OdometryAim {

    // ---- [MEASURE] where the robot starts and where the goal is ----
    // null = not measured yet, and isReady() stays false until both poses for an alliance are set.
    // inches and radians, all in one field frame, for example:
    //     RED_START = new Pose(0, 0, Math.toRadians(0));
    //     RED_GOAL  = new Pose(100, 24);
    // the SDK's tag library has no field positions for BioBuzz, so these can't come from there.
    private static final Pose RED_START = null;
    private static final Pose BLUE_START = null;
    private static final Pose RED_GOAL = null;
    private static final Pose BLUE_GOAL = null;

    // Which way the shooter points compared to the robot's front, in degrees. positive = to the left.
    // 0 = shoots out the front, 180 = shoots out the back.
    private static final double SHOOTER_OFFSET_DEG = 0;

    private final PinpointLocalizer localizer;
    private final Telemetry telemetry;

    private Pose goal = null;
    private boolean ready = false;

    public OdometryAim(HardwareMap h, Telemetry t) {
        // localizer only, NOT Constants.create(). that also builds Pedro's Mecanum, which sets the
        // drive motor directions and float mode, overriding what the TeleOp set up.
        // the robot has to sit still here because this recalibrates the Pinpoint's IMU.
        localizer = new PinpointLocalizer(h, Constants.localizerConfig);
        telemetry = t;
    }

    // Call once, after the alliance is final. Tells the Pinpoint where the robot physically is.
    public void start(boolean redAlliance) {
        Pose start = redAlliance ? RED_START : BLUE_START;
        goal = redAlliance ? RED_GOAL : BLUE_GOAL;
        ready = start != null && goal != null;

        if (start != null) {
            localizer.setPose(start);
        }
    }

    // Call every loop, even when not aiming, so the pose on the telemetry is always current.
    public void update() {
        localizer.update();
    }

    // False until the start and goal for this alliance have been measured. Don't aim when false.
    public boolean isReady() {
        return ready;
    }

    // Same convention as Limelight tx: positive = the goal is to the right, so turn right.
    // Only call when isReady() is true.
    public double getHeadingErrorDeg() {
        return headingErrorDeg(localizer.pose(), goal, SHOOTER_OFFSET_DEG);
    }

    public void showTelemetry() {
        Pose pose = localizer.pose();
        telemetry.addData("Odo pose", "x=%.1f y=%.1f h=%.1f°",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));

        if (ready) {
            telemetry.addData("Odo goal", "%.1f in away, error %.1f°",
                    pose.distance(goal), getHeadingErrorDeg());
        } else {
            telemetry.addLine("Odo aim off: measure start and goal in OdometryAim.java");
        }
    }

    // Pure math with no hardware, so it can be checked off the robot.
    public static double headingErrorDeg(Pose robot, Pose goal, double shooterOffsetDeg) {
        // atan2 gives the field direction from robot to goal: 0 = +X, positive = counter-clockwise,
        // which matches the Pinpoint's heading.
        double headingToGoal = Math.atan2(goal.y() - robot.y(), goal.x() - robot.x());
        double shooterHeading = robot.heading() + Math.toRadians(shooterOffsetDeg);

        // positive here means the goal is to the LEFT, because headings count counter-clockwise.
        double error = headingToGoal - shooterHeading;

        // wrap to -180..180 so the robot takes the short way around instead of spinning almost a full circle.
        double wrapped = Math.atan2(Math.sin(error), Math.cos(error));

        // flip it, since the drive's turn input is positive to the RIGHT.
        return -Math.toDegrees(wrapped);
    }
}
