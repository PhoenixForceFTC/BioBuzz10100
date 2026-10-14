package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import static com.pedropathing.api.Paths.*;

@Autonomous(name = "Drive Forward Example")
public class DriveForwardTest extends OpMode {

    // ---- change this to set how far the robot drives, in inches ----
    private static final double FORWARD_DISTANCE_INCHES = 20;

    private Follower follower;

    // Builds Poses in degrees. Use PoseFactory.radians() if you prefer radians.
    private final PoseFactory p = PoseFactory.degrees();

    // Where the robot starts. Must match where you physically place it on the field.
    private final Pose startPose = p.of(0, 0, 0);

    // Straight ahead of the start pose by FORWARD_DISTANCE_INCHES, same heading.
    private final Pose targetPose = p.of(1, 0, 0);

    private boolean started = false;
    private boolean finished = false;

    // A straight path from startPose to targetPose, holding the start heading
    // the whole way (paths default to tangential heading otherwise).
    private Path driveForward() {
        return line(startPose, targetPose).constant(startPose.heading());
    }

    @Override
    public void init() {
        // Builds the follower using the drivetrain, localizer, and Foresight
        // config from Constants.java.
        follower = Constants.create(hardwareMap);

        // Tells the localizer where the robot physically is right now.
        // Skipping this makes every path start from the wrong place.
        follower.setPose(startPose);
    }

    @Override
    public void loop() {
        follower.update();

        if (!started) {
            follower.follow(driveForward());
            started = true;
        }

        telemetry.addData("finished", finished);
        telemetry.addData("busy", follower.isBusy());
        telemetry.addData("x (in)", follower.pose().x());
        telemetry.addData("y (in)", follower.pose().y());
        telemetry.update();
    }
}