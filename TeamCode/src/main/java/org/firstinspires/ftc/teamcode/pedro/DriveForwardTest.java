package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "AutoDriver Example")
public class DriveForwardTest extends OpMode {

    private Follower follower;
    private AutoDriver driver;

    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose = p.of(0, 0, 0);

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

        driver = new AutoDriver(follower);

        // AUTOMATIC is the default: each move starts when the last one finishes.
        driver.moveTo(20, 0, 0)      // forward 20 in
                .moveTo(0, 0, 0)       // back to start
                .moveTo(0, 20, 90);    // left 20 in, ending facing left

        // For MANUAL mode instead:
        // driver.setMode(AutoDriver.Mode.MANUAL);
        // ...then call driver.next() whenever you want the next move to start.
    }

    @Override
    public void loop() {
        driver.update();

        telemetry.addData("busy", driver.isBusy());
        telemetry.addData("x (in)", follower.pose().x());
        telemetry.addData("y (in)", follower.pose().y());
        telemetry.update();
    }
}