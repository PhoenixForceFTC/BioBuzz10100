package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import static com.pedropathing.api.Paths.*;

@Autonomous(name = "Niranjan's simple auto")
public class SimplePedroNiranjan extends OpMode {

    private Follower follower;

    private final PoseFactory p = PoseFactory.degrees();


    private final Pose p1 = new Pose(0,0,90);

    @Override
    public void init(){

    }

    @Override
    public void loop(){

    }
}
