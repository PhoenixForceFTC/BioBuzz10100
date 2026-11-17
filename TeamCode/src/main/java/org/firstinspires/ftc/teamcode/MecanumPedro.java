package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.api.PoseFactory;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class MecanumPedro {

    private Gamepad gamepad;
    private Telemetry telemetry;
    Follower follower;

    public MecanumPedro(Telemetry t, HardwareMap h, Gamepad g, Pose start){

        gamepad = g;
        telemetry = t;
        follower = Constants.create(h);

        follower.setPose(start);
        follower.update();

    }

    public Pose getPose(){
        return follower.pose();
    }

    public void resetPoseTo(Pose pose){
        follower.setPose(pose);
    }

    public void run(){
        follower.manual(-gamepad.left_stick_y, -gamepad.left_stick_x, -gamepad.right_stick_x);
        follower.update();
        telemetry.addData("Current Pose x: ", String.format("%.2f", follower.pose().x()));
        telemetry.addData("Current Pose x: ", String.format("%.2f", follower.pose().y()));
        telemetry.addData("Current Pose x: ", String.format("%.2f", follower.pose().heading()));
    }
}
