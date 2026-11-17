package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.SimpleLimelightRelocalizer.PIPELINE;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.function.DoubleSupplier;

public class Relocalization {

    Limelight3A limelight;
    String status = "";

    double camDistFromPivot = 3.92;
    long lastTime = 0;
    int acceptableCounts = 0;

    public double distanceToTag(int tagId) {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return -1;

        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            if (tag.getFiducialId() == tagId) {
                Pose3D p = tag.getTargetPoseCameraSpace();   // meters
                double x = p.getPosition().x, y = p.getPosition().y, z = p.getPosition().z;
                return Math.sqrt(x*x + y*y + z*z) * 39.3701;
            }
        }
        return -1;
    }

    public Relocalization(String limelightName, Telemetry t, HardwareMap h){
        limelight = h.get(Limelight3A.class, limelightName);

        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(PIPELINE);
        limelight.start();
    }

    // turretangle in degrees
    boolean relocalize(boolean isForced, Pose beginPos, double turretAngle, Follower f){
        double CameraHeading = beginPos.heading() + turretAngle;

        //limelight.updateRobotOrientation(Math.toDegrees(pedroHeadingToFtc(cameraHeading)));
        double ftcCamDeg = Math.toDegrees(CameraHeading + Math.PI / 2);
        limelight.updateRobotOrientation(ftcCamDeg);
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) { status = "no valid result"; return false; }
        // don't use data over 100 ms old
        if (result.getStaleness() > 100) { status = "stale"; return false; }
        if (result.getBotposeTagCount() < 1) { status = "no tags"; return false; }
        // no readings > 3m away
        if (result.getBotposeAvgDist() > 3.0) { status = "tags too far"; return false; }

        Pose3D botpose = result.getBotpose_MT2();
        if (botpose == null) { status = "no MT2 pose"; return false; }

        // default pos
        double[] lensLocation = ftcMetersToPedroXY(botpose.getPosition().x, botpose.getPosition().y);
        double[] pivotLocation = rotate(2, beginPos.heading());
        double[] offsetFromPivot = rotate(camDistFromPivot, CameraHeading);

        double camX = lensLocation[0] - pivotLocation[0] - offsetFromPivot[0];
        double camY = lensLocation[1] - pivotLocation[1] - offsetFromPivot[1];

        Pose camPose = new Pose(camX, camY, beginPos.heading());

        if (!isForced){
            long now = System.currentTimeMillis();

            double speed = Math.hypot(f.velocity().vx, f.velocity().vy);
            if (speed > 20) { status = "moving too fast"; return false; }

            double cameraRate = f.velocity().omega;
            if (Math.abs(cameraRate) > Math.toRadians(90)) {
                status = "camera turning too fast"; return false;
            }
        }

        double correctionFactor = isForced ? 1.0 : 0.3;
        double correctedX = beginPos.x() + correctionFactor*(camX - beginPos.x());
        double correctedY = beginPos.y() + correctionFactor*(camY - beginPos.y());

        f.setPose(new Pose(correctedX, correctedY, beginPos.heading()));
        lastTime = System.currentTimeMillis();
        acceptableCounts++;
        status = isForced ? "forced" : "applied";
        return true;
    }

    static double[] ftcMetersToPedroXY(double ftcXm, double ftcYm) {
        double ftcX = ftcXm * 39.3701;
        double ftcY = ftcYm * 39.3701;
        return new double[]{ftcY + 72, 72 - ftcX};
    }

    private static double[] rotate(double fwd, double heading) {
        double c = Math.cos(heading), s = Math.sin(heading);
        return new double[]{fwd * c, fwd * s};
    }
}
