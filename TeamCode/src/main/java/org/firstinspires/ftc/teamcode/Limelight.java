package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.ArrayList;
import java.util.List;

//usage: purely scans using the limelight. scan() returns an empty list (never null) when no tags are seen.

public class Limelight {

    private Limelight3A limelight;

    private HardwareMap hardwareMap;

    private Telemetry telemetry;

    public Limelight(HardwareMap h, Telemetry t) {

        hardwareMap = h;

        telemetry = t;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        // pipeline 0 must be set to "Fiducial Markers" (AprilTag 36h11, 82.55mm / 3.25in tags) in the Limelight web UI.
        limelight.pipelineSwitch(0);
        // start() begins polling; don't stop() it here or getLatestResult() never refreshes.
        limelight.start();

        telemetry.addLine("Limelight initialized - waiting for start");
    }

    public void stop() {
        limelight.stop();
    }

    public ArrayList<LimelightData> scan(){

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // result fiducials are the apriltag information only, and the list is for the multiple april tag data.
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            if (fiducials.isEmpty()) {
                telemetry.addLine("No AprilTags detected");
                return new ArrayList<>();
            } else {
                ArrayList<LimelightData> returnData = new ArrayList<>();
                // var name fiducial of typye LLResultTypes.FiducialResult is an iterator of fiducials list.
                for (LLResultTypes.FiducialResult fiducial : fiducials) {

                    int id = fiducial.getFiducialId();

                    // Angle from the camera to the tag, in degrees
                    double angleX = fiducial.getTargetXDegrees();
                    double angleY = fiducial.getTargetYDegrees();

                    // Position of the tag relative to the camera (meters)
                    Pose3D targetPoseCameraSpace = fiducial.getTargetPoseCameraSpace();

                    double x = targetPoseCameraSpace.getPosition().x;
                    double y = targetPoseCameraSpace.getPosition().y;
                    double z = targetPoseCameraSpace.getPosition().z;

                    returnData.add(new LimelightData(id, angleX, angleY, x, y, z));
                }
                return returnData;
            }

        } else {
            telemetry.addLine("No valid Limelight result");
            return new ArrayList<>();
        }
    }
}