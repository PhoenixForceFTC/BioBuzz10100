package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.ArrayList;
import java.util.List;

public class Limelight {

    private Limelight3A limelight;

    private HardwareMap hardwareMap;

    private Telemetry telemetry;

    public Limelight(HardwareMap h, Telemetry t, Limelight3A l) {

        limelight = l;

        hardwareMap = h;

        telemetry = t;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("Limelight initialized - waiting for start");
        telemetry.update();

        limelight.stop();
    }

    public ArrayList<LimelightData> scan(){

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // result fiducials are the apriltag information only, and the list is for the multiple april tag data.
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            if (fiducials.isEmpty()) {
                telemetry.addLine("No AprilTags detected");
                return null;
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
            return null;
        }
    }
}