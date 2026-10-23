package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Limelight-only AprilTag detection with no alliance or tag ID filtering. */
public class LimelightTagSubsystem implements AutoCloseable {
    public static final String HW_LIMELIGHT = "limelight";
    public static final int APRILTAG_PIPELINE = 0;
    public static final long MAX_STALENESS_MS = 150;

    private final Limelight3A limelight;
    private LLResult result;

    public LimelightTagSubsystem(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, HW_LIMELIGHT);
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();
    }

    public void update() {
        result = limelight.getLatestResult();
    }

    public double getFrameTimestamp() {
        return result == null ? Double.NEGATIVE_INFINITY : result.getTimestamp();
    }

    private boolean isFresh() {
        return result != null && result.getStaleness() >= 0
                && result.getStaleness() <= MAX_STALENESS_MS;
    }

    public boolean hasDetectedTag() {
        if (!isFresh() || !result.isValid()) return false;
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            if (tag != null) return true;
        }
        return false;
    }

    public void addTelemetry(Telemetry telemetry) {
        boolean fresh = isFresh();
        int count = 0;
        StringBuilder ids = new StringBuilder();
        if (fresh && result.isValid()) {
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                if (tag == null) continue;
                if (count > 0) ids.append(", ");
                ids.append(tag.getFiducialId());
                count++;
            }
        }
        telemetry.addData("AprilTag", count > 0 ? "DETECTED" : "NOT DETECTED");
        telemetry.addData("Tag count", count);
        telemetry.addData("Tag IDs", count > 0 ? ids.toString() : "None");
        telemetry.addData("Camera data", result == null ? "Waiting for results"
                : fresh ? "Fresh" : "Stale (ignored)");
    }

    @Override
    public void close() {
        limelight.stop();
    }
}
