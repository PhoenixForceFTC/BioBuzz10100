package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.horizon36596.simloop.fakehardware.FakeDevice;
import org.horizon36596.simloop.fakehardware.FakeLimelight3A;
import org.json.JSONObject;
import java.net.InetAddress;
import java.util.ArrayList;
import java.util.List;

/**
 * Explicit TEST-ONLY convention experiment, not physical camera validation.
 * SimLoop alpha.2 tag +Z is into the print; the existing robot expects an outward +Z.
 * Rx(pi) changes only the target-coordinate basis. Projection, visibility, measured position,
 * latency, staleness, dropouts and physical turret motion still come from the upstream fake.
 * The raw-camera regression remains alongside the normalized-camera regression.
 */
final class TargetNormalCameraAdapter extends Limelight3A implements FakeDevice {
    private final FakeLimelight3A source;
    TargetNormalCameraAdapter(FakeLimelight3A source) {
        super(null, "limelight", InetAddress.getLoopbackAddress());
        super.stop();
        this.source = source;
    }
    @Override public void update(double dt) { source.update(dt); }
    @Override public void start() { source.start(); }
    @Override public void stop() { if (source != null) source.stop(); }
    @Override public void pause() { source.pause(); }
    @Override public boolean isRunning() { return source.isRunning(); }
    @Override public boolean pipelineSwitch(int index) { return source.pipelineSwitch(index); }
    @Override public void setPollRateHz(int hz) { source.setPollRateHz(hz); }
    @Override public LLStatus getStatus() { return source.getStatus(); }
    @Override public String getConnectionInfo() { return "SimLoop target-normal adapter (unvalidated on robot)"; }
    @Override public LLResult getLatestResult() {
        try { return new NormalizedResult(source.getLatestResult()); }
        catch (org.json.JSONException e) { throw new IllegalStateException(e); }
    }

    private static final class NormalizedResult extends LLResult {
        final LLResult raw;
        NormalizedResult(LLResult raw) throws org.json.JSONException { super(new JSONObject()); this.raw = raw; }
        @Override public boolean isValid() { return raw.isValid(); }
        @Override public double getTimestamp() { return raw.getTimestamp(); }
        @Override public long getStaleness() { return raw.getStaleness(); }
        @Override public long getControlHubTimeStamp() { return raw.getControlHubTimeStamp(); }
        @Override public long getControlHubTimeStampNanos() { return raw.getControlHubTimeStampNanos(); }
        @Override public double getCaptureLatency() { return raw.getCaptureLatency(); }
        @Override public double getTargetingLatency() { return raw.getTargetingLatency(); }
        @Override public double getParseLatency() { return raw.getParseLatency(); }
        @Override public double getTx() { return raw.getTx(); }
        @Override public double getTy() { return raw.getTy(); }
        @Override public double getTa() { return raw.getTa(); }
        @Override public int getPipelineIndex() { return raw.getPipelineIndex(); }
        @Override public List<FiducialResult> getFiducialResults() {
            List<FiducialResult> tags = new ArrayList<>();
            for (FiducialResult tag : raw.getFiducialResults()) tags.add(new FiducialResult(new JSONObject()) {
                @Override public int getFiducialId() { return tag.getFiducialId(); }
                @Override public double getTargetArea() { return tag.getTargetArea(); }
                @Override public double getTargetXDegrees() { return tag.getTargetXDegrees(); }
                @Override public double getTargetYDegrees() { return tag.getTargetYDegrees(); }
                @Override public Pose3D getTargetPoseCameraSpace() {
                    Pose3D pose = tag.getTargetPoseCameraSpace();
                    return new Pose3D(pose.getPosition(), new YawPitchRollAngles(AngleUnit.RADIANS,
                            pose.getOrientation().getYaw(AngleUnit.RADIANS),
                            pose.getOrientation().getPitch(AngleUnit.RADIANS),
                            pose.getOrientation().getRoll(AngleUnit.RADIANS) + Math.PI, 0));
                }
            });
            return tags;
        }
    }
}
