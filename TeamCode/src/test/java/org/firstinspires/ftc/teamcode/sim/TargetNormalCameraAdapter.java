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
 * TEST-ONLY target-basis adapter for the SimLoop camera.
 * <p>Physical sticker orientation (always applied): SimLoop alpha.2 draws each tag upright, so its
 * tag +X points camera-right. The field stickers sit on the cell underside with their bottom edge
 * toward field center, so the robot sees them upside down: the physical 2026-11-05 frame has tag +X
 * camera-left (edge azimuth about +93 deg). Rz(pi) in target coordinates models that in-plane flip;
 * tag +Z (into the print) is unchanged, matching both SimLoop and the physical camera.
 * <p>Inverted normal (optional): Rx(pi) on top gives the robot an outward +Z before its own
 * conversion, so every OpMode must reject it with a facing diagnostic.
 * <p>Projection, visibility, measured position, latency, staleness, dropouts and turret motion
 * still come from the upstream fake.
 */
final class TargetNormalCameraAdapter extends Limelight3A implements FakeDevice {
    private final FakeLimelight3A source;
    final boolean invertedNormal;
    TargetNormalCameraAdapter(FakeLimelight3A source, boolean invertedNormal) {
        super(null, "limelight", InetAddress.getLoopbackAddress());
        super.stop();
        this.source = source;
        this.invertedNormal = invertedNormal;
    }
    @Override public void update(double dt) { source.update(dt); }
    @Override public void start() { source.start(); }
    @Override public void stop() { if (source != null) source.stop(); }
    @Override public void pause() { source.pause(); }
    @Override public boolean isRunning() { return source.isRunning(); }
    @Override public boolean pipelineSwitch(int index) { return source.pipelineSwitch(index); }
    @Override public void setPollRateHz(int hz) { source.setPollRateHz(hz); }
    @Override public LLStatus getStatus() { return source.getStatus(); }
    @Override public String getConnectionInfo() { return "SimLoop inverted target-normal adapter (test only)"; }
    @Override public LLResult getLatestResult() {
        try { return new NormalizedResult(source.getLatestResult(), invertedNormal); }
        catch (org.json.JSONException e) { throw new IllegalStateException(e); }
    }

    private static final class NormalizedResult extends LLResult {
        final LLResult raw;
        final boolean invertedNormal;
        NormalizedResult(LLResult raw, boolean invertedNormal) throws org.json.JSONException {
            super(new JSONObject()); this.raw = raw; this.invertedNormal = invertedNormal;
        }
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
                    return pose == null ? null : new Pose3D(pose.getPosition(), rebase(pose.getOrientation(), invertedNormal));
                }
            });
            return tags;
        }
    }

    /** R' = R * Rz(pi) [* Rx(pi)], with R = Rz(yaw) Ry(pitch) Rx(roll) as the robot reads SDK angles. */
    static YawPitchRollAngles rebase(YawPitchRollAngles o, boolean invertedNormal) {
        double y = o.getYaw(AngleUnit.RADIANS), p = o.getPitch(AngleUnit.RADIANS), r = o.getRoll(AngleUnit.RADIANS);
        double cy = Math.cos(y), sy = Math.sin(y), cp = Math.cos(p), sp = Math.sin(p), cr = Math.cos(r), sr = Math.sin(r);
        double[][] m = {
                {cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr},
                {sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr},
                {-sp, cp*sr, cp*cr}};
        // Rz(pi): negate tag X and Y columns. Rx(pi): negate tag Y and Z columns.
        double[] sign = invertedNormal ? new double[]{-1, 1, -1} : new double[]{-1, -1, 1};
        for (int row = 0; row < 3; row++) for (int col = 0; col < 3; col++) m[row][col] *= sign[col];
        double pitch = Math.asin(Math.max(-1, Math.min(1, -m[2][0])));
        return new YawPitchRollAngles(AngleUnit.RADIANS, Math.atan2(m[1][0], m[0][0]), pitch,
                Math.atan2(m[2][1], m[2][2]), 0);
    }
}
