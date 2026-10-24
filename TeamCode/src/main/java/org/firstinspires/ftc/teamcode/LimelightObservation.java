package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import java.util.ArrayList;
import java.util.List;

/** SDK adapter. Reads only per-tag t6t_cs, never botpose or an assumed tag-height range. */
public final class LimelightObservation {
    public final double frame, captureTime;
    public final List<TagObservation> tags = new ArrayList<>();
    public LimelightObservation(double now, long wallNowMs, LLResult result) {
        frame = result == null ? Double.NaN : result.getTimestamp();
        captureTime = result == null ? Double.NaN : PoseHistory.captureTime(now, wallNowMs,
                result.getControlHubTimeStamp(), result.getCaptureLatency(), result.getTargetingLatency(),
                ShootingConfiguration.ADDITIONAL_VISION_TRANSPORT_SECONDS);
        if (result == null || !result.isValid() || result.getStaleness() < 0
                || result.getStaleness() > TurretSubsystem.LIMELIGHT_MAX_STALENESS_MS) return;
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            if (tag == null) continue;
            Pose3D pose = tag.getTargetPoseCameraSpace();
            if (pose == null) { tags.add(new TagObservation(tag.getFiducialId(),0,0,0,0,0,0,0)); continue; }
            Position p = pose.getPosition().toUnit(DistanceUnit.METER);
            tags.add(new TagObservation(tag.getFiducialId(), p.x, p.y, p.z,
                    pose.getOrientation().getRoll(AngleUnit.RADIANS), pose.getOrientation().getPitch(AngleUnit.RADIANS),
                    pose.getOrientation().getYaw(AngleUnit.RADIANS), tag.getTargetArea()));
        }
    }
}
