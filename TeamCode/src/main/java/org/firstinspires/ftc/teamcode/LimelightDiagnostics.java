package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Shared read-only diagnostics for cached camera observations; never polls hardware or flushes telemetry. */
public final class LimelightDiagnostics {
    private LimelightDiagnostics() { }

    public static void addTelemetry(Telemetry telemetry, LimelightObservation observation, double now) {
        if (observation == null) {
            telemetry.addData("Camera frame", "Not sampled");
            return;
        }
        telemetry.addData("Camera frame", observation.rejectionReason == null
                ? "Valid detection payload" : observation.rejectionReason);
        telemetry.addData("Camera capture age (s)", "%.3f", now-observation.captureTime);
        // Hub wait is SDK staleness (polled every 10 ms); capture and processing are reported by the Limelight.
        telemetry.addData("Camera delay (ms)", "hub %.0f + capture %.0f + processing %.0f | new frames %.0f/s",
                observation.hubWaitMs, observation.captureLatencyMs, observation.processingLatencyMs,
                observation.newFrameRateHz);
        int usable = 0;
        for (TagObservation tag : observation.tags) if (tag.usable()) usable++;
        telemetry.addData("Usable metric poses", "%d / %d (not field localization)", usable, observation.tags.size());
        for (TagObservation tag : observation.tags) {
            telemetry.addData("Metric tag " + tag.id,
                    "right %.3f down %.3f forward %.3f m | area %.3f%% | usable %s",
                    tag.x,tag.y,tag.z,tag.area,tag.usable());
            telemetry.addData("Tag " + tag.id + " R/P/Y (deg), facing",
                    "%.1f / %.1f / %.1f | %.3f (> %.2f) | %s",
                    Math.toDegrees(tag.roll),Math.toDegrees(tag.pitch),Math.toDegrees(tag.yaw),
                    tag.facingCosine(),TagObservation.MIN_FACING_COSINE,
                    tag.usable()?"Accepted metric pose":tag.rejectionReason());
        }
    }
}
