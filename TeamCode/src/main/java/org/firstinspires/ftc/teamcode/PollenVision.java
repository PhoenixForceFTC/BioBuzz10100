package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.opencv.core.Scalar;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

/** Layer 1: Logitech C270 pollen tracking, independent of the turret's AprilTag Limelight. */
public class PollenVision implements AutoCloseable {
    public static final String HW_WEBCAM = "webcam_1";
    public static final int IMAGE_WIDTH_PX = 320;
    public static final int IMAGE_HEIGHT_PX = 240;
    public static final PollenConfiguration CONFIG = PollenConfiguration.diagnostic();

    private static final class FrameObservation {
        final long captureTimeNanos;
        final PollenTarget target;

        FrameObservation(long captureTimeNanos, PollenTarget target) {
            this.captureTimeNanos = captureTimeNanos;
            this.target = target;
        }
    }

    private final PollenConfiguration config;
    private final FrameProcessor processor;
    private final VisionPortal portal;

    public PollenVision(HardwareMap hardwareMap) { this(hardwareMap, CONFIG); }
    public PollenVision(HardwareMap hardwareMap, PollenConfiguration config) {
        this.config = config; processor = new FrameProcessor(config);
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, HW_WEBCAM))
                .setCameraResolution(new Size(IMAGE_WIDTH_PX, IMAGE_HEIGHT_PX))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(processor)
                .build();
    }

    public boolean isAutonomousReady() { return isReady() && config.autonomousEligible(processor.sdkCalibration) && processor.model != null && processor.model.pickupVisible(config); }
    public PollenConfiguration configuration() { return config; }
    public boolean isReady() {
        long frameTimeNanos = processor.observation.captureTimeNanos;
        return portal.getCameraState() == VisionPortal.CameraState.STREAMING
                && isFresh(frameTimeNanos, System.nanoTime());
    }

    public PollenTarget getTarget() {
        PollenTarget target = processor.observation.target;
        return target != null && isReady() && isFresh(target.captureTimeNanos, System.nanoTime())
                ? target : null;
    }

    private static boolean isFresh(long captureTimeNanos, long nowNanos) {
        return PollenTracker.fresh(captureTimeNanos, nowNanos);
    }

    public void addTelemetry(Telemetry telemetry) {
        long frameTimeNanos = processor.observation.captureTimeNanos;
        long nowNanos = System.nanoTime();
        PollenTarget target = getTarget();
        telemetry.addData("Pollen Camera", "%s | %.1f FPS", portal.getCameraState(), portal.getFps());
        telemetry.addData("Pollen Model", config.status(processor.sdkCalibration));
        telemetry.addData("Pollen Autonomous Eligible", isAutonomousReady());
        if (processor.model != null) {
            telemetry.addData("Visible pickup floor region (mm)", "near %.0f | far %.0f | %s",
                    processor.model.nearestPickupVisibleMm(config),processor.model.farthestPickupVisibleMm(config),processor.model.distortionApplied() ? "undistorted" : "no distortion calibration");
            telemetry.addData("Pickup transition / blind travel (mm)", "%.0f / %.0f | %s",config.pickupStartMm,config.pickupTravelMm,
                    processor.model.pickupVisible(config) ? "visible transition" : "INVALID: transition outside visible region");
        }
        telemetry.addData("Pollen Rejection", processor.rejection);
        telemetry.addData("Pollen Intrinsics (px)", "fx %.1f fy %.1f cx %.1f cy %.1f", processor.fxPx, processor.fyPx, processor.cxPx, processor.cyPx);
        telemetry.addData("Pollen Frame Age (ms)", frameTimeNanos == 0 ? -1.0
                : (nowNanos - frameTimeNanos) / 1e6);
        telemetry.addData("Pollen Target", target == null ? "NONE / STALE" : "TRACKING");
        if (target != null) {
            telemetry.addData("Calibration contact (px)", "%.1f, %.1f", target.contactXPx, target.contactYPx);
            telemetry.addData("Pollen Bearing (deg)", "%+.1f (right positive)", target.bearingDeg);
            telemetry.addData("Pollen Floor Estimate (mm)", "forward %.0f | right %+.0f", target.forwardMm, target.rightMm);
            telemetry.addData("Pollen Blob", "%.0f px area | center (%.0f, %.0f) px",
                    target.areaPx, target.centerXPx, target.centerYPx);
        }
    }

    @Override
    public void close() {
        portal.close();
    }

    /** Runs on the camera thread; publishes immutable observations with the SDK capture timestamp. */
    private static class FrameProcessor implements VisionProcessor {
        private final PollenConfiguration config;
        private final PollenTracker tracker;
        private volatile String rejection = "No frames";
        private final ColorBlobLocatorProcessor blobs;
        FrameProcessor(PollenConfiguration config) {
            this.config = config; tracker = new PollenTracker(config);
            blobs = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(new ColorRange(ColorSpace.HSV, new Scalar(config.hueMin, config.saturationMin, config.valueMin), new Scalar(config.hueMax, 255, 255)))
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setBlurSize(5)
                .setDrawContours(true)
                .build();
        }
        private volatile FrameObservation observation = new FrameObservation(0, null);
        private volatile boolean sdkCalibration;
        private volatile PollenCameraModel model;
        private double fxPx;
        private double fyPx;
        private double cxPx;
        private double cyPx;
        private int widthPx;
        private int heightPx;

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            blobs.init(width, height, calibration);
            widthPx = width;
            heightPx = height;
            sdkCalibration = calibration != null && !calibration.isFake() && !calibration.isDegenerate();
            fxPx = config.intrinsicsMeasured ? config.fx : sdkCalibration ? calibration.focalLengthX
                    : width / (2.0 * Math.tan(Math.toRadians(config.horizontalFovDeg / 2.0)));
            fyPx = config.intrinsicsMeasured ? config.fy : sdkCalibration ? calibration.focalLengthY : fxPx;
            cxPx = config.intrinsicsMeasured ? config.cx : sdkCalibration ? calibration.principalPointX : width / 2.0;
            cyPx = config.intrinsicsMeasured ? config.cy : sdkCalibration ? calibration.principalPointY : height / 2.0;
            double[] distortion = config.distortionCoefficients();
            if (!config.intrinsicsMeasured && sdkCalibration) {
                distortion = new double[calibration.distortionCoefficients.length];
                for (int i=0;i<distortion.length;i++) distortion[i]=calibration.distortionCoefficients[i];
            }
            model = new PollenCameraModel(width,height,fxPx,fyPx,cxPx,cyPx,distortion);
            if (config.autonomousEligible(sdkCalibration) && !model.pickupVisible(config))
                rejection = "Pickup transition outside active visible floor region: autonomous disabled";

        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Object drawContext = blobs.processFrame(frame, captureTimeNanos);
            List<PollenTracker.Detection> detections = new ArrayList<>();
            for (ColorBlobLocatorProcessor.Blob blob : blobs.getBlobs()) {
                RotatedRect box = blob.getBoxFit();
                Point contact = null;
                boolean clipped = false;
                for (Point point : blob.getContourPoints()) {
                    if (point.x <= 1 || point.x >= widthPx - 2 || point.y <= 1 || point.y >= heightPx - 2) {
                        clipped = true;
                    }
                    if (contact == null || point.y > contact.y) {
                        contact = point;
                    }
                }
                if (contact != null) detections.add(new PollenTracker.Detection(box.center.x, box.center.y,
                        contact.x, contact.y, blob.getContourArea(), blob.getDensity(), blob.getAspectRatio(), blob.getCircularity(), clipped));
            }
            PollenTarget best = tracker.select(captureTimeNanos, detections, model);
            rejection = tracker.rejection();
            // Publish target and timestamp together; never reuse an old detection for a new frame.
            observation = new FrameObservation(captureTimeNanos, best);
            return drawContext;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int width, int height, float scaleBmpPxToCanvasPx,
                                float scaleCanvasDensity, Object userContext) {
            blobs.onDrawFrame(canvas, width, height, scaleBmpPxToCanvasPx, scaleCanvasDensity, userContext);
        }
    }
}
