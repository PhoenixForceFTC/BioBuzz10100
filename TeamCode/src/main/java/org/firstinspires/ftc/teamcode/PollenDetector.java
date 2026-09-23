package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

/** OpenCV yellow-pollen blob detector for the downward-facing C270. */
final class PollenDetector implements VisionProcessor {
    // Hive Vision's YOLO-derived yellow-pollen HSV model (OpenCV H 0..179),
    // https://github.com/sidhuharjas/Hive-Vision/blob/main/cv/tools/hsv_tuned.json
    // This Control Hub path runs the model-derived color gates, not YOLO weights.
    // Tune these values using actual C270 footage and match lighting.
    private static final Scalar HSV_MIN = new Scalar(9, 90, 45);
    private static final Scalar HSV_MAX = new Scalar(33, 255, 238);
    private static final double MIN_AREA = 70;
    private static final double MAX_AREA = 15000;
    private static final double MIN_CIRCULARITY = 0.42;
    private static final double MAX_ASPECT = 1.5;
    private static final double MIN_FILL = 0.5;

    static final class Detection {
        final double x;
        final double y;
        final double area;
        final long frameNumber;
        final long observedNanos;

        Detection(double x, double y, double area, long frameNumber) {
            this.x = x;
            this.y = y;
            this.area = area;
            this.frameNumber = frameNumber;
            this.observedNanos = System.nanoTime();
        }
    }

    private Mat hsv;
    private Mat mask;
    private Mat hierarchy;
    private Mat kernel;
    private final Paint paint = new Paint();
    private volatile Detection latest;
    private long frames;
    private int width;
    private int height;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        this.width = width;
        this.height = height;
        hsv = new Mat();
        mask = new Mat();
        hierarchy = new Mat();
        kernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE,
                new org.opencv.core.Size(5, 5));
        paint.setColor(Color.YELLOW);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(3);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        frames++;
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, HSV_MIN, HSV_MAX, mask);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL,
                Imgproc.CHAIN_APPROX_SIMPLE);
        Detection best = null;
        double bestScore = Double.NEGATIVE_INFINITY;
        for (MatOfPoint contour : contours) {
            try {
                double area = Imgproc.contourArea(contour);
                if (area < MIN_AREA || area > MAX_AREA) continue;
                Rect box = Imgproc.boundingRect(contour);
                double aspect = Math.max(box.width, box.height)
                        / (double) Math.max(1, Math.min(box.width, box.height));
                if (aspect > MAX_ASPECT) continue;
                if (area / Math.max(1.0, box.width * (double) box.height) < MIN_FILL) {
                    continue;
                }
                MatOfPoint2f curve = new MatOfPoint2f(contour.toArray());
                double perimeter = Imgproc.arcLength(curve, true);
                curve.release();
                if (perimeter <= 0) continue;
                double circularity = 4 * Math.PI * area / (perimeter * perimeter);
                if (circularity < MIN_CIRCULARITY) continue;
                Moments moments = Imgproc.moments(contour);
                if (moments.get_m00() <= 0) continue;
                double x = moments.get_m10() / moments.get_m00() / width;
                double y = moments.get_m01() / moments.get_m00() / height;
                if (x < 0.04 || x > 0.96 || y < 0.05 || y > 0.97) continue;
                // Favor reachable pollen near the intake, then central targets.
                double score = y * 0.55 + (1 - Math.abs(x - 0.5) * 2) * 0.30
                        + Math.min(area / 2500, 1) * 0.15;
                if (score > bestScore) {
                    bestScore = score;
                    best = new Detection(x, y, area, frames);
                }
            } finally {
                contour.release();
            }
        }
        latest = best;
        return best;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {
        if (userContext instanceof Detection) {
            Detection target = (Detection) userContext;
            canvas.drawCircle((float) (target.x * onscreenWidth),
                    (float) (target.y * onscreenHeight), 14 * scaleCanvasDensity, paint);
        }
    }

    Detection getLatest() {
        return latest;
    }

    void close() {
        if (hsv != null) hsv.release();
        if (mask != null) mask.release();
        if (hierarchy != null) hierarchy.release();
        if (kernel != null) kernel.release();
    }
}
