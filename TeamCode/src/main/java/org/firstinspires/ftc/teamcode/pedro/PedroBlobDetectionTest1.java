package org.firstinspires.ftc.teamcode.pedro;

import android.graphics.Color;
import android.util.Size;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Scalar;

import java.util.List;

@Autonomous(name = "blob detection 1")
public class PedroBlobDetectionTest1 extends OpMode {

    int dilateSize = 15; int erodeSize = 15; double circularity = 0.4;
    boolean ballFound = false;

    private Follower follower;
    private AutoDriver driver;

    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose = p.of(0, 0, 0);

    private ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(new ColorRange(ColorSpace.YCrCb, new Scalar( 120, 133, 15),
                    new Scalar(255, 160, 95)))   // Use a color match
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
            .setRoi(ImageRegion.asUnityCenterCoordinates(-1.0, 1.0, 1.0, -1.0))
            .setDrawContours(true)   // Show contours on the Stream Preview
            .setBoxFitColor(0)       // Disable the drawing of rectangles
            .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
            .setBlurSize(5)          // Smooth the transitions between different colors in image

            // the following options have been added to fill in perimeter holes.
            .setDilateSize(dilateSize)       // Expand blobs to fill any divots on the edges
            .setErodeSize(erodeSize)        // Shrink blobs back to original size
            .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)

            .build();

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

        driver = new AutoDriver(follower);

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        // AUTOMATIC is the default: each move starts when the last one finishes.
        driver.turn(359, AutoDriver.Direction.LEFT, 10);    // turn 360 deg

        // For MANUAL mode instead:
        // driver.setMode(AutoDriver.Mode.MANUAL);
        // ...then call driver.next() whenever you want the next move to start.
    }

    @Override
    public void loop() {
        driver.update();

        List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                50, 20000, blobs);  // filter out very small blobs.

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                circularity, 1, blobs);

        telemetry.addLine("Circularity Radius Center");

        telemetry.addLine("Current Heading: " + String.valueOf(driver.getPose().heading()));

        if(!ballFound){
            for(ColorBlobLocatorProcessor.Blob b : blobs){
                if (b.getCircle().getRadius() > 30){
                    driver.stopAll();
                    Pose currentPose = driver.getPose();

                    driver.moveTo(10*Math.cos(currentPose.heading()) + currentPose.x(),
                            10*Math.sin(currentPose.heading()) + currentPose.y(),
                            Math.toDegrees(currentPose.heading()));
                    ballFound = true;
                }
            }
        }

        // Display the Blob's circularity, and the size (radius) and center location of its circleFit.
        for (ColorBlobLocatorProcessor.Blob b : blobs) {

            Circle circleFit = b.getCircle();
            telemetry.addLine(String.format("%5.3f      %3d     (%3d,%3d)",
                    b.getCircularity(), (int) circleFit.getRadius(), (int) circleFit.getX(), (int) circleFit.getY()));
        }

        telemetry.addData("busy", driver.isBusy());
        telemetry.addData("x (in)", follower.pose().x());
        telemetry.addData("y (in)", follower.pose().y());
        telemetry.update();
    }
}