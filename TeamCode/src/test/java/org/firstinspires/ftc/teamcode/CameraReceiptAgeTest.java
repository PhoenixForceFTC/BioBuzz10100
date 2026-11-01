package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import static org.junit.Assert.*;

public class CameraReceiptAgeTest {
    @Test public void receiptAgeIncludesCameraLatencyWithoutAnEpochAssumption() {
        assertEquals(9.945, PoseHistory.captureTimeFromAge(10, 20, 15, 20, 0), 1e-12);
        assertEquals(PoseHistory.captureTime(10, 1767225600020L, 1767225600000L, 15, 20, 0),
                PoseHistory.captureTimeFromAge(10, 20, 15, 20, 0), 1e-12);
    }
    @Test public void negativeAndNonfiniteReceiptAgesStayInvalid() {
        assertTrue(Double.isNaN(PoseHistory.captureTimeFromAge(10, -1, 15, 20, 0)));
        assertTrue(Double.isNaN(PoseHistory.captureTimeFromAge(10, Double.NaN, 15, 20, 0)));
        assertTrue(Double.isNaN(PoseHistory.captureTimeFromAge(10, 10, -1, 20, 0)));
    }
}
