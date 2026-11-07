package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import org.json.JSONObject;
import org.junit.Test;
import static org.junit.Assert.*;

public class LimelightFrameRateTest {
    private static LLResult frame(double ts) throws Exception {
        return new LLResult(new JSONObject().put("ts", ts).put("v", 1)) { }; // Protected SDK constructor.
    }
    @Test public void countsDistinctFramesNotRepeatedPolls() throws Exception {
        LimelightObservation.FrameRate rate = new LimelightObservation.FrameRate();
        double hz = Double.NaN;
        // 40 camera frames per second, polled every 5 ms: each frame is seen about five times.
        for (int i = 0; i <= 400; i++) {
            double now = i * .005;
            hz = rate.update(frame(Math.floor(now * 40) / 40), now);
        }
        assertEquals(40, hz, 1.5);
        assertTrue(Double.isNaN(new LimelightObservation.FrameRate().update(null, 1)));
    }
}
