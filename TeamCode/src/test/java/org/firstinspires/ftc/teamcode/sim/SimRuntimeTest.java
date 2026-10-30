package org.firstinspires.ftc.teamcode.sim;

import org.json.JSONObject;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertEquals;

class SimRuntimeTest {
    @Test void limelightHasRealJsonRatherThanDefaultReturningAndroidStub() throws Exception {
        JSONObject result = new JSONObject("{\"Results\":{\"v\":1,\"Fiducial\":[{\"fID\":30}]}}");
        assertEquals(1, result.getJSONObject("Results").getInt("v"));
        assertEquals(30, result.getJSONObject("Results").getJSONArray("Fiducial").getJSONObject(0).getInt("fID"));
    }
}
