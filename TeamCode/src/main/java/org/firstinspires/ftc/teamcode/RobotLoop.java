package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.List;

/** OpMode-owned cache boundary. Subsystems must never clear caches or flush telemetry. */
public final class RobotLoop {
    private final List<LynxModule> hubs;
    private long previousStart, started;
    private double periodMs = Double.NaN, workMs = Double.NaN;
    public RobotLoop(HardwareMap map) {
        hubs = map.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }
    public void begin() {
        started = System.nanoTime();
        if (previousStart != 0) periodMs = (started-previousStart)/1e6;
        previousStart = started;
        for (LynxModule hub : hubs) hub.clearBulkCache();
    }
    public void addTelemetry(Telemetry telemetry) {
        if (started != 0) workMs = (System.nanoTime()-started)/1e6;
        telemetry.addData("Java loop (ms, Hz)", "period %.2f | %.1f Hz | work to telemetry %.2f ms",
                periodMs,periodMs > 0 ? 1000/periodMs : 0,workMs);
    }
}
