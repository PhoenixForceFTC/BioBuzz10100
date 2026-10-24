package org.firstinspires.ftc.teamcode;

/** Attempt every shutdown output even if one disconnected device throws. Preserve the first failure. */
public final class RobotShutdown {
    private RobotShutdown() { }
    public static void stopAll(Runnable... stops) {
        RuntimeException failure=null;
        for (Runnable stop : stops) {
            if (stop == null) continue;
            try { stop.run(); }
            catch (RuntimeException e) { if (failure == null) failure=e; else if (failure != e) failure.addSuppressed(e); }
        }
        if (failure != null) throw failure;
    }
}
