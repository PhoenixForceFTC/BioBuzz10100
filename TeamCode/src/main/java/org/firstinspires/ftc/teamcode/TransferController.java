package org.firstinspires.ftc.teamcode;

/** Deterministic full confirmation and two-stage feed/loading priority. */
public final class TransferController {
    private double highSince = Double.NaN;
    private boolean full;
    private double stage1, stage2;
    public void update(double now, boolean sensorHigh, boolean feed, boolean pause) {
        if (sensorHigh) {
            if (Double.isNaN(highSince)) highSince = now;
            full = now - highSince >= 2;
        } else { highSince = Double.NaN; full = false; }
        stage1 = feed || (!pause && !full) ? 1 : 0;
        stage2 = feed ? 1 : 0;
    }
    public void stop() { stage1 = stage2 = 0; highSince = Double.NaN; full = false; }
    public boolean full() { return full; }
    public double stage1() { return stage1; }
    public double stage2() { return stage2; }
    public double confirmingSeconds(double now) { return Double.isNaN(highSince) ? 0 : Math.max(0, now - highSince); }
}
