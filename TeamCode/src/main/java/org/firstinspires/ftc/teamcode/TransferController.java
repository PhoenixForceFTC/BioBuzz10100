package org.firstinspires.ftc.teamcode;

/** Deterministic full confirmation and two-stage feed/loading priority. */
public final class TransferController {
    private final double power, confirmationSeconds;
    public TransferController() { this(1, 2); }
    public TransferController(double power, double confirmationSeconds) {
        if (!RobotMath.finite(power, confirmationSeconds) || power <= 0 || power > 1 || confirmationSeconds <= 0)
            throw new IllegalArgumentException("Invalid transfer configuration");
        this.power = power; this.confirmationSeconds = confirmationSeconds;
    }
    private double highSince = Double.NaN;
    private boolean full, holdingFullQueue;
    private double stage1, stage2;
    public void update(double now, boolean sensorHigh, boolean feed, boolean pause) {
        if (sensorHigh) {
            if (Double.isNaN(highSince)) highSince = now;
            full = now - highSince >= confirmationSeconds;
        } else { highSince = Double.NaN; full = false; }
        // Once four balls are confirmed, sensor chatter must not restart loading before a shot.
        if (feed) holdingFullQueue = false;
        else if (full) holdingFullQueue = true;
        stage1 = feed || (!pause && !full()) ? power : 0;
        // Stage 2 touches the top ball: keep it stopped until gate-open shot feeding.
        stage2 = feed ? power : 0;
    }
    public void stop() { stage1 = stage2 = 0; highSince = Double.NaN; full = holdingFullQueue = false; }
    public boolean full() { return full || holdingFullQueue; }
    public double stage1() { return stage1; }
    public double stage2() { return stage2; }
    public double confirmingSeconds(double now) { return Double.isNaN(highSince) ? 0 : Math.max(0, now - highSince); }
}
