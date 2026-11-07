package org.firstinspires.ftc.teamcode;

/** Automatic queue-based intake direction; either bumper overrides and opposing bumpers stop. */
public final class IntakeQueueController {
    private final double fullQueuePower;

    /** Preserve the existing reverse-on-full policy for autonomous callers. */
    public IntakeQueueController() { this(-1.0); }

    public IntakeQueueController(double fullQueuePower) {
        if (!Double.isFinite(fullQueuePower) || fullQueuePower < -1 || fullQueuePower > 1)
            throw new IllegalArgumentException("Invalid full-queue intake power");
        this.fullQueuePower = fullQueuePower;
    }

    public double update(boolean full, boolean forwardBumper, boolean reverseBumper) {
        return update(full, forwardBumper, reverseBumper, false);
    }

    /** Pause ordinary collection for firing/inhibit; explicit direction overrides remain available. */
    public double update(boolean full, boolean forwardBumper, boolean reverseBumper, boolean pauseAutomatic) {
        if (forwardBumper || reverseBumper) return (forwardBumper ? 1 : 0) - (reverseBumper ? 1 : 0);
        return pauseAutomatic ? 0 : full ? fullQueuePower : 1;
    }
}
