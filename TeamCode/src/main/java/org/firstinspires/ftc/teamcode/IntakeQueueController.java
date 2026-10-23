package org.firstinspires.ftc.teamcode;

/** Bumpers always win. A newly full queue reverses intake until the GP1 auto trigger is released. */
public final class IntakeQueueController {
    private boolean previousFull, previousAuto, reverse;
    public double update(boolean full, boolean autoHeld, boolean forwardBumper, boolean reverseBumper) {
        if (full && !previousFull) reverse = true;
        if (previousAuto && !autoHeld) reverse = false;
        previousFull = full; previousAuto = autoHeld;
        if (forwardBumper || reverseBumper) return (forwardBumper ? 1 : 0) - (reverseBumper ? 1 : 0);
        return reverse ? -1 : 0;
    }
}
