package org.firstinspires.ftc.teamcode;

/** Shared Red/Blue mode selection and non-blocking transfer feed timing. */
public final class CompetitionFeedController {
    // Provisional competition values; replace with the team's final loaded timing measurements.
    public static final double GATE_TO_FEED_DELAY_SECONDS = 0.10;
    public static final double FEED_WINDOW_SECONDS = 0.30;
    public static final double RECOVERY_SECONDS = 0.10;
    public static final double READY_STABLE_SECONDS = ShootingConfiguration.READY_STABLE_SECONDS;
    private boolean continuous = true;
    private boolean previousToggle;
    private double gateOpenedAt = Double.NaN;

    public static FiringCoordinator firing() {
        FiringCoordinator firing = new FiringCoordinator(GATE_TO_FEED_DELAY_SECONDS,
                ShootingConfiguration.FEED_TO_EXIT_SECONDS, FEED_WINDOW_SECONDS,
                RECOVERY_SECONDS, READY_STABLE_SECONDS);
        firing.setImmediateContinuousFeed(true);
        return firing;
    }

    /** Call before updating firing. A mode change starts a fresh gate/readiness sequence. */
    public void updateSelection(double now, boolean toggle, FiringCoordinator firing) {
        if (toggle && !previousToggle) continuous = !continuous;
        previousToggle = toggle;
        if (firing.configureFeedMode(now, continuous, FEED_WINDOW_SECONDS,
                RECOVERY_SECONDS, READY_STABLE_SECONDS, true)) gateOpenedAt = Double.NaN;
    }

    /** Call after updating firing, including idle/inhibited loops so the next shot has a fresh timer. */
    public boolean feedStage1(double now, FiringCoordinator firing) {
        if (!firing.gateOpen()) gateOpenedAt = Double.NaN;
        else if (Double.isNaN(gateOpenedAt)) gateOpenedAt = now;
        // Both competition stages start after the same gate-open delay.
        return firing.recoveringBetweenTimedFeeds() || firing.feeding() && (continuous
                || now - gateOpenedAt >= firing.gateTravelSeconds);
    }

    /** Continue both stages through timed recovery and readiness, until the next opening. */
    public boolean feedTransfers(FiringCoordinator firing, boolean gateReady) {
        return firing.recoveringBetweenTimedFeeds()
                || firing.feeding() && (continuous || gateReady);
    }

    public boolean continuous() { return continuous; }
}
