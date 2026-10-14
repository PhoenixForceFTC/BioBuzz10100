package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import java.util.ArrayDeque;
import java.util.Queue;

import static com.pedropathing.api.Paths.*;

/**
 * Queue of moves for a Pedro Pathing Follower.
 *
 *   driver.moveTo(20, 0, 0);
 *   driver.moveTo(20, 20, 90);
 *   // every loop(): driver.update();
 *
 * AUTOMATIC: starts the next queued move as soon as the current one finishes.
 * MANUAL:    queued moves only start when you call next().
 *
 * Positions are field coordinates in inches, rotation is in degrees.
 */
public class AutoDriver {

    public enum Mode { AUTOMATIC, MANUAL }

    private final Follower follower;
    private final PoseFactory p = PoseFactory.degrees();
    private final Queue<Pose> queue = new ArrayDeque<>();

    private Mode mode = Mode.AUTOMATIC;
    private boolean moving = false;

    public AutoDriver(Follower follower) {
        this.follower = follower;
    }

    /** Call once per frame. Also runs follower.update(). */
    public void update() {
        follower.update();

        if (moving && !follower.isBusy()) {
            moving = false;
        }

        if (!moving && mode == Mode.AUTOMATIC) {
            next();
        }
    }

    /** Queue a move to field position (x, y) ending at the given rotation. */
    public AutoDriver moveTo(double x, double y, double rotationDegrees) {
        queue.add(p.of(x, y, rotationDegrees));
        return this;
    }

    /**
     * Start the next queued move now. If a move is already running it is
     * replaced. Returns false if the queue was empty.
     */
    public boolean next() {
        Pose target = queue.poll();
        if (target == null) return false;

        // Constant heading: the robot rotates toward the target heading while it drives.
        follower.follow(line(follower.pose(), target).constant(target.heading()));
        moving = true;
        return true;
    }

    public AutoDriver setMode(Mode mode) {
        this.mode = mode;
        return this;
    }

    /** Stop the current move only. In AUTOMATIC mode the next one then starts. */
    public AutoDriver stopCurrent() {
        if (moving) {
            // Pedro 3 renamed breakFollowing() to stop().
            follower.stop();
            moving = false;
        }
        return this;
    }

    /** Stop the current move and clear the queue. */
    public AutoDriver stopAll() {
        queue.clear();
        return stopCurrent();
    }

    /** True if a move is running or any are queued. */
    public boolean isBusy() {
        return moving || !queue.isEmpty();
    }
}