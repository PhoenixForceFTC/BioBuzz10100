package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import java.util.ArrayDeque;
import java.util.Queue;

import static com.pedropathing.api.Paths.*;

/**
 * Queue of moves for a Pedro Pathing Follower.
 *
 *   driver.moveTo(20, 0, 0);           // default speed
 *   driver.moveTo(20, 20, 90, 0.5);    // half speed
 *   // every loop(): driver.update();
 *
 * AUTOMATIC: starts the next queued move as soon as the current one finishes.
 * MANUAL:    queued moves only start when you call next().
 *
 * Positions are field coordinates in inches, rotation is in degrees.
 * Speed is a proportion (0 to 1) of the robot's max achievable path speed.
 */
public class AutoDriver {

    public enum Mode { AUTOMATIC, MANUAL }

    private static class Move {
        final Pose target;
        final double speed;

        Move(Pose target, double speed) {
            this.target = target;
            this.speed = speed;
        }
    }

    private final Follower follower;
    private final PoseFactory p = PoseFactory.degrees();
    private final Queue<Move> queue = new ArrayDeque<>();

    private Mode mode = Mode.AUTOMATIC;
    private double defaultSpeed = 1.0;
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

    /** Queue a move to field position (x, y) ending at the given rotation, at the default speed. */
    public AutoDriver moveTo(double x, double y, double rotationDegrees) {
        return moveTo(x, y, rotationDegrees, defaultSpeed);
    }

    /** Queue a move at a specific speed (0 to 1, proportion of max path speed). */
    public AutoDriver moveTo(double x, double y, double rotationDegrees, double speed) {
        queue.add(new Move(p.of(x, y, rotationDegrees), clamp(speed)));
        return this;
    }

    /** Speed used by moveTo(x, y, rotation) for moves queued after this call. */
    public AutoDriver setDefaultSpeed(double speed) {
        this.defaultSpeed = clamp(speed);
        return this;
    }

    /**
     * Start the next queued move now. If a move is already running it is
     * replaced. Returns false if the queue was empty.
     */
    public boolean next() {
        Move move = queue.poll();
        if (move == null) return false;

        // Constant heading: the robot rotates toward the target heading while it drives.
        Path path = line(follower.pose(), move.target).constant(move.target.heading());

        if (move.speed < 1.0) {
            // Per-path speed cap in Pedro 3. Assumes Constants exposes the ForesightConfig.
            path = path.with(Constants.foresightConfig.maxPathSpeed.at(move.speed));
        }

        follower.follow(path);
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

    private static double clamp(double speed) {
        return Math.max(0.0, Math.min(1.0, speed));
    }
}