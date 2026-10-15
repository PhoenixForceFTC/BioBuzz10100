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
 *   driver.moveTo(20, 0, 0);                         // default speed
 *   driver.moveTo(20, 20, 90, 0.5);                  // half speed
 *   driver.turn(45, AutoDriver.Direction.LEFT);      // rotate 45 deg left, in place
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

    public enum Direction { LEFT, RIGHT }

    /** A turn counts as finished once the heading is within this many degrees of the target. */
    private static final double TURN_TOLERANCE_DEGREES = 2.0;

    private static class Move {
        final Pose target;          // for moveTo moves
        final double speed;         // for moveTo moves
        final boolean isTurn;
        final double turnDegrees;   // for turns: positive = left (counter-clockwise)

        private Move(Pose target, double speed, boolean isTurn, double turnDegrees) {
            this.target = target;
            this.speed = speed;
            this.isTurn = isTurn;
            this.turnDegrees = turnDegrees;
        }

        static Move moveTo(Pose target, double speed) {
            return new Move(target, speed, false, 0);
        }

        static Move turn(double signedDegrees) {
            return new Move(null, 1.0, true, signedDegrees);
        }
    }

    private final Follower follower;
    private final PoseFactory p = PoseFactory.degrees();
    private final Queue<Move> queue = new ArrayDeque<>();

    private Mode mode = Mode.AUTOMATIC;
    private double defaultSpeed = 1.0;

    private boolean moving = false;
    private boolean currentIsTurn = false;
    private double currentTurnHeading = 0; // target heading of the running turn

    public AutoDriver(Follower follower) {
        this.follower = follower;
    }

    /** Call once per frame. Also runs follower.update(). */
    public void update() {
        follower.update();

        if (moving && isCurrentDone()) {
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
        queue.add(Move.moveTo(p.of(x, y, rotationDegrees), clamp(speed)));
        return this;
    }

    /**
     * Queue an in-place turn, relative to the robot's heading at the moment the
     * turn starts. LEFT is counter-clockwise, RIGHT is clockwise.
     */
    public AutoDriver turn(double degrees, Direction direction) {
        double signed = Math.abs(degrees) * (direction == Direction.LEFT ? 1 : -1);
        queue.add(Move.turn(signed));
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

        if (move.isTurn) {
            startTurn(move);
        } else {
            startMove(move);
        }
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

    // ------------------------------------------------------------------

    private void startMove(Move move) {
        currentIsTurn = false;

        // Constant heading: the robot rotates toward the target heading while it drives.
        Path path = line(follower.pose(), move.target).constant(move.target.heading());

        if (move.speed < 1.0) {
            // Per-path speed cap in Pedro 3. Assumes Constants exposes the ForesightConfig.
            path = path.with(Constants.foresightConfig.maxPathSpeed.at(move.speed));
        }

        follower.follow(path);
    }

    private void startTurn(Move move) {
        currentIsTurn = true;

        // Same x/y, heading rotated relative to where the robot is right now.
        Pose now = follower.pose();
        double targetDegrees = Math.toDegrees(now.heading()) + move.turnDegrees;
        Pose target = p.of(now.x(), now.y(), targetDegrees);
        currentTurnHeading = target.heading();

        follower.hold(target);
    }

    private boolean isCurrentDone() {
        if (currentIsTurn) {
            // Judged by heading error rather than isBusy(), since a hold can report
            // not-busy before the turn has actually finished.
            double error = currentTurnHeading - follower.pose().heading();
            error = Math.atan2(Math.sin(error), Math.cos(error)); // wrap to [-pi, pi]
            return Math.abs(error) < Math.toRadians(TURN_TOLERANCE_DEGREES);
        }
        return !follower.isBusy();
    }

    /** Current robot pose on the field, as tracked by the follower. Safe to call any time. */
    public Pose getPose() {
        return follower.pose();
    }

    private static double clamp(double speed) {
        return Math.max(0.0, Math.min(1.0, speed));
    }
}