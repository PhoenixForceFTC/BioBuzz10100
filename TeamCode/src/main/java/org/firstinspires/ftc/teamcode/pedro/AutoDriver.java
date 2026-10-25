package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTurn;
import java.util.ArrayDeque;
import java.util.Queue;

import static com.pedropathing.api.Paths.*;

/**
 * Queue of moves for a Pedro Pathing Follower.
 *
 *   driver.moveTo(20, 0, 0);                          // default speed
 *   driver.moveTo(20, 20, 90, 0.5);                   // half speed
 *   driver.turn(45, AutoDriver.Direction.LEFT);       // rotate 45 deg left, in place
 *   driver.turn(359, AutoDriver.Direction.LEFT, 0.3); // slow full spin, one move
 *   // every loop(): driver.update();
 *
 * AUTOMATIC: starts the next queued move as soon as the current one finishes.
 * MANUAL:    queued moves only start when you call next().
 *
 * Positions are field coordinates in inches, rotation is in degrees.
 * Speed is a proportion (0 to 1) of full power.
 * Turns are run by procedures/ForesightTurn.
 */
public class AutoDriver {

    public enum Mode { AUTOMATIC, MANUAL }

    public enum Direction { LEFT, RIGHT }

    private static class Move {
        final Pose target;          // for moveTo moves
        final double speed;         // moveTo: path speed; turn: max turn power
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

        static Move turn(double signedDegrees, double speed) {
            return new Move(null, speed, true, signedDegrees);
        }
    }

    private final Follower follower;
    private final ForesightTurn turner;
    private final PoseFactory p = PoseFactory.degrees();
    private final Queue<Move> queue = new ArrayDeque<>();

    private Mode mode = Mode.AUTOMATIC;
    private double defaultSpeed = 1.0;
    private double defaultTurnSpeed = 1.0;

    private boolean moving = false;
    private boolean currentIsTurn = false;

    /**
     * Uses Constants.drivetrain and reads heading tuning from Constants.foresightConfig.
     * Call Constants.create(hardwareMap) first.
     */
    public AutoDriver(Follower follower) {
        this.follower = follower;
        this.turner = new ForesightTurn(follower);
    }

    /**
     * Explicit heading tuning, for experimenting without editing Constants.
     * @param headingKP             heading gain (power per radian of error)
     * @param headingBrakeLinear    linear braking coefficient
     * @param headingBrakeQuadratic quadratic braking coefficient
     */
//    public AutoDriver(Follower follower,
//                      double headingKP, double headingBrakeLinear, double headingBrakeQuadratic) {
//        this.follower = follower;
//        this.turner = new ForesightTurn(follower,
//                headingKP, headingBrakeLinear, headingBrakeQuadratic);
//    }

    /** Call once per frame. Also runs follower.update(). */
    public void update() {
        follower.update();

        if (moving && currentIsTurn) {
            turner.update();
        }

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

    /** Queue an in-place turn at the default turn speed. */
    public AutoDriver turn(double degrees, Direction direction) {
        return turn(degrees, direction, defaultTurnSpeed);
    }

    /**
     * Queue an in-place turn, relative to the robot's heading at the moment the
     * turn starts. LEFT is counter-clockwise, RIGHT is clockwise. Any size works,
     * including over 180 degrees. Speed is the maximum turning power (0.1 to 1).
     */
    public AutoDriver turn(double degrees, Direction direction, double speed) {
        double signed = Math.abs(degrees) * (direction == Direction.LEFT ? 1 : -1);
        queue.add(Move.turn(signed, clampTurn(speed)));
        return this;
    }

    /** Speed used by moveTo(x, y, rotation) for moves queued after this call. */
    public AutoDriver setDefaultSpeed(double speed) {
        this.defaultSpeed = clamp(speed);
        return this;
    }

    /** Speed used by turn(degrees, direction) for turns queued after this call. */
    public AutoDriver setDefaultTurnSpeed(double speed) {
        this.defaultTurnSpeed = clampTurn(speed);
        return this;
    }

    /**
     * Start the next queued move now. If a move is already running it is
     * replaced. Returns false if the queue was empty.
     */
    public boolean next() {
        Move move = queue.poll();
        if (move == null) return false;

        if (moving && currentIsTurn) {
            turner.stop();
        }

        if (move.isTurn) {
            currentIsTurn = true;
            turner.start(move.turnDegrees, move.speed);
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
            if (currentIsTurn) {
                turner.stop();
            }
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

    /** Current robot pose on the field, as tracked by the follower. Safe to call any time. */
    public Pose getPose() {
        return follower.pose();
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

    private boolean isCurrentDone() {
        if (currentIsTurn) {
            return turner.isDone();
        }
        return !follower.isBusy();
    }

    private static double clamp(double speed) {
        return Math.max(0.0, Math.min(1.0, speed));
    }

    private static double clampTurn(double speed) {
        return Math.max(ForesightTurn.MIN_SPEED, Math.min(1.0, speed));
    }
}