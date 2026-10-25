package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.utils.Angle;
import com.pedropathing.utils.Utils;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * In-place turn using Pedro's Foresight heading law (the same one ForesightTuner uses):
 *
 *   power = headingFeedback(remainingAngle - predictedBrakingAngle(omega))
 *   predictedBrakingAngle = linear * omega + quadratic * omega^2 * sign(omega)
 *
 * By default the heading feedback and braking coefficients are read at runtime from
 * Constants.foresightConfig, so re-running the Foresight Tuner updates turns too.
 * The robot spins at up to `speed` power, brakes when its own braking data says to,
 * then hands off to follower.hold() for the final settle and x/y correction. Heading
 * is tracked unwrapped, so turns larger than 180 degrees go the way you ask.
 *
 * Usage:
 *   ForesightTurn turn = new ForesightTurn(follower);   // after Constants.create(hardwareMap)
 *   turn.start(90, 0.5);          // 90 deg left (negative = right), half power
 *   // every loop, after follower.update():
 *   turn.update();
 *   if (turn.isDone()) { ... }
 */
public class ForesightTurn {

    /** A turn counts as finished once the heading is within this many degrees of the target. */
    public static double TOLERANCE_DEGREES = 2.0;

    /** Within this many degrees, the spin hands off to Pedro's hold() to finish. */
    public static double HANDOFF_DEGREES = 5.0;

    /** Slowest allowed turn speed, so a turn can't stall at zero power. */
    public static final double MIN_SPEED = 0.1;

    /** Smoothing for the angular velocity estimate (0..1, higher = less smoothing). */
    public static double OMEGA_FILTER = 0.5;

    private enum Phase { IDLE, SPIN, SETTLE, DONE }

    private final Follower follower;
    private final Drivetrain drivetrain;
    private final Controller headingFeedback;   // heading error (rad) -> turn power
    private final double headingBrakeLinear;
    private final double headingBrakeQuadratic;
    private final PoseFactory p = PoseFactory.degrees();

    private Phase phase = Phase.IDLE;
    private double speed;
    private double x, y;
    private double startHeading;   // radians, wrapped
    private double target;         // radians, relative to start, unwrapped
    private double travelled;      // radians, relative to start, unwrapped
    private double lastHeading;    // radians, wrapped
    private double omega;          // rad/s, filtered
    private double timeout;        // seconds
    private final ElapsedTime turnTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();

    /**
     * Uses Constants.drivetrain and reads heading feedback and braking coefficients
     * from Constants.foresightConfig. Call Constants.create(hardwareMap) first.
     */
    public ForesightTurn(Follower follower) {
        this(follower, drivetrainFromConstants(),
                Constants.foresightConfig.headingFeedback.get(),
                brakeCoefficientsFromConfig().x(),   // cartesian(linear, quadratic): x = linear
                brakeCoefficientsFromConfig().y());  //                              y = quadratic
    }

    /**
     * Explicit values, for experimenting without editing Constants.
     * @param headingKP             heading gain (power per radian of error)
     * @param headingBrakeLinear    linear braking coefficient
     * @param headingBrakeQuadratic quadratic braking coefficient
     */
    public ForesightTurn(Follower follower,
                         double headingKP, double headingBrakeLinear, double headingBrakeQuadratic) {
        this(follower, drivetrainFromConstants(), Controller.proportional(headingKP),
                headingBrakeLinear, headingBrakeQuadratic);
    }

    private ForesightTurn(Follower follower, Drivetrain drivetrain, Controller headingFeedback,
                          double headingBrakeLinear, double headingBrakeQuadratic) {
        this.follower = follower;
        this.drivetrain = drivetrain;
        this.headingFeedback = headingFeedback;
        this.headingBrakeLinear = headingBrakeLinear;
        this.headingBrakeQuadratic = headingBrakeQuadratic;
    }

    /** The drivetrain Constants.create() built for the Follower, so both share one object. */
    private static Drivetrain drivetrainFromConstants() {
        if (Constants.drivetrain == null) {
            throw new IllegalStateException(
                    "Constants.drivetrain is null. Call Constants.create(hardwareMap) before creating ForesightTurn.");
        }
        return Constants.drivetrain;
    }

    private static Vector2D brakeCoefficientsFromConfig() {
        return Constants.foresightConfig.headingBrakeCoefficients.get();
    }

    /**
     * Start a turn relative to the current heading.
     * @param degrees positive = left (counter-clockwise), negative = right. Any size.
     * @param speed   maximum turning power, 0.1 to 1
     */
    public void start(double degrees, double speed) {
        // Release any path or hold so the spin owns the drivetrain.
        follower.stop();

        Pose now = follower.pose();
        x = now.x();
        y = now.y();
        startHeading = now.heading();
        lastHeading = startHeading;
        travelled = 0;
        target = Math.toRadians(degrees);
        this.speed = Math.max(MIN_SPEED, Math.min(1.0, speed));
        omega = 0;
        headingFeedback.reset();   // clears any stored state (matters for PID-style controllers)

        // Generous upper bound: a stuck turn ends instead of running forever.
        timeout = 2.0 + Math.abs(degrees) / (90.0 * this.speed);

        phase = Phase.SPIN;
        turnTimer.reset();
        loopTimer.reset();
    }

    /** Call every loop, after follower.update(). */
    public void update() {
        if (phase == Phase.IDLE || phase == Phase.DONE) return;

        // Unwrapped heading tracking, as the Foresight heading tuner does.
        double heading = follower.pose().heading();
        double delta = Angle.normalizeSigned(heading - lastHeading);
        lastHeading = heading;
        travelled += delta;

        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (dt > 1e-4) {
            omega += OMEGA_FILTER * (delta / dt - omega);
        }

        if (turnTimer.seconds() > timeout) {
            if (phase == Phase.SPIN) drivetrain.stop();
            phase = Phase.DONE;
            return;
        }

        if (phase == Phase.SETTLE) {
            double error = Angle.normalizeSigned(startHeading + target - heading);
            if (Math.abs(error) < Math.toRadians(TOLERANCE_DEGREES)) {
                phase = Phase.DONE;   // hold stays active and keeps the robot in place
            }
            return;
        }

        // SPIN
        double remaining = target - travelled;

        boolean close = Math.abs(remaining) < Math.toRadians(HANDOFF_DEGREES);
        boolean stalledNearEnd = Math.abs(remaining) < Math.toRadians(30)
                && Math.abs(omega) < Math.toRadians(5)
                && turnTimer.seconds() > 0.5;

        if (close || stalledNearEnd) {
            // Final settle with Pedro's own hold: tuned heading control plus x/y correction.
            drivetrain.stop();
            follower.hold(p.of(x, y, Math.toDegrees(startHeading + target)));
            phase = Phase.SETTLE;
            return;
        }

        // Foresight heading law: aim at where the robot will actually stop, given its speed.
        double brakingAngle = headingBrakeLinear * omega
                + headingBrakeQuadratic * omega * omega * Math.signum(omega);
        // Controller.calculate(target, error). A proportional controller only uses the error.
        double power = Utils.clamp(
                headingFeedback.calculate(startHeading + target, remaining - brakingAngle),
                -speed, speed);

        drivetrain.drive(new DrivePowers(0.0, 0.0, power), false);
    }

    /** True once the turn has finished (or timed out). */
    public boolean isDone() {
        return phase == Phase.DONE;
    }

    /** True while a turn is spinning or settling. */
    public boolean isActive() {
        return phase == Phase.SPIN || phase == Phase.SETTLE;
    }

    /** Abort the turn. Cuts the motors if spinning; the caller decides what the follower does next. */
    public void stop() {
        if (phase == Phase.SPIN) {
            drivetrain.stop();   // the spin drives the motors directly
        }
        phase = Phase.IDLE;
    }
}