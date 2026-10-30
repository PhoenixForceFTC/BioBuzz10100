package org.firstinspires.ftc.teamcode;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import java.util.function.Consumer;
import java.util.function.Supplier;

/** Cached Pinpoint bridge: field origin and CCW headings stay unchanged; meters become inches. */
public final class PedroPinpointAdapter implements Localizer {
    private final Supplier<MotionSample> samples;
    private final Consumer<Pose> seed;
    private MotionState state = MotionState.zero();

    public PedroPinpointAdapter(PinpointOdometry odometry) {
        this(odometry::getSample, p -> odometry.setPoseInches(p.x(), p.y(), Math.toDegrees(p.heading())));
    }
    PedroPinpointAdapter(Supplier<MotionSample> samples, Consumer<Pose> seed) {
        this.samples = samples; this.seed = seed;
    }
    @Override public void update() { state = convert(samples.get()); }
    static MotionState convert(MotionSample s) {
        double scale = PedroConfiguration.INCHES_PER_METER;
        return MotionState.ofVelocity(new Pose(s.x * scale, s.y * scale, s.heading),
                new Velocity(s.vx * scale, s.vy * scale, s.omega));
    }
    @Override public MotionState state() { return state; }
    @Override public void setPose(Pose pose) {
        seed.accept(pose); state = MotionState.ofVelocity(pose, Velocity.zero());
    }
    @Override public void reset() { setPose(Pose.zero()); }
}
