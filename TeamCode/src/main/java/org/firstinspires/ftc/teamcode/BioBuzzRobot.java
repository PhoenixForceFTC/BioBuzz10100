package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import org.horizon36596.zenith.ivy.AutoFromFile;
import org.horizon36596.zenith.ivy.NamedCommands;
import org.horizon36596.zenith.ivy.ZenithRobot;

/** Shared Pedro + Ivy runtime for every autonomous, including Zenith-generated OpModes. */
public final class BioBuzzRobot implements ZenithRobot {
    private final TurretSubsystem.Alliance selectedAlliance;
    private final boolean pollenTest;
    private TurretSubsystem.Alliance alliance;
    private RobotLoop loop;
    private PedroDrive drive;
    private IntakeSubsystem intake;
    private TransferSubsystem transfer;
    private GateSubsystem gate;
    private TurretSubsystem turret;
    private ShotCoordinator shots;
    private PollenVision vision;
    private boolean running, loading, aiming, fire;
    private double intakePower;
    private long started;
    private String collectionStatus = "Idle";
    private final Object shootingRequirement = new Object();

    /** Zenith-generated stubs use the alliance stored in the file. */
    public BioBuzzRobot() { this(null, false); }
    public BioBuzzRobot(TurretSubsystem.Alliance alliance) { this(alliance, false); }
    private BioBuzzRobot(TurretSubsystem.Alliance alliance, boolean pollenTest) {
        selectedAlliance = alliance; this.pollenTest = pollenTest;
    }
    public static BioBuzzRobot pollenTest() { return new BioBuzzRobot(null, true); }

    @Override public void init(AutoFromFile opMode) {
        alliance = selectedAlliance != null ? selectedAlliance : TurretSubsystem.Alliance.valueOf(opMode.spec().alliance);
        loop = new RobotLoop(opMode.hardwareMap);
        drive = new PedroDrive(opMode.hardwareMap);
        intake = new IntakeSubsystem(opMode.hardwareMap);
        transfer = new TransferSubsystem(opMode.hardwareMap);
        gate = new GateSubsystem(opMode.hardwareMap);
        if (!pollenTest) {
            turret = new TurretSubsystem(opMode.hardwareMap);
            shots = new ShotCoordinator(turret, TeleOpFieldReference.target(alliance, TurretSubsystem.HiveSide.AUDIENCE),
                    ShotCoordinator.AimMode.AUTONOMOUS_ODOMETRY);
        }
        // Optional for planned paths. collectPollen refuses to build without its camera.
        try { vision = new PollenVision(opMode.hardwareMap); }
        catch (RuntimeException e) { opMode.telemetry.addData("Pollen camera unavailable", e.getMessage()); }
    }

    @Override public void registerCommands(AutoFromFile opMode) {
        NamedCommands.register("intakeOn", (args, ctx) -> instant(() -> intakePower = 1, intake));
        NamedCommands.register("intakeOff", (args, ctx) -> instant(() -> intakePower = 0, intake));
        NamedCommands.register("loadOn", (args, ctx) -> instant(() -> loading = true, transfer));
        NamedCommands.register("loadOff", (args, ctx) -> instant(() -> loading = false, transfer));
        NamedCommands.register("aim", (args, ctx) -> {
            requireShooter();
            TurretSubsystem.HiveSide side = NamedCommands.argEnum(args, "aim", "side",
                    TurretSubsystem.HiveSide.class, TurretSubsystem.HiveSide.AUDIENCE);
            return instant(() -> { shots.select(TeleOpFieldReference.target(alliance, side)); aiming = true; }, shootingRequirement);
        });
        NamedCommands.register("shoot", (args, ctx) -> {
            requireShooter();
            double seconds = duration(NamedCommands.argDouble(args, "shoot", "seconds"));
            long[] start = {0};
            return new CommandBuilder().requiring(shootingRequirement, gate, transfer)
                    .setStart(() -> { start[0] = nanoTime(); aiming = fire = true; })
                    .setDone(() -> elapsed(start[0]) >= seconds)
                    .setEnd(reason -> {
                        fire = false; shots.firing().stop(nanoTime() * 1e-9);
                        RobotShutdown.stopAll(gate::close, transfer::stop);
                    });
        });
        NamedCommands.register("collectPollen", (args, ctx) -> collectPollen(
                duration(NamedCommands.argDouble(args, "collectPollen", "seconds", PollenNavigator.AUTO_TIME_LIMIT_SECONDS))));
        NamedCommands.register("stopAll", (args, ctx) -> instant(this::stopOutputs, drive, intake, transfer, gate, shootingRequirement));
        NamedCommands.registerCondition("queueFull", transfer::isFull);
        NamedCommands.registerCondition("pollenVisible", () -> vision != null && vision.isAutonomousReady() && vision.getTarget() != null);
        NamedCommands.registerCondition("localizationReady", () -> drive.ready(nanoTime() * 1e-9));
        NamedCommands.registerCondition("turretHomed", () -> turret != null && turret.isHomed());
    }

    private Command collectPollen(double seconds) {
        if (vision == null) throw new IllegalStateException("collectPollen requires the C270 pollen camera");
        PollenNavigator navigator = new PollenNavigator(PollenVision.CONFIG);
        PollenPathFollower navigation = new PollenPathFollower(drive);
        long[] start = {0};
        return new CommandBuilder().requiring(drive, intake, transfer, gate)
                .setStart(() -> {
                    start[0] = nanoTime(); navigation.stop();
                    MotionSample p = drive.sample(); navigator.start(start[0], p.x * 1000, p.y * 1000);
                    loading = true;
                })
                .setExecute(() -> {
                    long now = nanoTime(); MotionSample p = drive.sample();
                    navigator.update(now, vision.getTarget(), p.x * 1000, p.y * 1000, Math.toDegrees(p.heading),
                            drive.ready(now * 1e-9) && vision.isAutonomousReady(), transfer.isFull());
                    intakePower = navigator.getIntakePower();
                    collectionStatus = navigator.getState() + ": " + navigator.getStatus();
                    navigation.update(navigator, now);
                })
                .setDone(() -> navigator.isFinished() || elapsed(start[0]) >= seconds)
                .setEnd(reason -> {
                    navigator.stop(nanoTime()); intakePower = 0; loading = false;
                    RobotShutdown.stopAll(navigation::stop, intake::stop, transfer::stop, gate::close);
                });
    }

    private static Command instant(Runnable action, Object... requirements) {
        return new CommandBuilder().requiring(requirements).setStart(action).setDone(() -> true);
    }
    private void requireShooter() {
        if (shots == null) throw new IllegalStateException("The pollen navigation test does not initialize shooting hardware");
    }
    private static double duration(double seconds) {
        if (!Double.isFinite(seconds) || seconds <= 0 || seconds > PollenNavigator.AUTO_TIME_LIMIT_SECONDS)
            throw new IllegalArgumentException("Command seconds must be greater than 0 and at most 28");
        return seconds;
    }
    private double elapsed(long since) { return (nanoTime() - since) * 1e-9; }
    @Override public Follower follower() { return drive.follower(); }
    @Override public Object drive() { return drive; }
    @Override public String runningAlliance() { return alliance.name(); }
    @Override public Pose mirror(Pose p) { return new Pose(-p.x(), -p.y(), p.heading() + Math.PI); }
    @Override public Path atSpeed(Path path, double fraction) { return drive.atSpeed(path, fraction); }
    @Override public void setStartPose(Pose pose) { drive.setStartPose(pose); }

    private void readSensors() {
        loop.begin(); drive.readSensors();
        if (turret != null) turret.update();
    }
    @Override public void initLoop(AutoFromFile opMode) {
        readSensors();
        opMode.telemetry.addLine("Keep robot stationary during Pinpoint calibration.");
        drive.addTelemetry(opMode.telemetry);
        opMode.telemetry.addData("Start pose", drive.ready(nanoTime() * 1e-9) ? "READY" : "Waiting for Pinpoint");
        if (turret != null) turret.addTelemetry(opMode.telemetry);
        if (vision != null) vision.addTelemetry(opMode.telemetry);
        loop.addTelemetry(opMode.telemetry); // AutoFromFile flushes init telemetry once.
    }
    @Override public void onStart(AutoFromFile opMode) {
        if (!PedroConfiguration.TUNING_VERIFIED) throw new IllegalStateException("Measure and verify PedroConfiguration before running autonomous; use Test: Pedro Drive");
        if (!drive.ready(nanoTime() * 1e-9)) throw new IllegalStateException("Pinpoint has no READY, fresh autonomous start pose");
        started = nanoTime(); running = true;
    }
    @Override public void periodic(AutoFromFile opMode) {
        readSensors();
        if (!drive.ready(nanoTime() * 1e-9)) throw new IllegalStateException("Autonomous stopped: Pinpoint localization unavailable or stale");
        if (elapsed(started) >= PedroConfiguration.MAX_AUTONOMOUS_SECONDS) {
            finish(opMode); return;
        }
        drive.update(); // One follower update; localizer reads the cached sample.
    }
    @Override public void afterLoop(AutoFromFile opMode) {
        if (running && !Scheduler.isScheduled(opMode.routine())) finish(opMode);
        if (running) {
            double now = nanoTime() * 1e-9;
            if (shots != null) {
                shots.update(now, drive.sample(), aiming, fire, false, false, false);
                gate.setOpen(shots.firing().gateOpen());
            } else gate.close();
            boolean feeding = shots != null && shots.firing().feeding() && gate.isOpenForFeeding();
            transfer.update(feeding, !loading || fire);
            intake.setIntakePower(transfer.isFull() ? 0 : intakePower);
        }
        drive.addTelemetry(opMode.telemetry);
        opMode.telemetry.addData("Pollen navigation test", collectionStatus);
        transfer.addTelemetry(opMode.telemetry);
        if (shots != null) shots.addTelemetry(opMode.telemetry, nanoTime() * 1e-9);
        if (vision != null) vision.addTelemetry(opMode.telemetry);
        loop.addTelemetry(opMode.telemetry); opMode.telemetry.update();
    }
    private void finish(AutoFromFile opMode) {
        running = false;
        RobotShutdown.stopAll(() -> Scheduler.cancel(opMode.routine()), this::stopOutputs);
        opMode.requestOpModeStop();
    }
    private void stopOutputs() {
        intakePower = 0; loading = aiming = fire = false;
        if (shots != null) shots.firing().stop(nanoTime() * 1e-9);
        RobotShutdown.stopAll(drive == null ? null : drive::stop, intake == null ? null : intake::stop,
                transfer == null ? null : transfer::stop, gate == null ? null : gate::close, turret == null ? null : turret::stop);
    }
    @Override public void onStop(AutoFromFile opMode) {
        running = false;
        RobotShutdown.stopAll(opMode.routine() == null ? null : () -> Scheduler.cancel(opMode.routine()),
                this::stopOutputs, vision == null ? null : vision::close);
    }
}
