package org.firstinspires.ftc.teamcode;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.math.Pose;
import org.horizon36596.zenith.ivy.AutoContext;
import org.horizon36596.zenith.ivy.NamedCommands;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.function.BiFunction;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

/** Validated INIT factories. Units, resources, defaults and bounds also appear in autos/robot.json. */
public final class AutonomousCommands {
    public enum TargetFrame { FIELD, AUTHORED }
    private final BioBuzzRobot robot;
    private final AutonomousOutputCoordinator out;
    public AutonomousCommands(BioBuzzRobot robot) {
        this.robot = robot; out = robot.outputs;
    }
    public void register() {
        command("intakeOn", "intake", "", (a,c) -> instant(() -> out.intake(1), "intake"));
        command("intakeOff", "intake", "", (a,c) -> instant(() -> out.intake(0), "intake"));
        command("intakeReverse", "intake", "", (a,c) -> instant(() -> out.intake(-1), "intake"));
        command("intakeSet", "intake", "power", (a,c) -> {
            double power = number(a,"power",null,-1,1);
            return instant(() -> out.intake(power), "intake");
        });
        command("intakeRun", "intake", "power,seconds", (a,c) -> {
            double power = number(a,"power",null,-1,1);
            return timed(seconds(a), () -> out.intake(power), () -> false, null,
                    lease -> { if (out.owns(lease,"intake")) { out.intake(0); robot.intake.stop(); } }, false, "intake");
        });
        command("loadOn", "transfer", "", (a,c) -> instant(() -> out.loading(true), "transfer"));
        command("loadOff", "transfer", "", (a,c) -> instant(() -> out.loading(false), "transfer"));
        command("transferStop", "transfer,gate", "", (a,c) -> instant(this::stopFeed, "transfer","gate"));
        command("loadUntilFull", "intake,transfer", "seconds", (a,c) -> timed(seconds(a),
                () -> { out.intake(1); out.loading(true); }, robot.transfer::isFull, null, lease -> {
                    RobotShutdown.stopAll(
                            out.owns(lease,"intake") ? () -> { out.intake(0); robot.intake.stop(); } : null,
                            out.owns(lease,"transfer") ? () -> { out.loading(false); robot.transfer.pause(); } : null);
                }, true, "intake","transfer"));
        command("feedManual", "transfer,gate", "seconds", (a,c) -> feed(seconds(a), false, null));
        command("feedStop", "transfer,gate", "", (a,c) -> instant(this::stopFeed, "transfer","gate"));
        command("gateOpen", "gate,transfer", "seconds", (a,c) -> timed(seconds(a),
                () -> { stopFeed(); out.gateOnly(); }, () -> false, null,
                this::cleanupFeed, false, "gate","transfer"));
        command("gateClose", "gate,transfer", "", (a,c) -> instant(this::stopFeed, "gate","transfer"));
        command("feedInhibit", "gate,transfer", "", (a,c) -> instant(() -> { out.inhibit(true); stopFeed(); }, "gate","transfer"));
        command("feedResume", "gate,transfer", "", (a,c) -> instant(() -> out.inhibit(false), "gate","transfer"));
        command("depollinatorDeploy", "depollinator", "", (a,c) -> instant(robot.depollinator::deploy, "depollinator"));
        command("depollinatorHome", "depollinator", "", (a,c) -> instant(robot.depollinator::home, "depollinator"));

        command("turretHome", "turretYaw,gate,transfer", "timeoutSeconds", (a,c) -> timed(timeout(a),
                () -> { out.manualYaw(); stopFeed(); robot.turret.homeTurretAndZeroEncoder(robot.now()); },
                robot.turret::isHomed, robot.turret::hasHomingFault,
                lease -> { if (out.owns(lease,"turretYaw") && !robot.turret.isHomed()) robot.turret.holdYaw(); },
                true, "turretYaw","gate","transfer"));
        command("turretSetYaw", "turretYaw", "yawDeg", (a,c) -> {
            double yaw = number(a,"yawDeg",null,-175,175);
            return instant(() -> yaw(yaw), "turretYaw");
        });
        command("turretAdjustYaw", "turretYaw", "deltaDeg", (a,c) -> {
            double delta = number(a,"deltaDeg",null,-350,350);
            return instant(() -> yaw(range("adjusted yawDeg",robot.turret.getTargetYawDegrees()+delta,-175,175)), "turretYaw");
        });
        command("turretHold", "turretYaw", "", (a,c) -> instant(() -> {
            out.manualYaw(); robot.turret.holdYaw(); }, "turretYaw"));
        command("waitTurretReady", "turretYaw", "timeoutSeconds", (a,c) -> timed(timeout(a), () -> {},
                this::turretReady, robot.turret::hasHomingFault, lease -> {}, true, "turretYaw"));
        command("hoodSet", "hood", "travelDeg", (a,c) -> {
            double hood = number(a,"travelDeg",null,0,30);
            return instant(() -> hood(hood), "hood");
        });
        command("hoodAdjust", "hood", "deltaDeg", (a,c) -> {
            double delta = number(a,"deltaDeg",null,-30,30);
            return instant(() -> hood(range("adjusted hood travelDeg",robot.turret.getTargetPitchDegrees()+delta,0,30)), "hood");
        });
        command("hoodHome", "hood", "", (a,c) -> instant(() -> hood(0), "hood"));
        command("waitHoodReady", "hood", "timeoutSeconds", (a,c) -> timed(timeout(a), () -> {},
                robot.turret::isHoodTracking, null, lease -> {}, true, "hood"));
        command("flywheelSetRpm", "flywheel", "rpm", (a,c) -> {
            double rpm = number(a,"rpm",null,0,TurretSubsystem.SHOOTER_MAX_RPM);
            return instant(() -> { out.rpm(rpm); robot.turret.setShooterRpm(rpm); }, "flywheel");
        });
        command("flywheelSpinUp", "flywheel", "rpm,timeoutSeconds", (a,c) -> {
            double rpm = number(a,"rpm",null,1,TurretSubsystem.SHOOTER_MAX_RPM);
            @SuppressWarnings("unchecked") Map<String,Long>[] owner = new Map[1];
            Command spin = timed(timeout(a), () -> { out.rpm(rpm); robot.turret.setShooterRpm(rpm); },
                    robot.turret::isShooterAtSpeed, null, lease -> owner[0] = lease, true, "flywheel");
            return afterEnd(spin, reason -> {
                if (out.owns(owner[0],"flywheel") && (reason != EndCondition.NATURALLY || !robot.turret.isShooterAtSpeed())) {
                    out.rpm(0); robot.turret.stopShooter();
                }
            });
        });
        command("flywheelStop", "flywheel,gate,transfer", "", (a,c) -> instant(() -> {
            out.rpm(0); stopFeed(); robot.turret.stopShooter(); }, "flywheel","gate","transfer"));
        command("selectTarget", "turretYaw,hood,flywheel,gate,transfer", "side,frame", (a,c) -> {
            TurretSubsystem.HiveSide side = side(a,c);
            return instant(() -> select(side), "turretYaw","hood","flywheel","gate","transfer");
        });
        command("aim", "turretYaw,hood,flywheel,gate,transfer", "side,frame,spin", (a,c) -> {
            TurretSubsystem.HiveSide side = side(a,c);
            boolean spin = bool(a,"spin",true);
            return instant(() -> { select(side); out.aim(spin); }, "turretYaw","hood","flywheel","gate","transfer");
        });
        command("aimOff", "turretYaw,hood,flywheel,gate,transfer", "", (a,c) -> instant(() -> {
            out.aimOff(); robot.turret.holdYaw(); robot.turret.stopShooter(); stopFeed();
        }, "turretYaw","hood","flywheel","gate","transfer"));
        command("shoot", "turretYaw,hood,flywheel,gate,transfer", "seconds", (a,c) -> feed(seconds(a), true, null));
        command("prepareShotPreset", "turretYaw,hood,flywheel,gate,transfer", "preset", (a,c) -> {
            AutonomousShotPresets.Preset preset = preset(a);
            return instant(() -> prepare(preset), "turretYaw","hood","flywheel","gate","transfer");
        });
        command("shootPreset", "turretYaw,hood,flywheel,gate,transfer", "preset,seconds", (a,c) ->
                feed(seconds(a), true, preset(a)));
        command("driveHold", "drive", "xIn,yIn,headingRad,seconds", (a,c) -> {
            Pose pose = pose(a,c);
            return timed(seconds(a), () -> { robot.drive.stop(); robot.follower().hold(pose); },
                    () -> false, null, lease -> { if (out.owns(lease,"drive")) robot.drive.stop(); }, false, "drive");
        });
        command("driveStop", "drive", "", (a,c) -> instant(robot.drive::stop, "drive"));
        command("setFieldPose", "drive,gate,transfer", "xIn,yIn,headingRad", (a,c) -> {
            Pose pose = pose(a,c);
            return instant(() -> {
                stopFeed();
                if (robot.shots != null) robot.shots.invalidateReference(robot.now());
                robot.drive.setFieldPose(pose);
            }, "drive","gate","transfer");
        });
        command("collectPollen", "drive,intake,transfer,gate,pollenCamera", "seconds", (a,c) ->
                collect(number(a,"seconds",28.0,.001,28)));
        command("pollenVisionEnable", "pollenCamera", "", (a,c) -> instant(() -> {
            robot.vision.setEnabled(true); robot.pollenSnapshot=null;
        }, "pollenCamera"));
        command("pollenVisionDisable", "pollenCamera,drive,intake,transfer,gate", "", (a,c) -> instant(() -> {
            robot.vision.setEnabled(false); robot.pollenSnapshot=null; robot.drive.stop(); out.intake(0); stopFeed();
        }, "pollenCamera","drive","intake","transfer","gate"));
        command("limelightSetPipeline", "limelight,gate,transfer", "pipeline", (a,c) -> {
            // Only pipeline 0 is documented/configured on this robot.
            double pipeline = number(a,"pipeline",0.0,0,0);
            return instant(() -> {
                stopFeed();
                robot.turret.setLimelightPipeline((int)pipeline);
                robot.shots.invalidateReference(robot.now());
            }, "limelight","gate","transfer");
        });
        command("stopAll", "*", "", (a,c) -> instant(robot::stopOutputs, supportedChannels()));
        conditions();
    }
    private void command(String name, String resources, String params, BiFunction<Map<String,Object>,AutoContext,Command> factory) {
        String[] required = resources.equals("*") ? supportedChannels() : split(resources);
        Set<String> allowed = new HashSet<>(Arrays.asList(split(params)));
        NamedCommands.register(name, (args,ctx) -> {
            robot.profile.require(name, required);
            for (String key : args.keySet()) if (!allowed.contains(key))
                throw new IllegalArgumentException(name + ": unknown argument '" + key + "'");
            if (!name.equals("stopAll") && Arrays.asList(required).contains("pollenCamera") && robot.vision == null)
                throw new IllegalArgumentException(name + " requires the C270 pollen camera initialized during INIT");
            if (!name.equals("stopAll") && Arrays.asList(required).contains("limelight") && (robot.turret == null || !robot.turret.hasLimelight()))
                throw new IllegalArgumentException(name + " requires a configured Limelight");
            return factory.apply(args,ctx);
        });
    }
    private static String[] split(String value) { return value.isEmpty() ? new String[0] : value.split(","); }
    private String[] supportedChannels() {
        return robot.profile == AutonomousProfile.FULL_ROBOT
                ? split("drive,intake,transfer,gate,depollinator,turretYaw,hood,flywheel,pollenCamera,limelight")
                : split("drive,intake,transfer,gate,pollenCamera");
    }
    private Object[] requirements(String... resources) {
        Object[] result = new Object[resources.length];
        for (int i=0;i<resources.length;i++) result[i] = resources[i].equals("drive") ? robot.drive : resources[i];
        return result;
    }
    private Command instant(Runnable action, String... resources) {
        return new CommandBuilder().requiring(requirements(resources))
                .setStart(() -> { out.claim(resources); action.run(); }).setDone(() -> true);
    }
    private Command timed(double seconds, Runnable start, BooleanSupplier success, BooleanSupplier fault,
                          Consumer<Map<String,Long>> cleanup, boolean waitForSuccess, String... resources) {
        long[] since = {0}; boolean[] ended = {false};
        @SuppressWarnings("unchecked") Map<String,Long>[] lease = new Map[1];
        return new CommandBuilder().requiring(requirements(resources))
                .setStart(() -> { since[0]=robot.nanoTime(); ended[0]=false; lease[0]=out.claim(resources); start.run(); })
                .setDone(() -> success.getAsBoolean() || (fault != null && fault.getAsBoolean())
                        || (robot.nanoTime()-since[0])*1e-9 >= seconds)
                .setEnd(reason -> {
                    if (ended[0]) return;
                    ended[0]=true;
                    boolean failed = fault != null && fault.getAsBoolean();
                    boolean succeeded = success.getAsBoolean();
                    try { cleanup.accept(lease[0]); }
                    catch (RuntimeException failure) {
                        robot.outcome("FAULT", "Cleanup failed: " + failure.getMessage()); throw failure;
                    }
                    if (reason != EndCondition.NATURALLY) robot.outcome("CANCELLED", "Interrupted bounded action");
                    else if (failed) robot.outcome("FAULT", "Mechanism fault");
                    else if (waitForSuccess && !succeeded) robot.outcome("TIMEOUT", "Readiness/full timeout");
                    else robot.outcome("SUCCESS", waitForSuccess ? "Predicate satisfied" : "Duration elapsed");
                });
    }
    private void stopFeed() {
        out.clearFeed(); out.loading(false);
        RobotShutdown.stopAll(() -> robot.firing.stop(robot.now()), robot.gate::close, robot.transfer::pause);
    }
    private void cleanupFeed(Map<String,Long> lease) {
        RobotShutdown.stopAll(
                out.owns(lease,"gate") ? () -> { out.clearFeed(); robot.firing.stop(robot.now()); robot.gate.close(); } : null,
                out.owns(lease,"transfer") ? () -> { out.loading(false); robot.transfer.pause(); } : null);
    }
    private static Command afterEnd(Command command, Consumer<EndCondition> after) {
        boolean[] ended = {false};
        return new CommandBuilder().requiring(command.requirements())
                .setStart(() -> { ended[0]=false; command.start(); }).setExecute(command::execute).setDone(command::done)
                .setEnd(reason -> {
                    if (ended[0]) return;
                    ended[0]=true;
                    command.end(reason); after.accept(reason);
                });
    }
    private void yaw(double value) { out.manualYaw(); robot.turret.setTargetYawDegrees(value); }
    private void hood(double value) { out.manualHood(); robot.turret.setPitchAngleDegrees(value); }
    private void select(TurretSubsystem.HiveSide side) {
        stopFeed(); robot.shots.select(TeleOpFieldReference.target(robot.alliance,side));
        robot.shots.invalidateReference(robot.now());
    }
    private void prepare(AutonomousShotPresets.Preset preset) {
        stopFeed(); out.prepare(preset);
        robot.turret.setTargetYawDegrees(preset.yawDeg); robot.turret.setPitchAngleDegrees(preset.hoodTravelDeg);
        robot.turret.setShooterRpm(preset.outputRpm);
    }
    private Command feed(double seconds, boolean automatic, AutonomousShotPresets.Preset preset) {
        String[] resources = automatic ? split("turretYaw,hood,flywheel,gate,transfer") : split("gate,transfer");
        int[] attempts = {0};
        Command result = timed(seconds, () -> {
            stopFeed(); attempts[0] = robot.firing.feedAttempts();
            if (automatic) {
                if (preset != null) prepare(preset); else out.aim(true);
                out.requestAutomaticFeed();
            } else out.requestManualFeed();
        }, () -> false, null, lease -> {
            cleanupFeed(lease);
        }, false, resources);
        // Preserve an inhibited outcome after timed()'s normal completion bookkeeping.
        return afterEnd(result, reason -> {
                    boolean blocked = out.inhibited() || (automatic && robot.firing.feedAttempts() == attempts[0]);
                    if (reason == EndCondition.NATURALLY && blocked)
                        robot.outcome("INHIBITED", automatic && robot.shots != null ? robot.shots.solution().reason : "Feed inhibit");
                });
    }
    private Command collect(double seconds) {
        PollenNavigator navigator = new PollenNavigator(PollenVision.CONFIG);
        PollenPathFollower navigation = new PollenPathFollower(robot.drive);
        Command duration = timed(seconds, () -> {
            stopFeed(); navigation.stop();
            MotionSample p=robot.drive.sample(); navigator.start(robot.nanoTime(),p.x*1000,p.y*1000); out.loading(true);
        }, navigator::isFinished, null, lease -> {
            navigator.stop(robot.nanoTime());
            RobotShutdown.stopAll(
                    out.owns(lease,"drive") ? navigation::stop : null,
                    out.owns(lease,"intake") ? () -> { out.intake(0); robot.intake.stop(); } : null,
                    out.owns(lease,"transfer") && out.owns(lease,"gate") ? this::stopFeed : null);
        }, false, "drive","intake","transfer","gate","pollenCamera");
        Command collection = new CommandBuilder().requiring(duration.requirements()).setStart(duration::start)
                .setExecute(() -> {
                    long now=robot.nanoTime(); MotionSample p=robot.drive.sample();
                    navigator.update(now,robot.pollenSnapshot == null ? null : robot.pollenSnapshot.target,
                            p.x*1000,p.y*1000,Math.toDegrees(p.heading),
                            robot.drive.ready(now*1e-9) && robot.pollenSnapshot != null && robot.pollenSnapshot.autonomousReady,robot.transfer.isFull());
                    out.intake(navigator.getIntakePower()); robot.collectionStatus=navigator.getState()+": "+navigator.getStatus();
                    navigation.update(navigator,now);
                }).setDone(duration::done).setEnd(reason -> {
                    PollenNavigator.State state = navigator.getState();
                    String status = navigator.getStatus();
                    duration.end(reason);
                    if (reason == EndCondition.NATURALLY) robot.outcome(
                            state == PollenNavigator.State.FAULT ? "FAULT" :
                            state == PollenNavigator.State.COMPLETE ? "SUCCESS" : "TIMEOUT", status);
                });
        return afterEnd(collection, reason -> {});
    }
    private boolean turretReady() {
        return robot.turret != null && robot.turret.isHomed()
                && ShotReadiness.angleReady(Math.abs(robot.turret.getTargetYawDegrees()-robot.turret.getEncoderAngleDegrees()));
    }
    private void condition(String name, BooleanSupplier predicate) { NamedCommands.registerCondition(name,predicate); }
    private void conditions() {
        condition("queueFull", robot.transfer::isFull);
        condition("queueSensorHigh", robot.transfer::isSensorHigh);
        condition("loadingActive", robot.transfer::isLoading);
        condition("feedingActive", robot.transfer::isFeeding);
        condition("gateFeedDelayElapsed", robot.gate::isOpenForFeeding);
        condition("feedInhibited", out::inhibited);
        condition("localizationReady", () -> robot.drive.ready(robot.now()));
        condition("driveIdle", () -> robot.drive.idle());
        condition("turretHomed", () -> robot.turret != null && robot.turret.isHomed());
        condition("turretAtTarget", this::turretReady);
        condition("hoodTrackingEstimated", () -> robot.turret != null && robot.turret.isHoodTracking());
        condition("flywheelAtSpeed", () -> robot.turret != null && robot.turret.isShooterAtSpeed());
        condition("aimReady", () -> robot.shots != null && robot.shots.solution().aimable);
        condition("shotReady", () -> robot.shots != null && robot.shots.validForFeeding() && turretReady()
                && robot.turret.isHoodTracking() && robot.turret.isShooterAtSpeed());
        condition("pollenVisible", () -> robot.pollenSnapshot != null && robot.pollenSnapshot.autonomousReady && robot.pollenSnapshot.target != null);
        condition("pollenCameraReady", () -> robot.pollenSnapshot != null && robot.pollenSnapshot.cameraReady);
        condition("limelightFresh", () -> robot.turret != null && robot.turret.getVisionResult() != null
                && robot.turret.getVisionResult().isValid() && robot.turret.getVisionResult().getStaleness() >= 0
                && robot.turret.getVisionResult().getStaleness() <= TurretSubsystem.LIMELIGHT_MAX_STALENESS_MS);
        condition("selectedGoalTagVisible", () -> robot.turret != null && robot.turret.getVisionTag() != null);
        condition("depollinatorDeployedCommanded", () -> robot.depollinator != null && robot.depollinator.isDeployedCommanded());
        condition("depollinatorHomeCommanded", () -> robot.depollinator != null && !robot.depollinator.isDeployedCommanded());
        condition("depollinatorSettledEstimated", () -> robot.depollinator != null && robot.depollinator.isSettledEstimated());
        condition("commandFailed", robot::commandFailed);
        condition("matchTimeLow", () -> robot.running && robot.remainingSeconds() <= BioBuzzRobot.MATCH_TIME_LOW_SECONDS);
    }
    public static double range(String label, double value, double min, double max) {
        if (!Double.isFinite(value) || value < min || value > max)
            throw new IllegalArgumentException(label + " must be finite in [" + min + ", " + max + "]");
        return value;
    }
    static double number(Map<String,Object> args, String key, Double fallback, double min, double max) {
        Object value=args.get(key);
        if (value == null && !args.containsKey(key) && fallback != null) return range(key,fallback,min,max);
        if (!(value instanceof Number)) throw new IllegalArgumentException(key + " must be a number");
        return range(key,((Number)value).doubleValue(),min,max);
    }
    private static boolean bool(Map<String,Object> args, String key, boolean fallback) {
        if (!args.containsKey(key)) return fallback;
        if (!(args.get(key) instanceof Boolean)) throw new IllegalArgumentException(key + " must be boolean");
        return (Boolean)args.get(key);
    }
    private static double seconds(Map<String,Object> args) { return number(args,"seconds",null,.001,28); }
    private static double timeout(Map<String,Object> args) { return number(args,"timeoutSeconds",5.0,.001,28); }
    private static AutonomousShotPresets.Preset preset(Map<String,Object> args) {
        return AutonomousShotPresets.require(NamedCommands.argString(args,"preset","preset"));
    }
    private static Pose pose(Map<String,Object> args, AutoContext context) {
        return context.alliance(new Pose(number(args,"xIn",null,-72,72),number(args,"yIn",null,-72,72),
                number(args,"headingRad",0.0,-2*Math.PI,2*Math.PI)));
    }
    private static TurretSubsystem.HiveSide side(Map<String,Object> args, AutoContext context) {
        TurretSubsystem.HiveSide side=NamedCommands.argEnum(args,"target","side",TurretSubsystem.HiveSide.class,TurretSubsystem.HiveSide.AUDIENCE);
        TargetFrame frame=NamedCommands.argEnum(args,"target","frame",TargetFrame.class,TargetFrame.FIELD);
        return transformSide(side,frame,context.alliance(Pose.zero()).heading());
    }
    static TurretSubsystem.HiveSide transformSide(TurretSubsystem.HiveSide side, TargetFrame frame, double transformedZeroHeading) {
        return frame == TargetFrame.AUTHORED && Math.abs(RobotMath.wrap(transformedZeroHeading)) > Math.PI/2
                ? (side == TurretSubsystem.HiveSide.AUDIENCE ? TurretSubsystem.HiveSide.FAR : TurretSubsystem.HiveSide.AUDIENCE) : side;
    }
}
