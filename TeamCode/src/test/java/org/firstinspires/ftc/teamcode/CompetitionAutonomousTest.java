package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import org.horizon36596.zenith.AutoFile;
import org.horizon36596.zenith.AutoSource;
import org.horizon36596.zenith.Json;
import org.horizon36596.zenith.ivy.AutoBuilder;
import org.horizon36596.zenith.ivy.AutoContext;
import org.horizon36596.zenith.ivy.AutoFromFile;
import org.horizon36596.zenith.ivy.NamedCommands;
import org.junit.Before;
import org.junit.After;
import org.junit.Test;
import java.io.File;
import java.util.*;
import static org.junit.Assert.*;

public class CompetitionAutonomousTest {
    private final World world = new World();
    @Before public void setup() {
        Scheduler.reset(); NamedCommands.reset();
        NamedCommands.register("record", (args,ctx) -> new CommandBuilder()
                .setStart(() -> world.events.add(NamedCommands.argString(args,"record","tag"))).setDone(() -> true));
        NamedCommands.register("never", (args,ctx) -> new CommandBuilder()
                .setEnd(reason -> world.events.add("never:" + reason)));
        NamedCommands.registerCondition("selected", () -> true);
    }
    @After public void reset() { Scheduler.reset(); NamedCommands.reset(); }
    private Command build(String steps) {
        AutoFile file = AutoFile.parse(Json.parse("{\"formatVersion\":4,\"start\":{\"pose\":{\"xIn\":0,\"yIn\":0}},\"steps\":" + steps + "}", "synthetic"),
                "synthetic", Collections.emptyMap());
        return AutoBuilder.build(file, world);
    }
    private void run(Command command) {
        Scheduler.schedule(command);
        for (int i=0;i<300 && Scheduler.isScheduled(command);i++) {
            world.now += 20_000_000L; world.advance(); Scheduler.execute();
        }
        assertFalse("Routine must finish on the simulated robot clock", Scheduler.isScheduled(command));
    }
    @Test public void everyAutonomousUsesTheIvyRuntimeAndPollenIsATest() {
        for (Class<?> type : new Class<?>[]{RedCompetitionAutonomous.class, BlueCompetitionAutonomous.class, PollenAuto.class})
            assertTrue(AutoFromFile.class.isAssignableFrom(type));
        assertEquals("Test", PollenAuto.class.getAnnotation(com.qualcomm.robotcore.eventloop.opmode.Autonomous.class).group());
    }
    @Test public void packagedPollenTestRunsThroughTheRealFileBuilder() throws Exception {
        NamedCommands.register("collectPollen", (args,ctx) -> {
            assertEquals(28, NamedCommands.argDouble(args,"collectPollen","seconds"), 0);
            return new CommandBuilder().setStart(() -> world.events.add("collect")).setDone(() -> true);
        });
        NamedCommands.register("stopAll", (args,ctx) -> new CommandBuilder().setStart(() -> world.events.add("stop")).setDone(() -> true));
        AutoFile file = AutoFile.load(AutoSource.directory(new File("src/main/assets/autos")), "pollen-navigation-test");
        run(AutoBuilder.build(file, world));
        assertEquals(Arrays.asList("collect","stop"), world.events);
    }
    @Test public void curvesMarkersParallelTimeoutsBranchesAndCurrentStartsKeepTheirSemantics() throws Exception {
        AutoFile file = AutoFile.load(AutoSource.directory(new File("src/test/resources/zenith")), "curves-and-actions");
        run(AutoBuilder.build(file, world));
        assertEquals(2, world.followed.size());
        assertTrue(world.followed.get(0).getSegments().get(0).curve.getClass().getSimpleName().contains("Bezier"));
        assertEquals(Arrays.asList(.25,.2), world.speeds);
        assertEquals(24, world.followed.get(1).getSegments().get(0).get(0).x(), 1e-8);
        assertEquals(36, world.pose.x(), 1e-8);
        assertEquals(Arrays.asList("halfway","endpoint","never:INTERRUPTED","yes","done"), world.events);
    }
    @Test public void anEarlyPathEndCancelsUnreachedMarkersAndStopsFollowing() {
        NamedCommands.registerCondition("cut", () -> world.progress >= .3);
        run(build("[{\"id\":\"cut\",\"kind\":\"path\",\"segments\":[{\"kind\":\"line\",\"from\":\"current\",\"to\":{\"xIn\":24,\"yIn\":0}}],\"heading\":{\"mode\":\"constant\",\"headingRad\":0},\"endCondition\":{\"condition\":\"cut\"},\"markers\":[{\"at\":{\"t\":0.9},\"command\":{\"name\":\"record\",\"args\":{\"tag\":\"too late\"}}}]}]"));
        assertTrue(world.events.isEmpty()); assertFalse(world.following); assertTrue(world.holds > 0);
    }
    @Test public void cancellationEndsRunningActionsAndStopsThePath() {
        Command command=build("[{\"kind\":\"parallel\",\"mode\":\"all\",\"steps\":[{\"kind\":\"path\",\"segments\":[{\"kind\":\"line\",\"from\":\"current\",\"to\":{\"xIn\":24,\"yIn\":0}}],\"heading\":{\"mode\":\"constant\",\"headingRad\":0}},{\"kind\":\"command\",\"name\":\"never\"}]}]");
        Scheduler.schedule(command); world.advance(); Scheduler.execute(); Scheduler.cancel(command);
        assertFalse(world.following); assertEquals(Collections.singletonList("never:INTERRUPTED"),world.events);
    }
    @Test public void invalidNamesVersionsAndDuplicateIdsFailBeforeMotion() {
        assertThrows(IllegalArgumentException.class, () -> build("[{\"kind\":\"command\",\"name\":\"missing\"}]"));
        assertThrows(IllegalArgumentException.class, () -> build("[{\"id\":\"same\",\"kind\":\"wait\",\"seconds\":1},{\"id\":\"same\",\"kind\":\"wait\",\"seconds\":1}]"));
        assertThrows(IllegalStateException.class, () -> AutoFile.parse(Json.parse("{\"formatVersion\":999}", "future"), "future", Collections.emptyMap()));
        assertTrue(world.followed.isEmpty());
    }
    static final class World implements AutoContext {
        long now; boolean following; double progress; int holds;
        Pose pose=Pose.zero(); Path path;
        final List<String> events=new ArrayList<>();
        final List<Path> followed=new ArrayList<>();
        final List<Double> speeds=new ArrayList<>();
        final Follower follower=new Follower(null,null,null) {
            @Override public void follow(Path p) { path=p; followed.add(p); progress=0; following=true; }
            @Override public boolean following() { return following; }
            @Override public Pose pose() { return pose; }
            @Override public void hold(Pose p) { following=false; pose=p; holds++; }
            @Override public int pathIndex() { return 0; }
            @Override public double curveCompletion() { return progress; }
        };
        void advance() {
            if (following) {
                progress=Math.min(1,progress+.1);
                pose=path.getSegments().get(0).get(progress);
                if (progress>=1-1e-9) { pose=path.endPose(); following=false; }
            }
        }
        @Override public Follower follower() { return follower; }
        @Override public Object drive() { return this; }
        @Override public Pose alliance(Pose pose) { return pose; }
        @Override public long nanoTime() { return now; }
        @Override public Path atSpeed(Path path,double speed) { speeds.add(speed); return path; }
    }
}
