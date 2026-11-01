package org.firstinspires.ftc.teamcode.sim;

import java.io.IOException;
import org.horizon36596.simloop.opmode.GamepadScript;
import org.horizon36596.simloop.opmode.OpModeRunResult;
import org.horizon36596.simloop.opmode.OpModeRunner;
import org.horizon36596.simloop.opmode.RecordingTelemetry;

final class TeleOpScenario {
    final BioBuzzSimHardware hardware;
    final BioBuzzSimModel model;
    final TeleOpTrace trace;
    final OpModeRunner runner;
    OpModeRunResult result;
    java.util.function.Consumer<TeleOpTrace.Sample> observer = sample -> {};

    TeleOpScenario(String name, String opMode, BioBuzzSimHardware hardware, double tickSeconds,
                   int initTicks, int activeTicks, GamepadScript inputs) throws IOException {
        this.hardware = hardware;
        model = new BioBuzzSimModel(hardware);
        trace = new TeleOpTrace(name, opMode, tickSeconds);
        java.nio.file.Files.write(trace.directory.resolve("camera-convention.txt"),
                (hardware.cameraAdapter == null ? "RAW SimLoop alpha.2 target +Z into print\n"
                        : "TEST-ONLY Rx(pi) target basis normalization; physical convention UNVALIDATED\n").getBytes(java.nio.charset.StandardCharsets.UTF_8));
        runner = OpModeRunner.forTeamClass("org.firstinspires.ftc.teamcode." + opMode, hardware.map)
                .teamPackages("org.firstinspires.ftc.teamcode.", "com.seattlesolvers.solverslib.", "com.pedropathing.")
                .tickSeconds(tickSeconds).initLoopTicks(initTicks).activeTicks(activeTicks)
                .failOnOpModeThreads(true)
                .gamepads((phase, tick, gp1, gp2) -> {
                    inputs.apply(phase, tick, gp1, gp2);
                    trace.input(phase, tick, gp1, gp2);
                });
        runner.eachTick(dt -> {
            model.update(dt);
            trace.sample(hardware, model, dt, (RecordingTelemetry) runner.opMode().telemetry);
            observer.accept(trace.samples.get(trace.samples.size() - 1));
        });
    }

    void run() throws IOException {
        System.out.println(diagnostic());
        try {
            result = runner.run();
            trace.save("Completed; ticks=" + result.ticks() + "; endedEarly=" + result.endedBeforeStop()
                    + "; warnings=" + result.warnings());
        } catch (RuntimeException | Error failure) {
            trace.save("FAILED: " + failure);
            throw failure;
        }
    }

    String diagnostic() { return "Trace: " + trace.directory; }
}
