package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.hardware.Gamepad;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import org.horizon36596.simloop.opmode.OpModePhase;
import org.horizon36596.simloop.opmode.RecordingTelemetry;

/** Primitive decoded data crosses the sim class-loader boundary; retained for both pass and failure. */
final class TeleOpTrace {
    static final class Sample {
        OpModePhase phase;
        int tick;
        double seconds;
        String input, telemetry;
        final double[] powers = new double[4];
        final double[] shaftTicksPerSecond = new double[4];
        double xIn, yIn, headingRad, gatePosition, gatePhysical, gatePulseUs;
        boolean feedAllowed;

        String decoded() {
            return phase + "\t" + tick + "\t" + seconds + "\t" + input + "\t"
                    + java.util.Arrays.toString(powers) + "\t" + java.util.Arrays.toString(shaftTicksPerSecond)
                    + "\t" + xIn + "\t" + yIn + "\t" + headingRad + "\t" + gatePosition
                    + "\t" + gatePhysical + "\t" + gatePulseUs + "\t" + feedAllowed + "\t" + telemetry;
        }
    }

    final List<Sample> samples = new ArrayList<>();
    final Path directory;
    private String input;
    private OpModePhase phase;
    private int tick;

    TeleOpTrace(String name, String opMode, double tickSeconds) throws IOException {
        String root = System.getProperty("biobuzz.sim.outputDir");
        if (root == null) throw new IllegalStateException("Run with :TeamCode:simTeleOpTest");
        Files.createDirectories(Paths.get(root));
        directory = Files.createTempDirectory(Paths.get(root), name + "-");
        Files.write(directory.resolve("scenario.txt"), ("OpMode=" + opMode + "\ntickSeconds=" + tickSeconds
                + "\n" + BioBuzzSimModel.PARAMETERS + "\n").getBytes(StandardCharsets.UTF_8));
    }

    void input(OpModePhase phase, int tick, Gamepad gp1, Gamepad gp2) {
        this.phase = phase;
        this.tick = tick;
        input = "GP1=" + gp1.left_stick_y + "," + gp1.left_stick_x + "," + gp1.right_stick_x
                + ",LT=" + gp1.left_trigger + ",RB=" + gp1.right_bumper + ",XYAB="
                + gp1.x + "," + gp1.y + "," + gp1.a + "," + gp1.b;
    }

    void sample(BioBuzzSimHardware hardware, BioBuzzSimModel model, double tickSeconds,
                RecordingTelemetry telemetry) {
        Sample s = new Sample();
        s.phase = phase; s.tick = tick; s.seconds = (samples.size() + 1) * tickSeconds; s.input = input;
        for (int i = 0; i < 4; i++) if (hardware.wheels[i] != null) {
            s.powers[i] = hardware.wheels[i].getPower();
            s.shaftTicksPerSecond[i] = hardware.wheels[i].getPhysicalVelocityTicksPerSecond();
        }
        if (model.drive != null) {
            s.xIn = model.drive.getX(); s.yIn = model.drive.getY(); s.headingRad = model.drive.getHeading();
        }
        if (hardware.gate != null) {
            s.gatePosition = hardware.gate.getPosition(); s.gatePhysical = hardware.gate.getScaledPosition();
            s.gatePulseUs = hardware.gate.getPulseWidth();
        }
        s.telemetry = String.join(" | ", telemetry.lastTransmission());
        s.feedAllowed = telemetry.lastTransmission().contains("Gate Feed Allowed : true");
        samples.add(s);
    }

    Sample at(OpModePhase phase, int tick) {
        return samples.stream().filter(s -> s.phase == phase && s.tick == tick).findFirst()
                .orElseThrow(() -> new AssertionError("Missing " + phase + " tick " + tick + "; trace " + directory));
    }

    List<String> decoded() {
        List<String> rows = new ArrayList<>();
        for (Sample s : samples) rows.add(s.decoded());
        return rows;
    }

    void save(String outcome) throws IOException {
        List<String> rows = new ArrayList<>();
        rows.add("phase\ttick\tseconds\tinput\twheelPowers\tshaftTicksPerSecond\txIn\tyIn\theadingRad"
                + "\tgateLogical\tgatePhysical\tgatePulseUs\tfeedAllowed\ttelemetry");
        rows.addAll(decoded());
        Files.write(directory.resolve("trace.tsv"), rows, StandardCharsets.UTF_8);
        Files.write(directory.resolve("runner-outcome.txt"), (outcome + "\nLast requested phase=" + phase
                + ",tick=" + tick + ",input=" + input + "\nAssertion results: see Gradle JUnit XML/HTML reports.\n")
                .getBytes(StandardCharsets.UTF_8));
    }
}
