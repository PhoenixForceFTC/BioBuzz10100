package org.firstinspires.ftc.teamcode.pedro;

// If an import is still red, press Alt+Enter and let Android Studio find the
// right package. The AutoTune module's real top-level package is
// com.pedropathing.tuning.autotune (confirmed via the official Javadoc), not
// com.pedropathing.autotune — that was the likely cause if everything below
// was unresolved before.
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

import com.pedropathing.algorithm.Foresight;

/**
 * Registers each AutoTune procedure so it shows up at
 * http://192.168.43.1:10158 when the robot is connected.
 *
 * Run them in this fixed order, since each depends on the one before it:
 *   Mecanum -> Pinpoint -> Foresight -> Tests
 *
 * CONFIRMED against pedropathing.com docs: pinpointTuner() and tests().
 * NOT YET CONFIRMED verbatim: mecanumTuner() — follows the same naming
 * pattern as the others but hasn't been checked against a doc snippet.
 */
public class Tuning {

    // =====================================================================
    // 1. MECANUM TUNER — run first
    //    Confirms motor names/directions in Constants.drivetrainConfig.
    // =====================================================================
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    // =====================================================================
    // 2. PINPOINT TUNER — run second
    //    Finds pod type, offsets, and directions. Confirmed doc snippet.
    //    Paste its generated PinpointConfig into Constants.localizerConfig.
    // =====================================================================
    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    // =====================================================================
    // 3. FORESIGHT TUNER — run third, only after Mecanum + Pinpoint are done
    //    Produces the 12 required values for Constants.foresightConfig.
    //    Give the robot a LOT of open space — it can travel much farther
    //    than the configured test distance suggests.
    //
    //    Argument order is (localizer, drivetrain) — the OPPOSITE order
    //    from Tests below. Confirmed by the compiler: it named the expected
    //    types Localizer then Drivetrain for these two slots when they were
    //    given in the other order.
    // =====================================================================
    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner(
                hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig)
        );
    }

    // =====================================================================
    // 4. TESTS — run last, to verify the whole tuned Follower
    //    Confirmed doc snippet. Includes a Hold Test that checks how well
    //    Foresight corrects the robot when pushed off course.
    //
    //    NOTE: this one is (drivetrain, localizer, algorithm) order — check
    //    it still compiles now that Foresight's own order is known to be
    //    reversed. If Tests also complains, swap its first two the same way.
    // =====================================================================
    @Tuner
    public static Procedure tests() {
        return new Tests(
                hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig),
                hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                () -> new Foresight(Constants.foresightConfig)
        );
    }
}

/*public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
    c.name.set("pinpoint");
    c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
    c.xPodOffset.set(8.274738582100456);
    c.yPodOffset.set(-0.3386981843963383);
    c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    c.globalDistanceUnit.set(DistanceUnit.INCH);
    c.offsetUnits.set(DistanceUnit.INCH);
});

 */