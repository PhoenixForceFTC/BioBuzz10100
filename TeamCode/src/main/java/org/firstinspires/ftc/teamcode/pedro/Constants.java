package org.firstinspires.ftc.teamcode.pedro;

// If an import is red, click it and press Alt+Enter so Android Studio finds the
// right package for your exact Pedro 3.x version. Packages moved between 2.x and 3.x.
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro Pathing 3.x constants (mecanum drive + goBILDA Pinpoint + Foresight).
 *
 * Pedro 3 has THREE parts that you hand to the Follower:
 *   1. Drivetrain  - how the wheels are driven       (MecanumConfig)
 *   2. Localizer   - how the robot knows where it is  (PinpointConfig)
 *   3. Algorithm   - how it follows paths             (Foresight + ForesightConfig)
 *
 * There is no FollowerConstants or FollowerBuilder anymore. The old separate values
 * (mass, zero-power acceleration, translational/drive PID, centripetal, xVelocity,
 * yVelocity) are gone. Foresight replaces them, and you get its numbers by running
 * the Foresight tuner.
 *
 * KEEP THE FIELD NAMES drivetrainConfig, localizerConfig and foresightConfig.
 * The Quickstart's Tuning.java refers to them by these names (for example
 * new Mecanum(hardwareMap, Constants.drivetrainConfig)). Renaming them breaks
 * the tuners.
 *
 * Foresight's config values are REQUIRED and have no defaults. Do not guess them.
 * Run the Foresight tuner on the robot and paste in what it reports.
 *
 * EVERY NUMBER BELOW IS A PLACEHOLDER. Tags:
 *   [MEASURE] you measure or choose it
 *   [TUNER]   a tuner in Tuning.java gives you the value
 *
 * Tune in this order: Mecanum -> Pinpoint -> Foresight -> Tests.
 */
public class Constants {


    // =====================================================================
    // 1. DRIVETRAIN CONFIG (Mecanum)
    // =====================================================================
    public static MecanumConfig drivetrainConfig = new MecanumConfig(cfg -> {

        // [MEASURE] Motor names. Must match your Driver Station robot
        // configuration exactly, including capitalization.
        cfg.frontLeftName.set("leftfront");
        cfg.frontRightName.set("rightfront");
        cfg.backLeftName.set("leftback");
        cfg.backRightName.set("rightback");

        // [MEASURE] Motor directions. Typical mecanum: reverse the left side.
        // If the robot drives the wrong way or spins when told to go forward,
        // flip the wrong ones. The Mecanum tuner also checks this for you.
        // (Field names below are my best guess. Use autocomplete on `cfg.` to
        //  see the exact names in your version.)
        cfg.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        cfg.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        cfg.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        cfg.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });


    // =====================================================================
    // 2. LOCALIZER CONFIG (goBILDA Pinpoint)
    // =====================================================================
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(2.491578830508735);
        c.yPodOffset.set(0.38921570214699575);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });


    // =====================================================================
    // 3. ALGORITHM CONFIG (Foresight)
    // =====================================================================
    // [TUNER] This is where the Foresight tuner's results go. Foresight is the
    // path-following algorithm that replaces the old PIDF drive/translational/
    // centripetal setup. Until the tuner has run and you have pasted its values
    // in, the algorithm has no real values and paths should not be run.
    //
    // Fields mentioned in the library's own example include headingFeedback,
    // forwardTranslational, strafeTranslational, brake and coast. Paste
    // exactly what the tuner prints, using the same field names.
    //
    // There is also a maxPathSpeed field for capping speed on a path
    // (Pedro 2's per-call max power was replaced by this).
//    public static ForesightConfig foresightConfig = new ForesightConfig(
//            c -> {
//                Controller primaryTranslationalForward = Controller.proportional(0.1939061835845785);
//                Controller secondaryTranslationalForward = Controller.proportional(0.07164315356585159);
//                Controller primaryTranslationalLateral = Controller.proportional(0.2797286549034449);
//                Controller secondaryTranslationalLateral = Controller.proportional(0.10335226349950427);
//
//                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
//                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));
//
//                c.coast.set(Controller.proportionalFeedforward(0.01180888061594451));
//                c.brake.set(Controller.proportionalFeedforward(0.010037548523552833));
//
//                c.headingFeedback.set(Controller.proportional(4.354911614841229));
//                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0542834729398866, 0.0037762477619278107));
//
//                c.linearBrakeCoefficients.set(Matrix.diag(0.0543966947994119, 0.04611324630481297));
//                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0013323270518762142, 0.0014813980212008047));
//
//                c.maxAchievableForwardVelocity.set(84.06139971218252);
//                c.maxAchievableStrafeVelocity.set(70.77209341393909);
//                c.naturalForwardDeceleration.set(47.2070671893997);
//                c.naturalStrafeDeceleration.set(287.2913759356076);
//            }
//    );
//    public static ForesightConfig foresightConfig = new ForesightConfig(cfg -> {
//        // Example shape only, do NOT invent numbers:
//        // cfg.maxPathSpeed.set(1.0);
//        // ...paste tuner output here...
//    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.17818800870799986);
                Controller secondaryTranslationalForward = Controller.proportional(0.06583570794632368);
                Controller primaryTranslationalLateral = Controller.proportional(0.2510363789129798);
                Controller secondaryTranslationalLateral = Controller.proportional(0.09275123419276186);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01088185300350903));
                c.brake.set(Controller.proportionalFeedforward(0.009249575052982675));

                c.headingFeedback.set(Controller.proportional(3.682961377965725));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.049407578969023064, 0.004762383128154276));

                c.linearBrakeCoefficients.set(Matrix.diag(0.062437470132476094, 0.07499558943245356));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0012681551265074729, 0.0010011574586482966));

                c.maxAchievableForwardVelocity.set(90.22233270342807);
                c.maxAchievableStrafeVelocity.set(75.92182464502555);
                c.naturalForwardDeceleration.set(32.09493774800579);
                c.naturalStrafeDeceleration.set(58.581831817861016);
            }
    );

    // =====================================================================
    // 4. BUILD THE FOLLOWER
    // =====================================================================
    // Call Constants.create(hardwareMap) from your OpModes.
    public static Follower create(HardwareMap h) {
        // The three parts you hand to the Follower.
        Mecanum drivetrain = new Mecanum(h, drivetrainConfig);                   // how wheels are driven
        PinpointLocalizer localizer = new PinpointLocalizer(h, localizerConfig); // where the robot is
        Foresight algorithm = new Foresight(foresightConfig);                    // how paths are followed

        // Argument order matters. The types are different, so the wrong order
        // shows up as red errors on the drivetrain and localizer. The library's own
        // example passes the LOCALIZER first, then the drivetrain, then the algorithm.
        // If this line is still red, hover over it: the error lists the exact
        // constructor signature to match, so reorder to that.
        return new Follower(localizer, drivetrain, algorithm);
    }
}