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
//todo: create a way so there is precise start position. also make a way so that the robot's odometry is consistent on the robot, also make a robot model, instead of arrow, robot measurements...
// also, the flowers are centered along side barriers, but should be sticking into the filed.
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

    public static Mecanum drivetrain;


    // =====================================================================
    // 1. DRIVETRAIN CONFIG (Mecanum)
    // =====================================================================
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftfront");
        c.frontRightName.set("rightfront");
        c.backLeftName.set("leftback");
        c.backRightName.set("rightback");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    // =====================================================================
    // 2. LOCALIZER CONFIG (goBILDA Pinpoint)
    // =====================================================================
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(1.8469447038305087);
        c.yPodOffset.set(1.3003265200637457);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
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
                Controller primaryTranslationalForward = Controller.proportional(0.3100178262135112);
                Controller secondaryTranslationalForward = Controller.proportional(0.11454330295701048);
                Controller primaryTranslationalLateral = Controller.proportional(0.4855766023499335);
                Controller secondaryTranslationalLateral = Controller.proportional(0.17940757972252447);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.012715273265700046));
                c.brake.set(Controller.proportionalFeedforward(0.010807982275845038));

                c.headingFeedback.set(Controller.proportional(5.363935529565826));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.050404207300238334, 0.00662826876091725));

                c.linearBrakeCoefficients.set(Matrix.diag(0.10190028205033407, 0.04863765333157554));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0015098880451388642, 0.002147552414866776));

                c.maxAchievableForwardVelocity.set(75.791216731181);
                c.maxAchievableStrafeVelocity.set(60.54276069289256);
                c.naturalForwardDeceleration.set(44.33338627123392);
                c.naturalStrafeDeceleration.set(79.85589496541019);
            }
    );
    // =====================================================================
    // 4. BUILD THE FOLLOWER
    // =====================================================================
    // Call Constants.create(hardwareMap) from your OpModes.
    public static Follower create(HardwareMap h) {
        // The three parts you hand to the Follower.
        drivetrain = new Mecanum(h, drivetrainConfig);                   // how wheels are driven
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
