package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/** Layer 3: collect visible pollen with the C270; stop on full queue or a navigation limit. */
@Autonomous(name = "Auto: Pollen Collection", group = "Competition")
public class PollenAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotLoop loop = new RobotLoop(hardwareMap);
        MecanumDrivetrain drivetrain = new MecanumDrivetrain(hardwareMap);
        PinpointOdometry odometry;
        IntakeSubsystem intake = null;
        GateSubsystem gate = null;
        TransferSubsystem transfer = null;
        PollenNavigator navigator = new PollenNavigator(PollenVision.CONFIG);
        PollenVision vision = null;

        try {
            odometry = new PinpointOdometry(hardwareMap);
            intake = new IntakeSubsystem(hardwareMap);
            gate = new GateSubsystem(hardwareMap);
            transfer = new TransferSubsystem(hardwareMap);
            vision = new PollenVision(hardwareMap);
            while (opModeInInit()) {
                loop.begin();
                odometry.update();
                telemetry.addLine("Keep robot stationary during Pinpoint calibration.");
                telemetry.addLine("C270 must face forward; clear collection area before START.");
                telemetry.addData("Pinpoint Status", odometry.getDeviceStatus());
                vision.addTelemetry(telemetry);
                loop.addTelemetry(telemetry); telemetry.update();
                idle();
            }
            waitForStart();
            if (isStopRequested()) {
                return;
            }
            Pose2D initialPose = odometry.getPose();
            navigator.start(System.nanoTime(), initialPose.getX(DistanceUnit.MM), initialPose.getY(DistanceUnit.MM));

            while (opModeIsActive()) {
                loop.begin();
                odometry.update(); // One bulk read at the top of each loop.
                Pose2D pose = odometry.getPose();
                PollenTarget target = vision.getTarget();
                boolean hardwareReady = odometry.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY
                        && vision.isAutonomousReady();
                long nowNanos = System.nanoTime();
                // Gate stays closed. Stage 1 loads only while seeking/collecting; stage 2 stays stopped.
                transfer.update(false, !hardwareReady || navigator.isFinished());
                navigator.update(nowNanos, target, pose.getX(DistanceUnit.MM),
                        pose.getY(DistanceUnit.MM), pose.getHeading(AngleUnit.DEGREES), hardwareReady, transfer.isFull());
                drivetrain.driveRobotCentric(navigator.getAxialPower(), 0.0, navigator.getYawPower());
                intake.setIntakePower(navigator.getIntakePower());
                if (navigator.isFinished()) {
                    transfer.stop();
                }

                telemetry.addData("Pollen Auto", navigator.getState());
                telemetry.addData("Navigation", navigator.getStatus());
                telemetry.addData("Auto Drive", "forward %.2f | clockwise %.2f",
                        navigator.getAxialPower(), navigator.getYawPower());
                telemetry.addData("Auto Travel (mm)", "%.0f / %.0f", navigator.getTotalTravelMm(), PollenNavigator.MAX_TOTAL_TRAVEL_MM);
                telemetry.addData("Pickup Attempts (not ball count)", navigator.getPickupAttempts());
                vision.addTelemetry(telemetry);
                transfer.addTelemetry(telemetry);
                loop.addTelemetry(telemetry); telemetry.update();
                if (navigator.isFinished()) {
                    break;
                }
                idle();
            }
        } finally {
            RobotShutdown.stopAll(drivetrain::stop, intake == null ? null : intake::stop,
                    transfer == null ? null : transfer::stop, gate == null ? null : gate::close,
                    vision == null ? null : vision::close);
        }
    }
}
