package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Read-only camera diagnostic. No turret, shooter, gate, drive or transfer is initialized. */
@TeleOp(name = "Test: Limelight Goal State 3D", group = "Test")
public class GoalStateTestOpMode extends LinearOpMode {
    @Override public void runOpMode() {
        RobotLoop loop = new RobotLoop(hardwareMap);
        GoalStateObserver observer = new GoalStateObserver(ShootingConfiguration.MOUNT,ShootingConfiguration.LOWERED_TAG_HEIGHT_METERS,ShootingConfiguration.LOWERED_TAG_PLANE_DEG);
        TurretSubsystem.Alliance alliance = TurretSubsystem.Alliance.RED;
        TurretSubsystem.HiveSide side = TurretSubsystem.HiveSide.FAR;
        boolean previousRed = false, previousBlue = false, previousLeft = false, previousRight = false;
        try (LimelightTagSubsystem camera = new LimelightTagSubsystem(hardwareMap)) {
            while (opModeInInit() || opModeIsActive()) {
                loop.begin();
                if (gamepad1.b && !previousRed) alliance = TurretSubsystem.Alliance.RED;
                if (gamepad1.x && !previousBlue) alliance = TurretSubsystem.Alliance.BLUE;
                if (gamepad1.dpad_left && !previousLeft) side = TurretSubsystem.HiveSide.AUDIENCE;
                if (gamepad1.dpad_right && !previousRight) side = TurretSubsystem.HiveSide.FAR;
                previousRed = gamepad1.b; previousBlue = gamepad1.x;
                previousLeft = gamepad1.dpad_left; previousRight = gamepad1.dpad_right;
                observer.select(alliance,side); camera.update();
                double now = System.nanoTime()*1e-9;
                LimelightObservation frame = new LimelightObservation(now,System.currentTimeMillis(),camera.getResult());
                observer.update(now,frame.frame,frame.captureTime,frame.tags,ShootingConfiguration.LEVEL_CHASSIS_ASSUMED);
                telemetry.addData("Selected cell", "%s / %s",alliance,side);
                telemetry.addData("Observed goal state",observer.state(now));
                telemetry.addData("Height / tag plane (m, deg)", "%.3f | %.1f",observer.heightMeters(),observer.planeDegrees());
                telemetry.addData("Capture age (s)",observer.age(now));
                telemetry.addData("Decision",observer.reason());
                for (TagObservation tag : frame.tags) telemetry.addData("Metric tag " + tag.id,
                        "right %.3f down %.3f forward %.3f m | area %.3f%% | usable %s",tag.x,tag.y,tag.z,tag.area,tag.usable());
                telemetry.addLine("B/X Red/Blue | Dpad L/R Audience/Far | Assumes LEVEL chassis; tolerances UNVALIDATED");
                telemetry.addLine("Camera requirements UNVERIFIED: 36h11, 82.55 mm individual tags, Full 3D enabled.");
                loop.addTelemetry(telemetry); telemetry.update(); idle();
            }
        }
    }
}
