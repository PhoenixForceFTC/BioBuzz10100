package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Tests the competition aiming coordinator with Limelight, Pinpoint, and turret hardware.
 * Push the chassis by hand; drivetrain, gate, transfer, and intake are not initialized.
 * GP1: hold RT to aim, hold RB with RT to spin the flywheel, A to rehome the turret,
 * B/X to select Red/Blue, and D-pad Left/Right to select Audience/Far.
 * Uses measured ShootingConfiguration values and preserves all calibration checks.
 */
@TeleOp(name = "Test: Auto Aim", group = "Test")
public class AutoAimTestOpMode extends LinearOpMode {

    private static final double TRIGGER_DEADBAND = 0.1;

    @Override
    public void runOpMode() {
        PinpointOdometry odometry = new PinpointOdometry(hardwareMap);
        TurretSubsystem turret = new TurretSubsystem(hardwareMap);
        try {
            ShotCoordinator shots = new ShotCoordinator(turret,
                    ShootingConfiguration.target(TurretSubsystem.Alliance.RED, TurretSubsystem.HiveSide.FAR));
            boolean startingPoseApplied = false;
            while (opModeInInit()) {
                odometry.update();
                if (!startingPoseApplied && ShootingConfiguration.STARTING_FIELD_POSE_CONFIGURED
                        && odometry.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) {
                    odometry.setPoseInches(ShootingConfiguration.START_X_METERS / 0.0254,
                            ShootingConfiguration.START_Y_METERS / 0.0254,
                            Math.toDegrees(ShootingConfiguration.START_HEADING_RAD));
                    startingPoseApplied = true;
                }
                selectTarget(turret, shots);
                turret.update();
                double now = System.nanoTime() * 1e-9;
                shots.update(now, odometry.getSample(), false, false, true, true);
                addTelemetry(odometry, turret, shots, now, false);
                telemetry.update();
                idle();
            }
            waitForStart();
            if (isStopRequested()) return;

            boolean previousHome = false;
            while (opModeIsActive()) {
                odometry.update();
                selectTarget(turret, shots);
                boolean home = gamepad1.a;
                if (home && !previousHome) {
                    turret.homeTurretAndZeroEncoder();
                }
                previousHome = home;
                turret.update();

                boolean aimHeld = gamepad1.right_trigger > TRIGGER_DEADBAND;
                boolean spinFlywheel = aimHeld && gamepad1.right_bumper;
                double now = System.nanoTime() * 1e-9;
                // Exercise automatic aim, but inhibit feeding and require an explicit flywheel hold.
                shots.update(now, odometry.getSample(), aimHeld, false, true, !spinFlywheel);
                if (!spinFlywheel || !shots.solution().aimable) turret.stopShooter();

                addTelemetry(odometry, turret, shots, now, aimHeld);
                telemetry.update();
            }
        } finally {
            turret.stop();
        }
    }

    private void selectTarget(TurretSubsystem turret, ShotCoordinator shots) {
        TurretSubsystem.Alliance alliance = gamepad1.b ? TurretSubsystem.Alliance.RED
                : gamepad1.x ? TurretSubsystem.Alliance.BLUE : turret.getActiveAlliance();
        TurretSubsystem.HiveSide side = gamepad1.dpad_left ? TurretSubsystem.HiveSide.AUDIENCE
                : gamepad1.dpad_right ? TurretSubsystem.HiveSide.FAR : turret.getActiveHiveSide();
        if (alliance != turret.getActiveAlliance() || side != turret.getActiveHiveSide()) {
            shots.select(ShootingConfiguration.target(alliance, side));
        }
    }

    private void addTelemetry(PinpointOdometry odometry, TurretSubsystem turret,
                              ShotCoordinator shots, double now, boolean aiming) {
        telemetry.addLine("RT = Auto Aim | RT + RB = Aim + Flywheel | A = Rehome Turret");
        telemetry.addLine("B/X = Red/Blue | Dpad L/R = Audience/Far | Push chassis by hand");
        telemetry.addLine("Uses ShootingConfiguration measured mount, target, profile, timing, and field pose.");
        telemetry.addData("Auto Aim", aiming ? "REQUESTED" : "RELEASED");
        turret.addTelemetry(telemetry);
        odometry.addTelemetry(telemetry);
        shots.addTelemetry(telemetry, now);
    }
}
