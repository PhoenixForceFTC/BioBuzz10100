package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Tests the competition aiming coordinator with Limelight, Pinpoint, and turret hardware.
 * Push the chassis by hand; drivetrain, gate, transfer, and intake are not initialized.
 * GP1: hold RT to aim, hold RB with RT to spin the flywheel, A to rehome the turret,
 * B/X to select Red/Blue; the nearest cell is selected as in competition.
 * Uses measured ShootingConfiguration values and preserves all calibration checks.
 */
@TeleOp(name = "Test: Auto Aim", group = "Test")
public class AutoAimTestOpMode extends LinearOpMode {

    private static final double TRIGGER_DEADBAND = 0.1;

    @Override
    public void runOpMode() {
        RobotLoop loop = new RobotLoop(hardwareMap);
        PinpointOdometry odometry = new PinpointOdometry(hardwareMap);
        TurretSubsystem turret = new TurretSubsystem(hardwareMap);
        try {
            turret.setEncoderCorrectionEnabled(true);
            ShotCoordinator shots = new ShotCoordinator(turret,
                    TeleOpFieldReference.target(TurretSubsystem.Alliance.RED, TurretSubsystem.HiveSide.FAR),
                    ShotCoordinator.AimMode.TELEOP_ODOMETRY);
            boolean previousHome = false;
            while (opModeInInit()) {
                loop.begin();
                odometry.update();
                selectTarget(turret, shots);
                if (gamepad1.a && !previousHome) turret.homeTurretAndZeroEncoder();
                previousHome = gamepad1.a;
                turret.update();
                double now = System.nanoTime() * 1e-9;
                shots.update(now, odometry.getSample(), false, false, true, true);
                turret.applyYawCommand();
                addTelemetry(odometry, turret, shots, now, false);
                loop.addTelemetry(telemetry); telemetry.update();
                idle();
            }
            waitForStart();
            if (isStopRequested()) return;

            double previousTime = System.nanoTime() * 1e-9;
            while (opModeIsActive()) {
                loop.begin();
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
                turret.updateCompetitionManualYaw(gamepad1.right_stick_x, Math.min(.1, now - previousTime), aimHeld);
                previousTime = now;
                // Exercise automatic aim, but inhibit feeding and require an explicit flywheel hold.
                shots.update(now, odometry.getSample(), aimHeld, false, true, !spinFlywheel);
                turret.applyYawCommand(); // Same-loop servo write of the aim/manual target.
                if (!spinFlywheel || !shots.solution().aimable) turret.stopShooter();

                addTelemetry(odometry, turret, shots, now, aimHeld);
                loop.addTelemetry(telemetry); telemetry.update();
            }
        } finally {
            turret.stop();
        }
    }

    private void selectTarget(TurretSubsystem turret, ShotCoordinator shots) {
        TurretSubsystem.Alliance alliance = gamepad1.b ? TurretSubsystem.Alliance.RED
                : gamepad1.x ? TurretSubsystem.Alliance.BLUE : turret.getActiveAlliance();
        if (alliance != turret.getActiveAlliance()) {
            shots.select(TeleOpFieldReference.target(alliance, turret.getActiveHiveSide()));
        }
    }

    private void addTelemetry(PinpointOdometry odometry, TurretSubsystem turret,
                              ShotCoordinator shots, double now, boolean aiming) {
        telemetry.addLine("RT = Auto Aim | RT + RB = Aim + Flywheel | A = Rehome Turret");
        telemetry.addLine("B/X = Red/Blue | Nearest cell automatically | RS-X = Manual yaw with RT released");
        telemetry.addLine("Competition localization/aim mode; acquire a mapped raised tag. Push chassis by hand.");
        telemetry.addData("Auto Aim", aiming ? "REQUESTED" : "RELEASED");
        turret.addTelemetry(telemetry);
        odometry.addTelemetry(telemetry);
        shots.addTelemetry(telemetry, now);
    }
}
