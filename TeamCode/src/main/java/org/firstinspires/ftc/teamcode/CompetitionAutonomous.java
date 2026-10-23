package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

/** Combined match adapter. All decisions are non-blocking and hardware is owned by subsystems. */
public abstract class CompetitionAutonomous extends LinearOpMode {
    protected abstract TurretSubsystem.Alliance alliance();
    @Override public void runOpMode() {
        AutonomousStrategy strategy = AutonomousStrategy.configured(alliance());
        if (strategy == null || !strategy.approved) {
            while (opModeInInit()) {
                telemetry.addData("Autonomous inhibited", "Configure an approved %s route in AutonomousStrategy", alliance());
                telemetry.update(); idle();
            }
            return;
        }
        MecanumDrivetrain drive = new MecanumDrivetrain(hardwareMap);
        IntakeSubsystem intake = null;
        TransferSubsystem transfer = null;
        GateSubsystem gate = null;
        TurretSubsystem turret = null;
        PollenVision vision = null;
        CompetitionAutonomousController auto = new CompetitionAutonomousController(strategy, PollenVision.CONFIG);
        try {
            PinpointOdometry odometry = new PinpointOdometry(hardwareMap);
            intake = new IntakeSubsystem(hardwareMap); transfer = new TransferSubsystem(hardwareMap);
            gate = new GateSubsystem(hardwareMap); turret = new TurretSubsystem(hardwareMap);
            ShotCoordinator shots = new ShotCoordinator(turret, strategy.target);
            // Collection camera failure is bounded by the strategy's collection timeout.
            try { vision = new PollenVision(hardwareMap); }
            catch (RuntimeException e) { telemetry.addData("Collection camera unavailable", e.getMessage()); }
            boolean referenced = false;
            while (opModeInInit()) {
                odometry.update(); turret.update();
                if (!referenced && odometry.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) {
                    odometry.setPoseInches(strategy.start.x / .0254, strategy.start.y / .0254, Math.toDegrees(strategy.start.heading));
                    referenced = true;
                }
                telemetry.addData("Strategy", strategy.name); turret.addTelemetry(telemetry);
                if (vision != null) vision.addTelemetry(telemetry);
                telemetry.update(); idle();
            }
            waitForStart(); if (isStopRequested()) return;
            auto.start(System.nanoTime() * 1e-9);
            while (opModeIsActive() && !auto.finished()) {
                odometry.update(); turret.update();
                double now = System.nanoTime() * 1e-9;
                shots.updatePose(now, odometry.getSample());
                MotionSample pose = shots.pose();
                auto.update(now, pose, vision == null ? null : vision.getTarget(),
                        vision != null && vision.isAutonomousReady(), transfer.isFull(), isStopRequested());
                shots.update(now, odometry.getSample(), auto.fireRequested(), false, auto.finished(), false);
                drive.driveRobotCentric(auto.axial(), auto.lateral(), auto.yaw()); intake.setIntakePower(auto.intake());
                gate.setOpen(shots.firing().gateOpen());
                transfer.update(shots.firing().feeding(), !auto.loading() || auto.finished());
                if (auto.finished()) transfer.stop();
                telemetry.addData("Match autonomous", "%s | %s", auto.state(), auto.reason());
                shots.addTelemetry(telemetry, now); if (vision != null) vision.addTelemetry(telemetry);
                telemetry.update(); idle();
            }
        } finally {
            auto.stop(System.nanoTime() * 1e-9); drive.stop();
            if (intake != null) intake.stop(); if (transfer != null) transfer.stop();
            if (gate != null) gate.close(); if (turret != null) turret.stop();
            if (vision != null) vision.close();
        }
    }
}
