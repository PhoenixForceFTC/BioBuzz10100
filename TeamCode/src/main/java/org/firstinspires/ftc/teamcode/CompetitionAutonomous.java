package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

/** Combined match adapter. All decisions are non-blocking and hardware is owned by subsystems. */
public abstract class CompetitionAutonomous extends LinearOpMode {
    protected abstract TurretSubsystem.Alliance alliance();
    @Override public void runOpMode() {
        RobotLoop loop = new RobotLoop(hardwareMap);
        AutonomousStrategy strategy = AutonomousStrategy.configured(alliance());
        if (strategy == null || !strategy.approved) {
            while (opModeInInit()) {
                loop.begin();
                telemetry.addData("Autonomous inhibited", "Configure an approved %s route in AutonomousStrategy", alliance());
                loop.addTelemetry(telemetry); telemetry.update(); idle();
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
        IntakeQueueController intakeControl = new IntakeQueueController();
        try {
            PinpointOdometry odometry = new PinpointOdometry(hardwareMap);
            intake = new IntakeSubsystem(hardwareMap); transfer = new TransferSubsystem(hardwareMap);
            gate = new GateSubsystem(hardwareMap); turret = new TurretSubsystem(hardwareMap);
            ShotCoordinator shots = new ShotCoordinator(turret, strategy.target, ShotCoordinator.AimMode.AUTONOMOUS_ODOMETRY);
            // Collection camera failure is bounded by the strategy's collection timeout.
            try { vision = new PollenVision(hardwareMap); }
            catch (RuntimeException e) { telemetry.addData("Collection camera unavailable", e.getMessage()); }
            boolean referenced = false;
            while (opModeInInit()) {
                loop.begin();
                odometry.update(); turret.update();
                if (!referenced && odometry.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) {
                    odometry.setPoseInches(strategy.start.x / .0254, strategy.start.y / .0254, Math.toDegrees(strategy.start.heading));
                    referenced = true;
                }
                telemetry.addData("Strategy", strategy.name); turret.addTelemetry(telemetry);
                if (vision != null) vision.addTelemetry(telemetry);
                loop.addTelemetry(telemetry); telemetry.update(); idle();
            }
            waitForStart(); if (isStopRequested()) return;
            auto.start(System.nanoTime() * 1e-9);
            while (opModeIsActive() && !auto.finished()) {
                loop.begin();
                odometry.update(); turret.update();
                double now = System.nanoTime() * 1e-9;
                shots.updatePose(now, odometry.getSample());
                MotionSample pose = shots.pose();
                auto.update(now, pose, vision == null ? null : vision.getTarget(),
                        vision != null && vision.isAutonomousReady(), transfer.isFull(), isStopRequested());
                shots.update(now, odometry.getSample(), !auto.finished(), auto.fireRequested(), false, auto.finished(), false);
                drive.driveRobotCentric(auto.axial(), auto.lateral(), auto.yaw());
                gate.setOpen(shots.firing().gateOpen());
                transfer.update(shots.firing().feeding() && gate.isOpenForFeeding(), !auto.loading() || auto.finished());
                intake.setIntakePower(auto.finished() ? 0 : intakeControl.update(transfer.isFull(),
                        gamepad1.right_bumper || gamepad2.right_bumper,
                        gamepad1.left_bumper || gamepad2.left_bumper));
                if (auto.finished()) transfer.stop();
                telemetry.addData("Match autonomous", "%s | %s", auto.state(), auto.reason());
                shots.addTelemetry(telemetry, now); if (vision != null) vision.addTelemetry(telemetry);
                loop.addTelemetry(telemetry); telemetry.update(); idle();
            }
        } finally {
            auto.stop(System.nanoTime() * 1e-9);
            RobotShutdown.stopAll(drive::stop, intake == null ? null : intake::stop,
                    transfer == null ? null : transfer::stop, gate == null ? null : gate::close,
                    turret == null ? null : turret::stop, vision == null ? null : vision::close);
        }
    }
}
