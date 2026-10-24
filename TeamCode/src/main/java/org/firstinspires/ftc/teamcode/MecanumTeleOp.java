package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Layer 3 Combined TeleOp: Coordinates {@link MecanumDrivetrain}, {@link PinpointOdometry},
 * {@link IntakeSubsystem}, {@link TransferSubsystem}, {@link GateSubsystem}, {@link DepollinatorSubsystem},
 * and {@link TurretSubsystem} (rotation servos on ports 1 & 2, 108:40 REV Through-Bore Encoder
 * on quad port 1, 2:1 pitch servo on port 0 from 0-30 deg downward travel from 0.5 home (+15 to -15 deg launch elevation), 6000 RPM flywheel motor on port 0,
 * and Limelight 3A angled 30 deg toward the goal).
 *
 * <p>Controls (Gamepad 1 — Driver + Turret Assist, or Gamepad 2 — Dedicated Turret Operator):
 * <ul>
 *     <li><b>Gamepad 1 Left Stick</b>: Forward / Backward & Strafe translation</li>
 *     <li><b>Gamepad 1 Right Stick X</b>: Chassis rotation</li>
 *     <li><b>Gamepad 1 Right / Left Stick Click</b>: Deploy depollinator 135 deg CCW / return to 30 deg home</li>
 *     <li><b>Gamepad 1 Right Trigger (Hold)</b>: Auto-aim, hood/RPM adjustment, automatic gate and feed when ready</li>
 *     <li><b>Gamepad 1 Left Trigger (Hold)</b>: Manually open gate and feed after gate travel</li>
 *     <li><b>Gamepad 2 Right Trigger (Hold)</b>: Manually open gate and feed after gate travel</li>
 *     <li><b>Gamepad 2 Left Trigger (Hold)</b>: Force gate closed and stop both transfer stages; overrides firing</li>
 *     <li><b>Gamepad 2 B (Hold)</b>: Stop flywheel; release restores manual or automatic RPM</li>
 *     <li><b>Gamepad 1/2 Right / Left Bumper</b>: Intake forward/reverse, always overriding automatic reversal</li>
 *     <li><b>Flywheel</b>: Starts at 4200 RPM after START; GP2 Y/A adjusts manual RPM, auto-aim overrides it</li>
 *     <li><b>Transfer</b>: Stage 1 loads until full; stage 2 runs only during gate-open feeding; shots run both</li>
 *     <li><b>Full Queue</b>: Intake reverses after 2 seconds HIGH until GP1 right trigger is released</li>
 *     <li><b>Gamepad 2 Right Stick X</b>: Proportional clockwise turret speed, full stick = rated 6 V speed;
 *         release holds measured heading. Front=0, right/home=90, back=180, left=270 deg (260..280 excluded).</li>
 *     <li><b>Gamepad 2 D-Pad Up / Down</b>: Raise/lower launch elevation by 0.5 deg within -15..+15 deg when not auto-aiming</li>
 *     <li><b>Gamepad 2 Y / A</b>: Increase / decrease manual flywheel RPM by 50 within 0..6000 RPM when not auto-aiming</li>
 * </ul>
 */
public abstract class MecanumTeleOp extends LinearOpMode {

    protected abstract TurretSubsystem.Alliance getAlliance();

    private static final double PITCH_STEP_DEG = 0.5;
    private static final double SHOOTER_RPM_STEP = 50.0;
    private static final double TRIGGER_DEADBAND = 0.1;

    @Override
    public void runOpMode() {
        RobotLoop loop = new RobotLoop(hardwareMap);
        MecanumDrivetrain drivetrain = new MecanumDrivetrain(hardwareMap);
        PinpointOdometry odometry;
        IntakeSubsystem intake = null;
        GateSubsystem gate = null;
        TransferSubsystem transfer = null;
        DepollinatorSubsystem depollinator;
        TurretSubsystem turret = null;
        try {
            odometry = new PinpointOdometry(hardwareMap);
            intake = new IntakeSubsystem(hardwareMap);
            gate = new GateSubsystem(hardwareMap);
            transfer = new TransferSubsystem(hardwareMap);
            depollinator = new DepollinatorSubsystem(hardwareMap);
            try {
                turret = new TurretSubsystem(hardwareMap);
                turret.setTargetHiveCell(getAlliance(), TurretSubsystem.HiveSide.FAR);
            } catch (Exception e) {
                telemetry.addData("Turret Warning", "Turret hardware not fully plugged in (%s)", e.getMessage());
            }

            ShotCoordinator shots = turret == null ? null : new ShotCoordinator(turret,
                    ShootingConfiguration.target(getAlliance(), TurretSubsystem.HiveSide.FAR));
            telemetry.addData("Status", "Initialized (Bare Motor: %.0f PPR, %.0f RPM)",
                    MecanumDrivetrain.BARE_MOTOR_TICKS_PER_REV, MecanumDrivetrain.BARE_MOTOR_MAX_RPM);
            odometry.addInitTelemetry(telemetry);
            telemetry.addData("Alliance", getAlliance());
            telemetry.addLine("GP1: LS = Drive | RS-X = Turn");
            telemetry.addLine("GP1: RS Click = Deploy Depollinator | LS Click = Home Depollinator");
            telemetry.addLine("GP1/2 Intake: RB = Forward (Motor CCW) | LB = Reverse");
            telemetry.addLine("Intake bumpers override auto reversal | Opposing bumpers = Stop");
            telemetry.addLine("GP1: RT = Auto-Aim + Fire When Ready | LT = Manual Gate + Feed");
            telemetry.addLine("GP2 RS-X = Turret Yaw");
            telemetry.addLine("GP2: D-Pad U/D = Hood +/-0.5 deg | Y/A = Flywheel +/-50 RPM");
            telemetry.addLine("GP2: RT = Manual Gate + Feed | LT = Close Gate + Stop Transfer (Priority) | B = Stop Flywheel");
            telemetry.addLine("Flywheel starts at 4200 RPM after START; auto-aim overrides manual hood/RPM");
            telemetry.addLine("Transfer: Automatic after START | HIGH for 2 s = Stop | LOW = Run");
            telemetry.addLine("Full queue reverses intake until GP1 RT release | Both transfer stages run during feed");
            transfer.addTelemetry(telemetry);
            loop.addTelemetry(telemetry); telemetry.update();

            // Homing continues through INIT and an early START without blocking the drivetrain.
            boolean startingPoseApplied = false;
            boolean previousHome = false, previousSideLeft = false, previousSideRight = false;
            while (opModeInInit()) {
                loop.begin();
                odometry.update();
                if (turret != null) {
                    if (gamepad2.x && !previousHome) turret.homeTurretAndZeroEncoder();
                    if (gamepad2.dpad_left && !previousSideLeft) shots.select(ShootingConfiguration.target(getAlliance(), TurretSubsystem.HiveSide.AUDIENCE));
                    if (gamepad2.dpad_right && !previousSideRight) shots.select(ShootingConfiguration.target(getAlliance(), TurretSubsystem.HiveSide.FAR));
                }
                previousHome = gamepad2.x; previousSideLeft = gamepad2.dpad_left; previousSideRight = gamepad2.dpad_right;
                if (!startingPoseApplied && ShootingConfiguration.STARTING_FIELD_POSE_CONFIGURED
                        && odometry.getDeviceStatus() == com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus.READY) {
                    odometry.setPoseInches(ShootingConfiguration.START_X_METERS / .0254,
                            ShootingConfiguration.START_Y_METERS / .0254, Math.toDegrees(ShootingConfiguration.START_HEADING_RAD));
                    startingPoseApplied = true;
                }
                if (turret != null) { turret.update(); turret.addTelemetry(telemetry); }
                telemetry.addLine("GP2 X = Rehome | GP2 Dpad L/R = Audience/Far target");
                odometry.addTelemetry(telemetry); loop.addTelemetry(telemetry); telemetry.update(); idle();
            }
            waitForStart();
            if (isStopRequested()) return;

            FiringCoordinator fallbackFiring = ShootingConfiguration.firing();
            boolean previousDpadUp = false;
            boolean previousDpadDown = false;
            boolean previousRpmUp = false;
            boolean previousRpmDown = false;
            double manualShooterRpm = TurretSubsystem.SHOOTER_DEFAULT_TARGET_RPM;
            IntakeQueueController intakeControl = new IntakeQueueController();
            boolean previousDepollinatorDeploy = false;
            boolean previousDepollinatorHome = false;

            double previousLoopTime = System.nanoTime() * 1e-9;
            while (opModeIsActive()) {
                loop.begin();
                double now = System.nanoTime() * 1e-9;
                double dt = Math.max(0, Math.min(.1, now - previousLoopTime)); previousLoopTime = now;
                // 1. Update Pinpoint Odometry Computer
                odometry.update();

                // 2. Robot-centric Mecanum Drive on Controller 1
                double axial = -gamepad1.left_stick_y;
                double lateral = gamepad1.left_stick_x;
                double yaw = gamepad1.right_stick_x;

                drivetrain.driveRobotCentric(axial, lateral, yaw);
                boolean deployDepollinator = gamepad1.right_stick_button;
                boolean homeDepollinator = gamepad1.left_stick_button;
                if (homeDepollinator && !previousDepollinatorHome) {
                    depollinator.home();
                } else if (deployDepollinator && !previousDepollinatorDeploy && !homeDepollinator) {
                    depollinator.deploy();
                }
                previousDepollinatorDeploy = deployDepollinator;
                previousDepollinatorHome = homeDepollinator;
                boolean autoAimHeld = gamepad1.right_trigger > TRIGGER_DEADBAND;
                boolean manualFeedHeld = gamepad1.left_trigger > GateSubsystem.TRIGGER_DEADBAND
                        || gamepad2.right_trigger > TRIGGER_DEADBAND;
                boolean inhibitFeedHeld = gamepad2.left_trigger > TRIGGER_DEADBAND;
                boolean flywheelStopHeld = gamepad2.b;
                boolean autoShotReady = false;

                // 3. Turret & Limelight 3A Control (supports both Gamepad 1 and Gamepad 2)
                if (turret != null) {
                    boolean currentDpadUp = gamepad2.dpad_up;
                    boolean currentDpadDown = gamepad2.dpad_down;
                    boolean currentRpmUp = gamepad2.y;
                    boolean currentRpmDown = gamepad2.a;
                    if (!autoAimHeld) {
                        if (currentDpadUp && !previousDpadUp) {
                            turret.adjustPitchAngleDegrees(-PITCH_STEP_DEG);
                        }
                        if (currentDpadDown && !previousDpadDown) {
                            turret.adjustPitchAngleDegrees(+PITCH_STEP_DEG);
                        }
                        double rpmChange = 0.0;
                        if (currentRpmUp && !previousRpmUp) {
                            rpmChange += SHOOTER_RPM_STEP;
                        }
                        if (currentRpmDown && !previousRpmDown) {
                            rpmChange -= SHOOTER_RPM_STEP;
                        }
                        manualShooterRpm = Math.max(0.0, Math.min(
                                TurretSubsystem.SHOOTER_MOTOR_MAX_RPM, manualShooterRpm + rpmChange));
                    }
                    previousDpadUp = currentDpadUp;
                    previousDpadDown = currentDpadDown;
                    previousRpmUp = currentRpmUp;
                    previousRpmDown = currentRpmDown;

                    if (!autoAimHeld && gamepad2.x && !previousHome) turret.homeTurretAndZeroEncoder();
                    if (gamepad2.dpad_left && !previousSideLeft) shots.select(ShootingConfiguration.target(getAlliance(), TurretSubsystem.HiveSide.AUDIENCE));
                    if (gamepad2.dpad_right && !previousSideRight) shots.select(ShootingConfiguration.target(getAlliance(), TurretSubsystem.HiveSide.FAR));
                    previousHome = gamepad2.x; previousSideLeft = gamepad2.dpad_left; previousSideRight = gamepad2.dpad_right;
                    turret.update();
                    turret.updateManualYaw(gamepad2.right_stick_x, dt, autoAimHeld);

                    if (!autoAimHeld) {
                        if (flywheelStopHeld) {
                            turret.stopShooter();
                        } else {
                            turret.setShooterRpm(manualShooterRpm);
                        }
                    }
                    now = System.nanoTime() * 1e-9;
                    shots.update(now, odometry.getSample(), autoAimHeld, manualFeedHeld, inhibitFeedHeld, flywheelStopHeld);
                    if (autoAimHeld && !flywheelStopHeld && !shots.solution().aimable) turret.setShooterRpm(manualShooterRpm);
                    autoShotReady = shots.firing().gateOpen();
                    shots.addTelemetry(telemetry, now);
                    telemetry.addData("Manual Flywheel Target (RPM)", "%.0f", manualShooterRpm);
                }

                boolean openGate = shots != null && shots.firing().gateOpen();
                // Manual feed also works when turret hardware is unavailable.
                if (shots == null) {
                    fallbackFiring.update(now, false, manualFeedHeld, inhibitFeedHeld, flywheelStopHeld, false, false, false, "Turret unavailable");
                    openGate = fallbackFiring.gateOpen();
                }
                FiringCoordinator firing = shots == null ? fallbackFiring : shots.firing();
                gate.setOpen(openGate);
                transfer.update(firing.feeding() && gate.isOpenAndSettled(), inhibitFeedHeld || autoAimHeld || openGate || firing.state() == FiringCoordinator.State.RECOVERY);
                double intakePower = intakeControl.update(transfer.isFull(), autoAimHeld,
                        gamepad1.right_bumper || gamepad2.right_bumper, gamepad1.left_bumper || gamepad2.left_bumper);
                intake.setIntakePower(intakePower);
                telemetry.addData("Automatic Shot", !autoAimHeld ? "OFF"
                        : autoShotReady ? "READY" : "WAITING FOR AIM / HOOD / RPM");
                telemetry.addData("GP2 Overrides", "Feed Inhibit: %s | Flywheel Stop: %s",
                        inhibitFeedHeld, flywheelStopHeld);

                // 4. Flush Telemetry once per loop at the very end
                odometry.addTelemetry(telemetry);
                drivetrain.addTelemetry(telemetry);
                intake.addTelemetry(telemetry);
                gate.addTelemetry(telemetry);
                transfer.addTelemetry(telemetry);
                depollinator.addTelemetry(telemetry);
                if (turret != null) {
                    turret.addTelemetry(telemetry);
                }
                loop.addTelemetry(telemetry); telemetry.update();
            }
        } finally {
            RobotShutdown.stopAll(drivetrain::stop, intake == null ? null : intake::stop,
                    transfer == null ? null : transfer::stop, gate == null ? null : gate::close,
                    turret == null ? null : turret::stop);
        }
    }
}
