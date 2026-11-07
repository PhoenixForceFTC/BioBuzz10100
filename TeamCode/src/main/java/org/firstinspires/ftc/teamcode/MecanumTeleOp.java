package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Layer 3 Combined TeleOp: Coordinates {@link MecanumDrivetrain}, {@link PinpointOdometry},
 * {@link IntakeSubsystem}, {@link TransferSubsystem}, {@link GateSubsystem}, {@link DepollinatorSubsystem},
 * and {@link TurretSubsystem} (rotation servos on ports 1 & 2, 108:40 REV Through-Bore Encoder
 * on Expansion Hub quad port 2 via transfer_motor_1, 2:1 pitch servo on port 0 from 0-30 deg downward travel from the offset hood home (+15 to -15 deg launch elevation), 6000 RPM flywheel motor on port 0,
 * and Limelight 3A angled 30 deg toward the goal).
 *
 * <p>Controls (Gamepad 1 — Driver + Turret Assist, or Gamepad 2 — Dedicated Turret Operator):
 * <ul>
 *     <li><b>Gamepad 1 Left Stick</b>: Forward / Backward & Strafe translation</li>
 *     <li><b>Gamepad 1 Right Stick X</b>: Chassis rotation</li>
 *     <li><b>Gamepad 1 Right / Left Stick Click</b>: Deploy depollinator 135 deg CW / return to 30 deg home</li>
 *     <li><b>Gamepad 1 Right Trigger (Hold)</b>: Prepare auto-aim, hood angle and flywheel RPM</li>
 *     <li><b>Gamepad 1 Right + Left Triggers (Hold)</b>: Start automatic shooting when ready</li>
 *     <li><b>Gamepad 1 A (Hold)</b>: Manually feed immediately in continuous; delayed in staggered</li>
 *     <li><b>Gamepad 2 Right Trigger (Hold)</b>: Manually feed immediately in continuous; delayed in staggered</li>
 *     <li><b>Gamepad 2 Left Trigger (Hold)</b>: Force gate closed and stop both transfer stages; overrides firing</li>
 *     <li><b>Gamepad 2 B (Press)</b>: Toggle continuous/staggered feeding; starts continuous</li>
 *     <li><b>Gamepad 2 D-Pad Right (Press)</b>: Clear auto yaw/hood/RPM offsets</li>
 *     <li><b>Gamepad 2 D-Pad Left (Press)</b>: Set field pose at the alliance corner opposite its pollen depot</li>
 *     <li><b>Gamepad 2 X (Press)</b>: Rehome turret only</li>
 *     <li><b>Gamepad 1/2 Right Bumper</b>: Force intake and stage-1 loading forward, overriding full</li>
 *     <li><b>Gamepad 1/2 Left Bumper</b>: Reverse intake only and pause stage-1 loading; both bumpers stop collection</li>
 *     <li><b>Flywheel</b>: Starts at 3500 output RPM after START; GP2 Y/A adjusts manual RPM, auto-aim overrides it</li>
 *     <li><b>Transfer</b>: Stage 1 loads until full; staggered shots start both stages after 0.10 s</li>
 *     <li><b>Full Queue</b>: After 0.5 seconds HIGH, intake reverses; stage 1 runs on 0.5 seconds. LOW resumes loading.</li>
 *     <li><b>Intake</b>: Automatically runs forward with space; bumpers always override direction</li>
 *     <li><b>Gamepad 2 Right Stick X</b>: Proportional clockwise turret speed, full stick = rated 6 V speed;
 *         release holds measured heading. Front=0, right/home=90, back=180, left=270 deg (265..275 excluded).</li>
 *     <li><b>Gamepad 2 D-Pad Up / Down</b>: Lower/raise launch elevation by 0.5 deg within -15..+15 deg; auto-aim adjusts a bounded hood offset</li>
     *     <li><b>Gamepad 2 Y / A</b>: Increase / decrease manual flywheel RPM by 50, or a bounded RPM offset while auto-aiming</li>
 *     <li><b>Gamepad 2 Back (Press)</b>: Show / hide full diagnostics telemetry below the short driver view</li>
 * </ul>
 */
public abstract class MecanumTeleOp extends LinearOpMode {

    protected abstract TurretSubsystem.Alliance getAlliance();

    /** Rear and side against the corner walls, chassis pointing inward toward the alliance goal. */
    protected abstract Pose2D getFieldRehomePose();

    /** Default robot configuration; desktop fixtures can inject explicitly synthetic shot measurements. */
    protected ShotCoordinator createShotCoordinator(TurretSubsystem turret) {
        return new ShotCoordinator(turret,
                TeleOpFieldReference.target(getAlliance(), TurretSubsystem.HiveSide.AUDIENCE),
                ShotCoordinator.AimMode.TELEOP_ODOMETRY);
    }

    private static final double PITCH_STEP_DEG = 0.5;
    private static final double SHOOTER_RPM_STEP = 50.0;
    private static final double TRIGGER_DEADBAND = 0.1;
    public static final double STARTING_FLYWHEEL_RPM = 3500.0;

    private void rehomeField(PinpointOdometry odometry, ShotCoordinator shots, Pose2D cornerPose) {
        odometry.setPose(cornerPose);
        if (shots != null) shots.requestFieldRehomeRefinement(System.nanoTime() * 1e-9);
    }

    @Override
    public void runOpMode() {
        final Pose2D fieldRehomePose = getFieldRehomePose();
        RobotLoop loop = new RobotLoop(hardwareMap);
        MecanumDrivetrain drivetrain = new MecanumDrivetrain(hardwareMap);
        PinpointOdometry odometry;
        IntakeSubsystem intake = null;
        GateSubsystem gate = null;
        TransferSubsystem transfer = null;
        DepollinatorSubsystem depollinator;
        TurretSubsystem turret = null;
        RobotStatusLights lights = null;
        try {
            odometry = new PinpointOdometry(hardwareMap);
            intake = new IntakeSubsystem(hardwareMap);
            gate = new GateSubsystem(hardwareMap, CompetitionFeedController.GATE_TO_FEED_DELAY_SECONDS);
            transfer = new TransferSubsystem(hardwareMap);
            depollinator = new DepollinatorSubsystem(hardwareMap);
            lights = new RobotStatusLights(hardwareMap);
            String turretError = null;
            try {
                turret = new TurretSubsystem(hardwareMap);
                turret.setEncoderCorrectionEnabled(true);
                turret.setTargetHiveCell(getAlliance(), TurretSubsystem.HiveSide.AUDIENCE);
            } catch (Exception e) {
                turretError = e.getMessage();
            }

            ShotCoordinator shots = turret == null ? null : createShotCoordinator(turret);
            boolean showDiagnostics = false;
            boolean previousDiagnosticsToggle = false;

            // Homing continues through INIT and an early START without blocking the drivetrain.
            boolean previousHome = false;
            boolean previousFieldRehome = false;
            while (opModeInInit()) {
                loop.begin();
                if (gamepad2.dpad_left && !previousFieldRehome) rehomeField(odometry,shots,fieldRehomePose);
                previousFieldRehome = gamepad2.dpad_left;
                if (gamepad2.back && !previousDiagnosticsToggle) showDiagnostics = !showDiagnostics;
                previousDiagnosticsToggle = gamepad2.back;
                odometry.update();
                if (turret != null) {
                    if (gamepad2.x && !previousHome) {
                        shots.cancelFieldRehomeRefinement("turret home requested");
                        turret.homeTurretAndZeroEncoder();
                    }
                    if (Math.abs(gamepad2.right_stick_x) > TurretControl.MANUAL_STICK_DEADBAND)
                        shots.cancelFieldRehomeRefinement("manual turret control");
                }
                previousHome = gamepad2.x;
                double now = System.nanoTime() * 1e-9;
                if (turret != null) {
                    turret.update();
                    shots.updatePose(now,odometry.getSample());
                }
                MotionSample initPose = shots == null ? null : shots.pose();
                lights.showReference(turret != null && !turret.hasHomingFault(), turret != null && turret.isHomed(),
                        initPose != null && initPose.fieldReferenced && initPose.fresh(now, ShotPlanner.MAX_SAMPLE_AGE_SECONDS));

                telemetry.addData("Alliance", getAlliance());
                if (turret != null) turret.addDriverTelemetry(telemetry);
                else telemetry.addData("Turret", "UNAVAILABLE: %s", turretError);
                if (shots != null) shots.addDriverTelemetry(telemetry, now, false);
                odometry.addDriverTelemetry(telemetry);
                lights.addWarningTelemetry(telemetry);
                telemetry.addLine("GP2: X = home turret | D-Pad Left = corner rehome | Back = diagnostics");
                if (showDiagnostics) {
                    telemetry.addLine("--- DIAGNOSTICS ---");
                    drivetrain.addConfigurationTelemetry(telemetry);
                    odometry.addInitTelemetry(telemetry);
                    odometry.addTelemetry(telemetry);
                    if (shots != null) shots.addTelemetry(telemetry, now);
                    if (turret != null) turret.addTelemetry(telemetry);
                    transfer.addTelemetry(telemetry);
                    lights.addTelemetry(telemetry);
                    loop.addTelemetry(telemetry);
                }
                telemetry.update(); idle();
            }
            waitForStart();
            if (isStopRequested()) return;

            FiringCoordinator fallbackFiring = CompetitionFeedController.firing();
            FiringCoordinator firing = shots == null ? fallbackFiring : shots.firing();
            CompetitionFeedController feedControl = new CompetitionFeedController();
            boolean previousDpadUp = false;
            boolean previousDpadDown = false;
            boolean previousRpmUp = false;
            boolean previousRpmDown = false;
            boolean previousTrimReset = gamepad2.dpad_right;
            AutoAimAdjustments aimAdjustments = new AutoAimAdjustments();
            double manualShooterRpm = STARTING_FLYWHEEL_RPM;
            IntakeQueueController intakeControl = new IntakeQueueController();
            boolean previousDepollinatorDeploy = false;
            boolean previousDepollinatorHome = false;

            double previousLoopTime = System.nanoTime() * 1e-9;
            while (opModeIsActive()) {
                loop.begin();
                double now = System.nanoTime() * 1e-9;
                double dt = Math.max(0, Math.min(.1, now - previousLoopTime)); previousLoopTime = now;
                // 1. Update Pinpoint Odometry Computer
                // Apply placement before the single bulk read; aiming requalifies after velocity history resumes.
                if (gamepad2.dpad_left && !previousFieldRehome) rehomeField(odometry,shots,fieldRehomePose);
                previousFieldRehome = gamepad2.dpad_left;
                odometry.update();
                if (gamepad2.back && !previousDiagnosticsToggle) showDiagnostics = !showDiagnostics;
                previousDiagnosticsToggle = gamepad2.back;

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
                boolean automaticFeedHeld = autoAimHeld && gamepad1.left_trigger > TRIGGER_DEADBAND;
                boolean manualFeedHeld = gamepad1.a
                        || gamepad2.right_trigger > TRIGGER_DEADBAND;
                boolean inhibitFeedHeld = gamepad2.left_trigger > TRIGGER_DEADBAND;
                boolean intakeForwardHeld = gamepad1.right_bumper || gamepad2.right_bumper;
                boolean intakeReverseHeld = gamepad1.left_bumper || gamepad2.left_bumper;
                boolean autoShotReady = false;
                feedControl.updateSelection(now, gamepad2.b, firing);

                // 3. Turret & Limelight 3A Control (supports both Gamepad 1 and Gamepad 2)
                if (turret != null) {
                    boolean currentDpadUp = gamepad2.dpad_up;
                    boolean currentDpadDown = gamepad2.dpad_down;
                    boolean currentRpmUp = gamepad2.y;
                    boolean currentRpmDown = gamepad2.a;
                    if (gamepad2.dpad_right && !previousTrimReset) aimAdjustments.reset();
                    previousTrimReset = gamepad2.dpad_right;
                    if (autoAimHeld) {
                        aimAdjustments.adjust(gamepad2.right_stick_x, dt,
                                (currentDpadUp && !previousDpadUp ? 1 : 0) - (currentDpadDown && !previousDpadDown ? 1 : 0),
                                (currentRpmUp && !previousRpmUp ? 1 : 0) - (currentRpmDown && !previousRpmDown ? 1 : 0));
                    } else {
                        if (currentDpadUp && !previousDpadUp) {
                            turret.adjustPitchAngleDegrees(+PITCH_STEP_DEG);
                        }
                        if (currentDpadDown && !previousDpadDown) {
                            turret.adjustPitchAngleDegrees(-PITCH_STEP_DEG);
                        }
                        double rpmChange = 0.0;
                        if (currentRpmUp && !previousRpmUp) {
                            rpmChange += SHOOTER_RPM_STEP;
                        }
                        if (currentRpmDown && !previousRpmDown) {
                            rpmChange -= SHOOTER_RPM_STEP;
                        }
                        manualShooterRpm = Math.max(0.0, Math.min(
                                TurretSubsystem.SHOOTER_MAX_RPM, manualShooterRpm + rpmChange));
                    }
                    previousDpadUp = currentDpadUp;
                    previousDpadDown = currentDpadDown;
                    previousRpmUp = currentRpmUp;
                    previousRpmDown = currentRpmDown;

                    if (!autoAimHeld && gamepad2.x && !previousHome) {
                        aimAdjustments.reset();
                        shots.cancelFieldRehomeRefinement("turret home requested");
                        turret.homeTurretAndZeroEncoder();
                    }
                    if (Math.abs(gamepad2.right_stick_x) > TurretControl.MANUAL_STICK_DEADBAND)
                        shots.cancelFieldRehomeRefinement("manual turret control");
                    previousHome = gamepad2.x;
                    turret.update();
                    turret.updateCompetitionManualYaw(gamepad2.right_stick_x, dt, autoAimHeld || shots.fieldRehomePending());

                    if (!autoAimHeld) {
                        turret.setShooterRpm(manualShooterRpm);
                    }
                    now = System.nanoTime() * 1e-9;
                    shots.setAutoAimAdjustments(now, aimAdjustments);
                    shots.update(now, odometry.getSample(), autoAimHeld, automaticFeedHeld,
                            manualFeedHeld, inhibitFeedHeld, false);
                    if (autoAimHeld && !shots.solution().aimable) turret.setShooterRpm(manualShooterRpm);
                    autoShotReady = shots.automaticTrackingReady();
                }

                boolean openGate = shots != null && shots.firing().gateOpen();
                // Manual feed also works when turret hardware is unavailable.
                if (shots == null) {
                    fallbackFiring.update(now, false, manualFeedHeld, inhibitFeedHeld, false, false, false, false, "Turret unavailable");
                    openGate = fallbackFiring.gateOpen();
                }
                boolean feedStage1 = feedControl.feedStage1(now, firing);
                gate.setOpen(openGate);
                boolean shootingBusy = automaticFeedHeld || manualFeedHeld || openGate
                        || firing.state() == FiringCoordinator.State.RECOVERY;
                transfer.update(now, feedControl.feedTransfers(firing, gate.isOpenForFeeding()),
                        inhibitFeedHeld || shootingBusy || intakeReverseHeld,
                        feedStage1, intakeForwardHeld && !intakeReverseHeld);
                double intakePower = intakeControl.update(transfer.isFull(),
                        intakeForwardHeld, intakeReverseHeld, inhibitFeedHeld || shootingBusy);
                intake.setIntakePowerWithJamRecovery(now, intakePower);
                lights.showIntake(now, transfer.isFull(), intakeForwardHeld, intakeReverseHeld,
                        intake.getCommandedPower(), intake.isClearingJam());
                lights.showShooter(now, firing.state(), manualFeedHeld, autoShotReady,
                        turret != null && turret.getTargetShooterRpm() > 0);
                MotionSample aimingPose = shots == null ? null : shots.pose();
                lights.showReference(turret != null && !turret.hasHomingFault(), turret != null && turret.isHomed(),
                        aimingPose != null && aimingPose.fieldReferenced && aimingPose.fresh(now, ShotPlanner.MAX_SAMPLE_AGE_SECONDS));
                // 4. Driver view (most important first), optional diagnostics below, one flush per loop.
                FiringCoordinator.State firingState = firing.state();
                String shotStatus = inhibitFeedHeld ? "BLOCKED (GP2 LT)" : manualFeedHeld ? "MANUAL FEED"
                        : !autoAimHeld ? "OFF"
                        : !automaticFeedHeld ? "AIMING (hold LT to fire)"
                        : autoShotReady ? "READY" : "WAITING FOR AIM / HOOD / RPM";
                if (firingState == FiringCoordinator.State.INHIBITED || firingState == FiringCoordinator.State.FAULT)
                    telemetry.addData("Shot", "%s | %s: %s", shotStatus, firingState, firing.reason());
                else telemetry.addData("Shot", "%s | %s", shotStatus, firingState);
                if (turret != null) turret.addDriverTelemetry(telemetry);
                else telemetry.addData("Turret", "UNAVAILABLE: %s", turretError);
                if (shots != null) shots.addDriverTelemetry(telemetry, now, autoAimHeld);
                else telemetry.addData("Field rehome refinement", "CORNER ONLY: turret/camera unavailable");
                odometry.addDriverTelemetry(telemetry);
                transfer.addDriverTelemetry(telemetry);
                telemetry.addData("Collection", "%s | intake %+.1f%s", intakeReverseHeld
                                ? intakeForwardHeld ? "STOPPED" : "REVERSE"
                                : intakeForwardHeld ? "FORCED FORWARD" : "AUTO",
                        intake.getCommandedPower(), intake.isClearingJam() ? " | CLEARING JAM" : "");
                telemetry.addData("Feed mode (GP2 B)", feedControl.continuous() ? "CONTINUOUS" : "STAGGERED");
                telemetry.addData("Depollinator", depollinator.isDeployedCommanded() ? "DEPLOYED" : "HOME");
                lights.addWarningTelemetry(telemetry);
                loop.addRateTelemetry(telemetry);
                if (showDiagnostics) {
                    telemetry.addLine("--- DIAGNOSTICS (GP2 Back to hide) ---");
                    transfer.addStatusTelemetry(telemetry);
                    if (shots != null) shots.addTelemetry(telemetry, now);
                    odometry.addTelemetry(telemetry);
                    drivetrain.addTelemetry(telemetry);
                    intake.addTelemetry(telemetry);
                    gate.addTelemetry(telemetry);
                    transfer.addTelemetry(telemetry);
                    depollinator.addTelemetry(telemetry);
                    lights.addTelemetry(telemetry);
                    if (turret != null) turret.addTelemetry(telemetry);
                    loop.addTelemetry(telemetry);
                }
                telemetry.update();
            }
        } finally {
            RobotShutdown.stopAll(drivetrain::stop, intake == null ? null : intake::stop,
                    transfer == null ? null : transfer::stop, gate == null ? null : gate::close,
                    turret == null ? null : turret::stop, lights == null ? null : lights::stop);
        }
    }

}
