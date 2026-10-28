package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Combined intake/index/shooter bench test with no drive, odometry, camera or depollinator. */
@TeleOp(name = "Test: Intake + Index + Shooter", group = "Test")
public class IntakeShooterTestOpMode extends LinearOpMode {
    private static final double PITCH_STEP_DEG = 0.5;
    private static final double SHOOTER_RPM_STEP = 100.0;
    private static final double MAX_LOOP_DT_SECONDS = 0.1;

    @Override
    public void runOpMode() {
        RobotLoop loop = new RobotLoop(hardwareMap);
        GateSubsystem gate = null;
        IntakeSubsystem intake = null;
        TransferSubsystem transfer = null;
        TurretSubsystem turret = null;
        try {
            gate = new GateSubsystem(hardwareMap);
            // Intake configures the motor whose encoder channel can also serve the turret.
            intake = new IntakeSubsystem(hardwareMap);
            transfer = new TransferSubsystem(hardwareMap);
            turret = new TurretSubsystem(hardwareMap, false);
            IntakeShooterTestController controls = new IntakeShooterTestController();
            double selectedRpm = TurretSubsystem.SHOOTER_DEFAULT_TARGET_RPM;
            boolean previousHome = false;

            while (opModeInInit()) {
                loop.begin();
                if (gamepad1.x && !previousHome) turret.setAllServosToZeroPosition();
                previousHome = gamepad1.x;
                turret.update();
                addControlsTelemetry();
                telemetry.addLine("Turret homes during INIT: home points robot-right (90 deg).");
                telemetry.addLine("After START, release bumpers/triggers to enable motor controls.");
                turret.addTelemetry(telemetry);
                gate.addTelemetry(telemetry);
                loop.addTelemetry(telemetry);
                telemetry.update();
                idle();
            }
            waitForStart();
            if (isStopRequested()) return;

            boolean previousRpmUp = false, previousRpmDown = false;
            boolean previousHoodUp = false, previousHoodDown = false;
            boolean previousPidUp = false, previousPidDown = false;
            boolean previousFUp = false, previousFDown = false, previousPidReset = false;
            boolean previousIUp = false, previousIDown = false;
            boolean previousControllerToggle = gamepad2.b;
            boolean wasFeeding = false;
            double minimumFeedRpm = Double.NaN;
            double recoveryStarted = Double.NaN, lastRecoverySeconds = Double.NaN;
            double previousTime = System.nanoTime() * 1e-9;
            while (opModeIsActive()) {
                loop.begin();
                double now = System.nanoTime() * 1e-9;
                double dt = Math.max(0, Math.min(MAX_LOOP_DT_SECONDS, now - previousTime));
                previousTime = now;
                boolean controllerChanged = gamepad2.b && !previousControllerToggle;
                previousControllerToggle = gamepad2.b;
                if (controllerChanged) {
                    turret.setShooterController(turret.getShooterController() == TurretSubsystem.ShooterController.PIDF
                            ? TurretSubsystem.ShooterController.BANG_BANG : TurretSubsystem.ShooterController.PIDF);
                    minimumFeedRpm = Double.NaN;
                    recoveryStarted = lastRecoverySeconds = Double.NaN;
                    wasFeeding = false;
                }
                double deltaP = (gamepad2.dpad_up && !previousPidUp ? 1.0 : 0)
                        - (gamepad2.dpad_down && !previousPidDown ? 1.0 : 0);
                double deltaF = (gamepad2.dpad_right && !previousFUp ? .05 : 0)
                        - (gamepad2.dpad_left && !previousFDown ? .05 : 0);
                double deltaI = (gamepad2.y && !previousIUp ? ShooterVelocityTuning.I_STEP : 0)
                        - (gamepad2.a && !previousIDown ? ShooterVelocityTuning.I_STEP : 0);
                if (gamepad2.x && !previousPidReset) turret.resetShooterVelocityTuning();
                else if (deltaP != 0 || deltaI != 0 || deltaF != 0)
                    turret.adjustShooterVelocityTuning(deltaP, deltaI, deltaF);
                previousPidUp = gamepad2.dpad_up; previousPidDown = gamepad2.dpad_down;
                previousFUp = gamepad2.dpad_right; previousFDown = gamepad2.dpad_left;
                previousPidReset = gamepad2.x;
                previousIUp = gamepad2.y; previousIDown = gamepad2.a;
                boolean load = gamepad1.right_bumper;
                boolean reverse = gamepad1.left_bumper;
                boolean spin = gamepad1.right_trigger > GateSubsystem.TRIGGER_DEADBAND;
                boolean feed = gamepad1.left_trigger > GateSubsystem.TRIGGER_DEADBAND;
                boolean rehome = gamepad1.x && !previousHome && !gamepad1.b
                        && !load && !reverse && !spin && !feed;
                previousHome = gamepad1.x;
                // Close the gate and re-arm after a switch; a held trigger cannot resume feeding.
                boolean stop = gamepad1.b || rehome || controllerChanged;

                if (rehome) turret.setAllServosToZeroPosition();
                boolean rpmChanged = false, hoodChanged = false;
                if (!stop) {
                    double rpmDelta = (gamepad1.y && !previousRpmUp ? SHOOTER_RPM_STEP : 0)
                            - (gamepad1.a && !previousRpmDown ? SHOOTER_RPM_STEP : 0);
                    double nextRpm = Math.max(0, Math.min(TurretSubsystem.SHOOTER_MAX_RPM,
                            selectedRpm + rpmDelta));
                    rpmChanged = nextRpm != selectedRpm;
                    selectedRpm = nextRpm;
                    double hoodDelta = (gamepad1.dpad_up && !previousHoodUp ? PITCH_STEP_DEG : 0)
                            - (gamepad1.dpad_down && !previousHoodDown ? PITCH_STEP_DEG : 0);
                    hoodChanged = hoodDelta != 0;
                    if (hoodChanged) turret.adjustPitchAngleDegrees(hoodDelta);
                }
                previousRpmUp = gamepad1.y;
                previousRpmDown = gamepad1.a;
                previousHoodUp = gamepad1.dpad_up;
                previousHoodDown = gamepad1.dpad_down;

                turret.update();
                if (!stop) turret.updateManualYaw(gamepad1.right_stick_x, dt, false);
                boolean yawInput = Math.abs(gamepad1.right_stick_x) > TurretControl.MANUAL_STICK_DEADBAND;
                // Compare cached measured speed against this loop's selected target.
                boolean atSpeed = selectedRpm > 0 && Math.abs(turret.getShooterCurrentRpm() - selectedRpm)
                        <= TurretSubsystem.SHOOTER_READY_TOLERANCE_RPM;
                if (stop || !spin || rpmChanged) recoveryStarted = Double.NaN;
                else if (!atSpeed && Double.isNaN(recoveryStarted)) recoveryStarted = now;
                else if (atSpeed && !Double.isNaN(recoveryStarted)) {
                    lastRecoverySeconds = now - recoveryStarted;
                    recoveryStarted = Double.NaN;
                }
                double yawErrorDeg = turret.getTargetYawDegrees() - turret.getEncoderAngleDegrees();
                boolean yawSettled = IntakeShooterTestController.isYawSettled(yawErrorDeg,
                        turret.getEncoderVelocityDegPerSec());
                boolean hoodSettled = turret.isHoodTracking();
                boolean ready = atSpeed && yawSettled
                        && hoodSettled && !yawInput && !rpmChanged && !hoodChanged;
                controls.update(now, load, reverse, spin && selectedRpm > 0, feed, stop, turret.isHomed(), ready);
                if (controls.spinFlywheel() && selectedRpm > 0) turret.setShooterRpm(selectedRpm);
                else turret.stopShooter();

                FiringCoordinator firing = controls.firing();
                gate.setOpen(firing.gateOpen());
                boolean feeding = controls.feedTransfer(gate.isOpenForFeeding());
                if (feeding) minimumFeedRpm = !wasFeeding ? turret.getShooterCurrentRpm()
                        : Math.min(minimumFeedRpm, turret.getShooterCurrentRpm());
                wasFeeding = feeding;
                transfer.update(feeding, !controls.loadIndex() || firing.gateOpen());
                intake.setIntakePower(controls.intakePower(transfer.isFull()));

                // Put the exact blocker first so it remains visible above the hardware diagnostics.
                String feedStatus;
                if (stop) feedStatus = "STOP requested";
                else if (!controls.armed()) feedStatus = "Release B, bumpers and triggers to enable";
                else if (reverse) feedStatus = "Release LB: reverse intake inhibits feed";
                else if (!feed) feedStatus = "Hold GP1 LT with RT to request feed";
                else if (!spin) feedStatus = "Hold GP1 RT to spin the flywheel";
                else if (!turret.isHomed()) feedStatus = "Home required: " + turret.getHomingStatus();
                else if (selectedRpm <= 0) feedStatus = "Selected RPM is zero: tap Y to increase";
                else if (firing.feeding()) feedStatus = "TIMED FEED: both stages forward; release either trigger to stop";
                else if (!atSpeed) feedStatus = "Flywheel outside RPM tolerance: check actual/target and encoder cable";
                else if (yawInput) feedStatus = "Release right stick to settle turret";
                else if (!yawSettled) feedStatus = "Turret outside 4 deg / 3 deg/s bench limits";
                else if (!hoodSettled || hoodChanged) feedStatus = "Waiting for hood travel";
                else if (rpmChanged) feedStatus = "RPM target changed: requalifying readiness";
                else if (firing.state() == FiringCoordinator.State.PREPARING)
                    feedStatus = "Checks passing: qualifying stable readiness";
                else feedStatus = firing.reason();
                telemetry.addData("FEED STATUS", feedStatus);
                telemetry.addData("GP1 triggers / LB", "RT %.2f | LT %.2f | LB %s",
                        gamepad1.right_trigger, gamepad1.left_trigger, reverse);
                telemetry.addData("Feed flywheel actual / selected (RPM)", "%.0f / %.0f",
                        turret.getShooterCurrentRpm(), selectedRpm);
                telemetry.addData("Lowest flywheel RPM in last/current feed", minimumFeedRpm);
                telemetry.addData("Last return to RPM tolerance (s)", lastRecoverySeconds);
                if (!Double.isNaN(recoveryStarted)) telemetry.addData("RPM recovery elapsed (s)", now - recoveryStarted);
                telemetry.addData("Feed yaw error / rate (deg, deg/s)", "%.2f / %.2f",
                        yawErrorDeg, turret.getEncoderVelocityDegPerSec());
                telemetry.addData("Motor controls", controls.armed() ? "ENABLED"
                        : "STOPPED: release B, bumpers and triggers to enable");
                telemetry.addData("Selected flywheel (RPM)", "%.0f (hold RT to spin)", selectedRpm);
                telemetry.addData("Feed checks", "home %s | RPM %s | yaw %s | hood %s",
                        turret.isHomed(), atSpeed, yawSettled, hoodSettled);
                telemetry.addData("Feed state", "%s | %s", firing.state(),
                        firing.feeding() ? "Timed two-stage forward feed" : firing.reason());
                telemetry.addData("Timed shooting (s)", "gate %.2f | feed %.2f | recovery %.2f",
                        firing.gateTravelSeconds, firing.feedWindowSeconds, firing.recoverySeconds);
                telemetry.addData("Feed starts (not ball count)", firing.feedAttempts());
                telemetry.addData("Feed motors", feeding ? "RUNNING" : "STOPPED");
                telemetry.addData("Flywheel regulation", turret.getShooterController());
                addControlsTelemetry();
                intake.addTelemetry(telemetry);
                transfer.addTelemetry(telemetry);
                gate.addTelemetry(telemetry);
                turret.addTelemetry(telemetry);
                loop.addTelemetry(telemetry);
                telemetry.update();
            }
        } finally {
            RobotShutdown.stopAll(intake == null ? null : intake::stop,
                    transfer == null ? null : transfer::stop,
                    gate == null ? null : gate::close,
                    turret == null ? null : turret::stop);
        }
    }

    private void addControlsTelemetry() {
        telemetry.addLine("GP1 RB = Intake + Stage 1 (Stage 2 Off) | LB = Reverse Intake | Both = Stop Loading");
        telemetry.addLine("Hold RT = Flywheel | Hold RT + LT = Timed Feed/Recovery After Readiness");
        telemetry.addLine("GP2 PID tuning: Dpad U/D = P +/-1 | R/L = F scale +/-0.05 | Y/A = I +/-0.05 | X = Reset");
        telemetry.addLine("GP2 B = Toggle PIDF / Bang-Bang; release GP1 motor controls to re-arm after switching");
        telemetry.addLine("Y/A = RPM +/-100 | Dpad Up = Lower Hood, Down = Raise Hood (0.5 deg) | RS-X = Turret");
        telemetry.addLine("B = Stop Motors + Close Gate (release controls to resume) | X = Rehome with motor controls released");
    }
}
