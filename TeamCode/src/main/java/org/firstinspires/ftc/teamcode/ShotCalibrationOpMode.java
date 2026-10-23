package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Locale;

/** Guided stationary shooting calibration; only turret, gate and transfer hardware are initialized. */
@TeleOp(name = "Calibrate: Hive Shot Table", group = "Test")
public class ShotCalibrationOpMode extends LinearOpMode {
    public static final double DISTANCE_STEP_IN = 1, NEXT_DISTANCE_STEP_IN = 12;
    public static final double RPM_STEP = 50, HOOD_STEP_DEG = .5;
    private enum Step { PLACE, TUNE, FIRING, RATE, REVIEW }
    private Step step = Step.PLACE;
    private final ShotCalibration notebook = new ShotCalibration();
    private String message = "Measure horizontal distance from turret axis to raised cell opening center.";

    @Override public void runOpMode() {
        TurretSubsystem turret = new TurretSubsystem(hardwareMap, false);
        GateSubsystem gate = null;
        TransferSubsystem transfer = null;
        FiringCoordinator firing = ShootingConfiguration.firing();
        try {
            gate = new GateSubsystem(hardwareMap);
            transfer = new TransferSubsystem(hardwareMap);
            while (opModeInInit()) {
                turret.update();
                telemetry.addLine("Stationary calibration: turret + gate + both transfer stages + full sensor.");
                telemetry.addLine("No drivetrain, intake, odometry or camera. Load ONE ball per trial.");
                telemetry.addLine("Keep the same hive opening/height and muzzle reference for this session.");
                turret.addTelemetry(telemetry);
                telemetry.update();
                idle();
            }
            waitForStart();
            if (isStopRequested()) return;
            Gamepad previous = new Gamepad();
            Gamepad current = new Gamepad();
            previous.copy(gamepad1);
            double lastTime = System.nanoTime() * 1e-9;
            int attemptsBefore = 0;
            double previewInches = notebook.distanceInches();
            while (opModeIsActive()) {
                double now = System.nanoTime() * 1e-9;
                double dt = Math.max(0, Math.min(.1, now - lastTime));
                lastTime = now;
                current.copy(gamepad1);
                turret.update();
                boolean a = current.a && !previous.a, b = current.b && !previous.b;
                boolean x = current.x && !previous.x, y = current.y && !previous.y;
                boolean up = current.dpad_up && !previous.dpad_up;
                boolean down = current.dpad_down && !previous.dpad_down;
                boolean left = current.dpad_left && !previous.dpad_left;
                boolean right = current.dpad_right && !previous.dpad_right;
                Step enteredStep = step;

                if (step == Step.PLACE) {
                    double inches = Math.max(1, notebook.distanceInches()
                            + (right ? DISTANCE_STEP_IN : 0) - (left ? DISTANCE_STEP_IN : 0));
                    notebook.configure(inches, notebook.rpm(), notebook.hoodDeg());
                    if (a && turret.isHomed()) {
                        step = Step.TUNE;
                        message = "Load ONE ball; aim with RS-X. Hold RB to spin; X requests one trial.";
                    }
                } else if (step == Step.TUNE) {
                    notebook.configure(notebook.distanceInches(),
                            Math.max(RPM_STEP, Math.min(TurretSubsystem.SHOOTER_MOTOR_MAX_RPM,
                                    notebook.rpm() + (right ? RPM_STEP : 0) - (left ? RPM_STEP : 0))),
                            Math.max(TurretSubsystem.PITCH_MIN_DEG, Math.min(TurretSubsystem.PITCH_MAX_DEG,
                                    notebook.hoodDeg() - (up ? HOOD_STEP_DEG : 0) + (down ? HOOD_STEP_DEG : 0))));
                    turret.setPitchAngleDegrees(notebook.hoodDeg());
                    turret.updateManualYaw(current.right_stick_x, dt, false);
                    if (b) { step = Step.PLACE; message = "Recheck measured distance before tuning."; }
                    else if (y) {
                        if (notebook.hits() > 0) {
                            stopOutputs(turret, gate, transfer, firing, now);
                            notebook.configure(notebook.distanceInches() + NEXT_DISTANCE_STEP_IN,
                                    notebook.rpm(), notebook.hoodDeg());
                            step = Step.PLACE;
                            message = "Averages remain on screen. Move robot, measure next distance; A confirms.";
                        } else message = "Record at least one HIT for this distance before moving on with Y.";
                    } else if (x) {
                        if (current.right_bumper && turret.isHomed()) {
                            firing.stop(now);
                            attemptsBefore = firing.feedAttempts();
                            step = Step.FIRING;
                            message = "Hold RB through trial. LB aborts. Wait for result prompt.";
                        } else message = "Hold RB to spin and wait for turret home before requesting a trial.";
                    }
                } else if (step == Step.RATE) {
                    if (a || b) {
                        notebook.record(a);
                        stopOutputs(turret, gate, transfer, firing, now);
                        step = Step.TUNE;
                        message = a ? "HIT included in distance average. Take more shots or Y for next distance."
                                : "MISS excluded from average. Adjust RPM/hood and retest.";
                    } else if (x) {
                        step = Step.TUNE;
                        message = "Trial discarded (no launch / uncertain result). Reload ONE ball.";
                    }
                } else if (step == Step.REVIEW) {
                    previewInches = Math.max(1, previewInches + (right ? DISTANCE_STEP_IN : 0)
                            - (left ? DISTANCE_STEP_IN : 0));
                }

                // Review is available only when there is no in-flight trial or unrecorded result.
                if (current.back && !previous.back && (enteredStep == Step.PLACE
                        || enteredStep == Step.TUNE || enteredStep == Step.REVIEW)) {
                    step = enteredStep == Step.REVIEW ? Step.PLACE : Step.REVIEW;
                    previewInches = notebook.distanceInches();
                    stopOutputs(turret, gate, transfer, firing, now);
                }
                if (current.left_stick_button && !previous.left_stick_button
                        && (step == Step.PLACE || step == Step.TUNE)) {
                    step = Step.PLACE;
                    stopOutputs(turret, gate, transfer, firing, now);
                    turret.homeTurretAndZeroEncoder();
                }
                boolean abort = current.left_bumper || (step == Step.FIRING && !current.right_bumper);
                if (abort) {
                    step = Step.PLACE;
                    stopOutputs(turret, gate, transfer, firing, now);
                    message = "Aborted; no result recorded. A returns to tuning after checking distance.";
                }
                boolean spin = !abort && current.right_bumper && turret.isHomed()
                        && (step == Step.TUNE || step == Step.FIRING);
                if (spin) turret.setShooterRpm(notebook.rpm()); else turret.stopShooter();
                if (step == Step.FIRING) {
                    boolean tracking = turret.isShooterAtSpeed() && turret.isHoodTracking()
                            && !turret.isRotationErrorHigh();
                    firing.update(now, true, false, false, !spin, turret.isHomed(), tracking, tracking,
                            "Turret not homed");
                    if (firing.state() == FiringCoordinator.State.RECOVERY) {
                        if (firing.feedAttempts() > attemptsBefore) {
                            step = Step.RATE;
                            message = "Observe ball: A = HIT, B = MISS, X = discard (no ball / uncertain).";
                        } else {
                            step = Step.TUNE;
                            message = "Readiness lost before feeding. No trial recorded; retry X.";
                        }
                        stopOutputs(turret, gate, transfer, firing, now);
                    }
                } else firing.stop(now);
                gate.setOpen(step == Step.FIRING && firing.gateOpen());
                transfer.update(step == Step.FIRING && firing.feeding() && gate.isOpenAndSettled(), true);

                telemetry.addData("STEP", step);
                telemetry.addLine(message);
                telemetry.addData("Turret axis to raised opening, horizontal (in)", "%.1f", notebook.distanceInches());
                telemetry.addData("Candidate", "%.0f motor RPM | %.1f deg travel | launch %+.1f deg",
                        notebook.rpm(), notebook.hoodDeg(), TurretSubsystem.hoodTravelToLaunchAngleDegrees(notebook.hoodDeg()));
                telemetry.addData("Samples", "%d confirmed HITs here (unlimited) | %d distances",
                        notebook.hits(), notebook.size());
                ShotProfile.Row average = notebook.currentAverage();
                if (average != null) telemetry.addData("Distance average", "%.1f RPM | %.2f deg travel | launch %+.2f deg",
                        average.rpm, average.hoodDeg, TurretSubsystem.hoodTravelToLaunchAngleDegrees(average.hoodDeg));
                if (step == Step.PLACE) telemetry.addLine("ENTER DISTANCE HERE: Dpad L/R +/-1 in | A: confirm | LS click: rehome");
                if (step == Step.TUNE) telemetry.addLine("Dpad L/R: RPM +/-50 | U/D: raise/lower launch 0.5 deg | RS-X: aim\nRB hold: spin | X: trial | Y: next distance | B: edit distance");
                if (step == Step.FIRING) telemetry.addData("Feed", firing.reason());
                if (step == Step.REVIEW) {
                    telemetry.addLine("Dpad L/R: query +/-1 in | BACK: return to measured-distance entry");
                    telemetry.addData("Lookup range (in)", previewInches);
                    ShotProfile.Row row = notebook.size() < 2 ? null
                            : notebook.profile("session-preview").sample(previewInches * ShotCalibration.METERS_PER_INCH);
                    telemetry.addData("Interpolated lookup", row == null ? "Need two distances / range outside table"
                            : String.format(Locale.US, "%.0f motor RPM | %.2f deg travel | launch %+.2f deg",
                                    row.rpm, row.hoodDeg, TurretSubsystem.hoodTravelToLaunchAngleDegrees(row.hoodDeg)));
                }
                telemetry.addLine("LIVE AVERAGES: distance (in) | motor RPM | launch deg above horizontal");
                int rowNumber = 0;
                for (ShotProfile.Row row : notebook.rows()) {
                    telemetry.addData("Average " + (++rowNumber), "%.1f in | %.1f RPM | %+.2f deg (travel %.2f deg)",
                            row.distance / ShotCalibration.METERS_PER_INCH, row.rpm,
                            TurretSubsystem.hoodTravelToLaunchAngleDegrees(row.hoodDeg), row.hoodDeg);
                }
                telemetry.addLine("Record these rows before STOP; session values reset on a new run.");
                telemetry.addLine("LB: abort + stop | BACK: review (PLACE/TUNE) | STOP: exit");
                turret.addTelemetry(telemetry);
                telemetry.update();
                previous.copy(current);
                idle();
            }
        } finally {
            turret.stop();
            if (gate != null) gate.close();
            if (transfer != null) transfer.stop();
        }
    }

    private void stopOutputs(TurretSubsystem turret, GateSubsystem gate, TransferSubsystem transfer,
                             FiringCoordinator firing, double now) {
        turret.stopShooter(); gate.close(); transfer.stop(); firing.stop(now);
    }

}
