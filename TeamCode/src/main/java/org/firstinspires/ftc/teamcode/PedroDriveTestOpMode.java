package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Isolated Pedro drive check. Requires only the four drive motors and Pinpoint. */
@TeleOp(name = "Test: Pedro Drive", group = "Test")
public final class PedroDriveTestOpMode extends LinearOpMode {
    public static final double MANUAL_POWER_LIMIT = .2;
    public static final double TEST_DISTANCE_IN = 12;
    public static final double TEST_TIMEOUT_SECONDS = 5;
    @Override public void runOpMode() {
        Scheduler.reset();
        RobotLoop loop = new RobotLoop(hardwareMap);
        PedroDrive drive = new PedroDrive(hardwareMap);
        Command[] active = {null};
        try {
            drive.setStartPose(Pose.zero());
            while (opModeInInit()) {
                loop.begin(); drive.readSensors();
                telemetry.addLine("Keep stationary during INIT. Use a cleared area for motion tests.");
                telemetry.addLine("Sticks: 0.2 power; A: 12 in forward; B: 90 deg CCW; X: stop.");
                drive.addTelemetry(telemetry); loop.addTelemetry(telemetry); telemetry.update(); idle();
            }
            waitForStart(); if (isStopRequested()) return;
            boolean previousA = false, previousB = false;
            while (opModeIsActive()) {
                loop.begin(); drive.readSensors();
                boolean manual = Math.abs(gamepad1.left_stick_y) + Math.abs(gamepad1.left_stick_x)
                        + Math.abs(gamepad1.right_stick_x) > .05;
                if (gamepad1.x || manual || !drive.ready(System.nanoTime() * 1e-9)) {
                    if (active[0] != null) Scheduler.cancel(active[0]);
                    active[0] = null; drive.stop();
                }
                if (drive.ready(System.nanoTime() * 1e-9) && !gamepad1.x) {
                    if (manual) drive.follower().manual(-gamepad1.left_stick_y * MANUAL_POWER_LIMIT,
                            -gamepad1.left_stick_x * MANUAL_POWER_LIMIT, -gamepad1.right_stick_x * MANUAL_POWER_LIMIT);
                    else if ((gamepad1.a && !previousA) || (gamepad1.b && !previousB)) {
                        Pose from = drive.follower().pose();
                        boolean turn = gamepad1.b;
                        Pose to = turn ? new Pose(from.x(), from.y(), from.heading() + Math.PI / 2)
                                : new Pose(from.x() + TEST_DISTANCE_IN * Math.cos(from.heading()),
                                from.y() + TEST_DISTANCE_IN * Math.sin(from.heading()), from.heading());
                        long[] start = {0};
                        active[0] = new CommandBuilder().requiring(drive)
                                .setStart(() -> {
                                    start[0] = System.nanoTime();
                                    if (turn) drive.follower().hold(to);
                                    else drive.follower().follow(drive.atSpeed(Paths.line(from, to).constant(from.heading()), .2));
                                })
                                .setDone(() -> (System.nanoTime() - start[0]) * 1e-9 >= TEST_TIMEOUT_SECONDS
                                        || (turn ? Math.abs(RobotMath.wrap(to.heading() - drive.follower().pose().heading())) < Math.toRadians(3)
                                        : !drive.follower().following()))
                                .setEnd(reason -> drive.stop());
                        Scheduler.schedule(active[0]);
                    } else if (active[0] == null || !Scheduler.isScheduled(active[0])) {
                        drive.stop(); // Releasing the sticks must not leave Pedro in its last MANUAL command.
                    }
                }
                previousA = gamepad1.a; previousB = gamepad1.b;
                Scheduler.execute(); drive.update();
                drive.addTelemetry(telemetry); loop.addTelemetry(telemetry); telemetry.update(); idle();
            }
        } finally {
            RobotShutdown.stopAll(active[0] == null ? null : () -> Scheduler.cancel(active[0]), drive::stop, Scheduler::reset);
        }
    }
}
