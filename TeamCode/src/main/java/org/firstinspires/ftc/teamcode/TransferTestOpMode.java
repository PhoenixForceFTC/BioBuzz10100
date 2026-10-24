package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Tests stage 1/full sensor plus a deliberate stage-2 hold with gate closed. */
@TeleOp(name = "Test: Transfer Only", group = "Test")
public class TransferTestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        RobotLoop loop = new RobotLoop(hardwareMap);
        GateSubsystem gate = new GateSubsystem(hardwareMap);
        TransferSubsystem transfer = new TransferSubsystem(hardwareMap);
        telemetry.addLine("transfer_motor_1: Expansion Hub motor port 2, REVERSE, no encoder");
        telemetry.addLine("transfer_motor_2: Expansion Hub motor port 3, FORWARD, no encoder | hold RB for stage-2-only test, gate closed");
        telemetry.addLine("distance_sensor: Expansion Hub digital port 0, Digital Device input");
        telemetry.addLine("After START: HIGH continuously for 2 s = Stop | LOW = Full power");
        transfer.addTelemetry(telemetry);
        loop.addTelemetry(telemetry); telemetry.update();

        waitForStart();
        boolean previousStage2 = false;

        try {
            while (opModeIsActive()) {
                loop.begin();
                gate.close();
                if (gamepad1.right_bumper) transfer.diagnosticStage2(true);
                else transfer.update(false, gamepad1.left_bumper || previousStage2);
                previousStage2 = gamepad1.right_bumper;
                transfer.addTelemetry(telemetry);
                loop.addTelemetry(telemetry); telemetry.update();
            }
        } finally {
            RobotShutdown.stopAll(transfer::stop, gate::close);
        }
    }
}
