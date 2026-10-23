package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Tests automatic stage 1 loading and the full sensor; stage 2 remains stopped. */
@TeleOp(name = "Test: Transfer Only", group = "Test")
public class TransferTestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        TransferSubsystem transfer = new TransferSubsystem(hardwareMap);
        telemetry.addLine("transfer_motor_1: Expansion Hub motor port 2, REVERSE, no encoder");
        telemetry.addLine("transfer_motor_2: Expansion Hub motor port 3, FORWARD, no encoder (stopped in this test)");
        telemetry.addLine("distance_sensor: Expansion Hub digital port 0, Digital Device input");
        telemetry.addLine("After START: HIGH continuously for 2 s = Stop | LOW = Full power");
        transfer.addTelemetry(telemetry);
        telemetry.update();

        waitForStart();

        try {
            while (opModeIsActive()) {
                transfer.update();
                transfer.addTelemetry(telemetry);
                telemetry.update();
            }
        } finally {
            transfer.stop();
        }
    }
}
