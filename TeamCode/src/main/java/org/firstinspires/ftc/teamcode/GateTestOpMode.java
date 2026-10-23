package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Tests only the gate servo, without running the flywheel or other mechanisms. */
@TeleOp(name = "Test: Gate Only", group = "Test")
public class GateTestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        GateSubsystem gate = new GateSubsystem(hardwareMap);
        telemetry.addData("GP1 LT = Open | Release = Closed", "%.1f / %.1f deg",
                GateSubsystem.OPEN_ANGLE_DEG, GateSubsystem.CLOSED_ANGLE_DEG);
        gate.addTelemetry(telemetry);
        telemetry.update();

        waitForStart();

        try {
            while (opModeIsActive()) {
                gate.setOpen(gamepad1.left_trigger > GateSubsystem.TRIGGER_DEADBAND);
                gate.addTelemetry(telemetry);
                telemetry.update();
            }
        } finally {
            gate.close();
        }
    }
}
