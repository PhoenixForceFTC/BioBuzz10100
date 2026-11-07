package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Isolated light wiring/palette diagnostic. No mechanism hardware is initialized. */
@TeleOp(name = "Test: Status Lights", group = "Test")
public final class LightsTestOpMode extends LinearOpMode {
    @Override public void runOpMode() {
        RobotStatusLights lights = new RobotStatusLights(hardwareMap);
        try {
            telemetry.addLine("Dpad L/R selects intake/shooter/reference | A green B red X blue Y yellow | RB white | LB off");
            lights.addTelemetry(telemetry); telemetry.update();
            waitForStart();
            int selected = 0;
            boolean previousLeft = false, previousRight = false;
            while (opModeIsActive()) {
                if (gamepad1.dpad_left && !previousLeft) selected = (selected + 2) % 3;
                if (gamepad1.dpad_right && !previousRight) selected = (selected + 1) % 3;
                previousLeft = gamepad1.dpad_left; previousRight = gamepad1.dpad_right;
                if (gamepad1.left_bumper) lights.set(selected, RobotStatusLights.Color.OFF);
                else if (gamepad1.a) lights.set(selected, RobotStatusLights.Color.GREEN);
                else if (gamepad1.b) lights.set(selected, RobotStatusLights.Color.RED);
                else if (gamepad1.x) lights.set(selected, RobotStatusLights.Color.BLUE);
                else if (gamepad1.y) lights.set(selected, RobotStatusLights.Color.YELLOW);
                else if (gamepad1.right_bumper) lights.set(selected, RobotStatusLights.Color.WHITE);
                telemetry.addData("Selected light", selected == 0 ? "intake" : selected == 1 ? "shooter" : "reference");
                lights.addTelemetry(telemetry); telemetry.update();
            }
        } finally { lights.stop(); }
    }
}
