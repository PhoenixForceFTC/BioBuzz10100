package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Tests only the depollinator servo, with the same stick-click controls as competition TeleOp. */
@TeleOp(name = "Test: Depollinator Only", group = "Test")
public class DepollinatorTestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        DepollinatorSubsystem depollinator = new DepollinatorSubsystem(hardwareMap);
        telemetry.addLine("Expansion Hub servo 4: depollinator | RS Click = Deploy | LS Click = Home");
        depollinator.addTelemetry(telemetry);
        telemetry.update();

        waitForStart();

        boolean previousDeploy = false;
        boolean previousHome = false;
        while (opModeIsActive()) {
            boolean deployPressed = gamepad1.right_stick_button;
            boolean homePressed = gamepad1.left_stick_button;
            if (homePressed && !previousHome) {
                depollinator.home();
            } else if (deployPressed && !previousDeploy && !homePressed) {
                depollinator.deploy();
            }
            previousDeploy = deployPressed;
            previousHome = homePressed;
            depollinator.addTelemetry(telemetry);
            telemetry.update();
        }
    }
}
