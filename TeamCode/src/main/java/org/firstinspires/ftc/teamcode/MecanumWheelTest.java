package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/** Open-loop test: a broken encoder cannot cause the hub to demand full power. */
@TeleOp(name = "Mecanum Wheel Test", group = "Drive")
public class MecanumWheelTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotorEx[] wheels = new DcMotorEx[MecanumTeleOp.DRIVE_NAMES.length];
        for (int i = 0; i < wheels.length; i++) {
            wheels[i] = hardwareMap.get(DcMotorEx.class, MecanumTeleOp.DRIVE_NAMES[i]);
            wheels[i].setPower(0);
            wheels[i].setDirection(MecanumTeleOp.DRIVE_DIRECTIONS[i]);
            wheels[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            wheels[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        telemetry.addLine("SUPPORT ROBOT WITH ALL WHEELS OFF THE GROUND; keep clear");
        telemetry.addLine("Hold right bumper + ONE button: X=FL, Y=FR, A=BL, B=BR");
        telemetry.addLine("15% forward power; release to stop. Left bumper reverses test.");
        telemetry.addLine("Positive power must roll the wheel forward and increase its encoder.");
        telemetry.setMsTransmissionInterval(100);
        telemetry.update();
        waitForStart();

        try {
            while (opModeIsActive()) {
                int pressed = (gamepad1.x ? 1 : 0) + (gamepad1.y ? 1 : 0)
                        + (gamepad1.a ? 1 : 0) + (gamepad1.b ? 1 : 0);
                int selected = gamepad1.x ? 0 : gamepad1.y ? 1 : gamepad1.a ? 2 : 3;
                // Stop all other wheels BEFORE energizing the selected wheel.
                for (int i = 0; i < wheels.length; i++) {
                    if (!gamepad1.right_bumper || pressed != 1 || i != selected) {
                        wheels[i].setPower(0);
                    }
                }
                if (gamepad1.right_bumper && pressed == 1) {
                    wheels[selected].setPower(gamepad1.left_bumper ? -0.15 : 0.15);
                }
                telemetry.addLine("RB + X=FL / Y=FR / A=BL / B=BR; LB reverses; release stops");
                for (int i = 0; i < wheels.length; i++) {
                    telemetry.addData(MecanumTeleOp.DRIVE_NAMES[i],
                            "%s | power %.2f | pos %d | speed %.0f ticks/s",
                            wheels[i].getDirection(), wheels[i].getPower(),
                            wheels[i].getCurrentPosition(), wheels[i].getVelocity());
                }
                telemetry.update();
                idle();
            }
        } finally {
            for (DcMotorEx wheel : wheels) {
                wheel.setPower(0);
            }
        }
    }
}
