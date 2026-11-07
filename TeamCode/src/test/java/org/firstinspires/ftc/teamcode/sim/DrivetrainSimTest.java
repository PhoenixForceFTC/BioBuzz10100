package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.horizon36596.simloop.opmode.GamepadScript;
import org.horizon36596.simloop.opmode.OpModePhase;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class DrivetrainSimTest {
    @Test void allDriveOpModesFollowAllSixStickDirections() throws Exception {
        String[] directions = {"forward", "backward", "left", "right", "ccw", "cw"};
        // Expected chassis +X forward, +Y left, heading CCW; independent of production mixing.
        double[][] expected = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0},
                {0, 0, 1}, {0, 0, -1}};
        for (String opMode : new String[] {"DrivetrainTestOpMode", "RedMecanumTeleOp",
                "BlueMecanumTeleOp", "PedroDriveTestOpMode"}) {
            for (int axis = 0; axis < directions.length; axis++) {
                final int input = axis;
                BioBuzzSimHardware h = opMode.equals("DrivetrainTestOpMode") ? BioBuzzSimHardware.drivetrain()
                        : opMode.equals("PedroDriveTestOpMode") ? BioBuzzSimHardware.drivetrain().withPinpoint()
                        : BioBuzzSimHardware.fullRobot();
                TeleOpScenario s = new TeleOpScenario("drive-direction-" + opMode + "-" + directions[axis],
                        opMode, h, .02, 160, 25, (phase, tick, gp1, gp2) -> {
                    if (phase != OpModePhase.ACTIVE) return;
                    if (input < 2) gp1.left_stick_y = input == 0 ? -.5f : .5f;
                    else if (input < 4) gp1.left_stick_x = input == 2 ? -.5f : .5f;
                    else gp1.right_stick_x = input == 4 ? -.5f : .5f;
                });
                s.run();
                assertFalse(s.result.endedBeforeStop(), s.diagnostic());
                assertTrue(s.result.warnings().isEmpty(), s.diagnostic());
                TeleOpTrace.Sample last = s.trace.at(OpModePhase.ACTIVE, 24);
                double[] actual = {last.xIn, last.yIn, last.headingRad};
                for (int coordinate = 0; coordinate < actual.length; coordinate++) {
                    if (expected[axis][coordinate] == 0) {
                        assertEquals(0, actual[coordinate], 1e-9, s.diagnostic());
                    } else {
                        assertTrue(actual[coordinate] * expected[axis][coordinate] > .01, s.diagnostic());
                    }
                }
                assertArrayEquals(new double[4], s.trace.at(OpModePhase.STOP, 0).powers, 0, s.diagnostic());
            }
        }
    }

    @Test void allDriveOpModesConvertMotorFeedbackThroughBothReductionStages() throws Exception {
        for (String opMode : new String[] {"DrivetrainTestOpMode", "RedMecanumTeleOp",
                "BlueMecanumTeleOp", "PedroDriveTestOpMode"}) {
            BioBuzzSimHardware h = opMode.equals("DrivetrainTestOpMode") ? BioBuzzSimHardware.drivetrain()
                    : opMode.equals("PedroDriveTestOpMode") ? BioBuzzSimHardware.drivetrain().withPinpoint()
                    : BioBuzzSimHardware.fullRobot();
            TeleOpScenario s = new TeleOpScenario("drive-transmission-" + opMode, opMode,
                    h, .02, 160, 30, (phase, tick, gp1, gp2) ->
                    gp1.left_stick_y = phase == OpModePhase.ACTIVE ? -.5f : 0);
            s.run();
            assertFalse(s.result.endedBeforeStop(), s.diagnostic());
            assertTrue(s.result.warnings().isEmpty(), s.diagnostic());
            TeleOpTrace.Sample before = s.trace.at(OpModePhase.ACTIVE, 20);
            TeleOpTrace.Sample after = s.trace.at(OpModePhase.ACTIVE, 21);
            // Independent physical specification: 28 motor ticks/rev, 20:80 gears, 30:91 belt.
            // Nominal diameter is the team's 104-mm goBILDA 30A mecanum wheel.
            // Installed right-hand shaft-to-wheel sign from the team's reversal report.
            double expectedInPerSecond = -after.shaftTicksPerSecond[1] / 28
                    * (20.0 / 80) * (30.0 / 91) * (Math.PI * 104 / 25.4);
            assertTrue(expectedInPerSecond > 0, s.diagnostic());
            assertEquals(expectedInPerSecond, (after.xIn - before.xIn) / .02, 1e-8, s.diagnostic());
            assertEquals(0, after.yIn, 1e-9, s.diagnostic());
            assertEquals(0, after.headingRad, 1e-9, s.diagnostic());
            assertArrayEquals(new double[4], s.trace.at(OpModePhase.STOP, 0).powers, 0, s.diagnostic());
        }
    }

    private TeleOpScenario drive(String name, double dt, int activeTicks, GamepadScript inputs) throws Exception {
        TeleOpScenario s = new TeleOpScenario(name, "DrivetrainTestOpMode", BioBuzzSimHardware.drivetrain(),
                dt, 3, activeTicks, inputs);
        s.run();
        assertFalse(s.result.endedBeforeStop(), s.diagnostic());
        assertTrue(s.result.warnings().isEmpty(), s.result.warnings() + "; " + s.diagnostic());
        assertEquals(5 + activeTicks, s.result.ticks(), s.diagnostic());
        // INIT and each ACTIVE loop flush once; waiting at waitForStart does not run a control loop.
        assertEquals(1 + activeTicks, s.result.telemetry().transmissions().size(), s.diagnostic());
        assertArrayEquals(new double[4], s.trace.at(OpModePhase.INIT, 0).powers, 0, s.diagnostic());
        assertArrayEquals(new double[4], s.trace.at(OpModePhase.STOP, 0).powers, 0, s.diagnostic());
        return s;
    }

    @Test void lifecycleForwardAndNeutralAtBothTickRates() throws Exception {
        for (double dt : new double[] {.02, .01}) {
            int ticks = (int) Math.round(1 / dt);
            TeleOpScenario s = drive("forward-neutral", dt, ticks, (phase, tick, gp1, gp2) ->
                    gp1.left_stick_y = phase == OpModePhase.ACTIVE && tick < ticks / 2 ? -1 : 0);
            assertArrayEquals(new double[] {1, 1, 1, 1}, s.trace.at(OpModePhase.ACTIVE, 0).powers, 0, s.diagnostic());
            assertArrayEquals(new double[4], s.trace.at(OpModePhase.ACTIVE, ticks / 2).powers, 0, s.diagnostic());
            assertTrue(s.model.drive.getX() > 1, s.diagnostic());
            assertEquals(0, s.model.drive.getY(), 1e-9, s.diagnostic());
            assertEquals(0, s.model.drive.getHeading(), 1e-9, s.diagnostic());
            assertTrue(s.trace.at(OpModePhase.ACTIVE, ticks / 4).shaftTicksPerSecond[0] > 0, s.diagnostic());
            assertTrue(s.trace.at(OpModePhase.ACTIVE, ticks / 4).shaftTicksPerSecond[1] < 0, s.diagnostic());
            for (int i = 0; i < 4; i++) {
                assertEquals(i % 2 == 0 ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE,
                        s.hardware.wheels[i].getDirection(), s.diagnostic());
                assertEquals(DcMotor.ZeroPowerBehavior.BRAKE, s.hardware.wheels[i].getZeroPowerBehavior());
                assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, s.hardware.wheels[i].getMode());
                assertEquals(28, s.hardware.wheels[i].getMotorType().getTicksPerRev(), 0);
                assertEquals(5800, s.hardware.wheels[i].getMotorType().getMaxRPM(), 0);
                assertEquals(1, s.hardware.wheels[i].getMotorType().getGearing(), 0);
            }
            assertTrue(s.hardware.map.getAll(LynxModule.class).isEmpty());
            assertThrows(IllegalArgumentException.class, () -> s.hardware.map.get(Servo.class, "gate_servo"));
        }
    }

    @Test void reverseStrafeAndClockwiseHaveIndependentModelSigns() throws Exception {
        TeleOpScenario reverse = drive("reverse", .02, 25, (p, t, g, g2) -> g.left_stick_y = 1);
        assertArrayEquals(new double[] {-1, -1, -1, -1}, reverse.trace.at(OpModePhase.ACTIVE, 0).powers, 0);
        assertTrue(reverse.model.drive.getX() < -1, reverse.diagnostic());
        TeleOpScenario strafe = drive("strafe-right", .02, 25, (p, t, g, g2) -> g.left_stick_x = 1);
        assertArrayEquals(new double[] {1, -1, -1, 1}, strafe.trace.at(OpModePhase.ACTIVE, 0).powers, 0);
        assertTrue(strafe.model.drive.getY() < -1, strafe.diagnostic()); // Localizer +Y is left.
        assertEquals(0, strafe.model.drive.getHeading(), 1e-9);
        TeleOpScenario turn = drive("clockwise", .02, 25, (p, t, g, g2) -> g.right_stick_x = 1);
        assertArrayEquals(new double[] {1, -1, 1, -1}, turn.trace.at(OpModePhase.ACTIVE, 0).powers, 0);
        assertTrue(turn.model.drive.getHeading() < -.1, turn.diagnostic());
        assertEquals(0, turn.model.drive.getX(), 1e-9);
        assertEquals(0, turn.model.drive.getY(), 1e-9);
    }

    @Test void diagonalNormalizesAndWheelButtonsSelectOnlyTheirWheel() throws Exception {
        TeleOpScenario diagonal = drive("diagonal", .02, 5, (p, t, g, g2) -> {
            g.left_stick_y = -1; g.left_stick_x = 1;
        });
        assertArrayEquals(new double[] {1, -1.0 / 21, -1.0 / 21, 1},
                diagonal.trace.at(OpModePhase.ACTIVE, 0).powers, 1e-9, diagonal.diagnostic());
        TeleOpScenario selected = drive("wheel-selection", .02, 4, (p, t, g, g2) -> {
            g.right_bumper = true;
            g.x = t == 0; g.y = t == 1; g.a = t == 2; g.b = t == 3;
        });
        for (int wheel = 0; wheel < 4; wheel++) {
            double[] expected = new double[4]; expected[wheel] = .25;
            assertArrayEquals(expected, selected.trace.at(OpModePhase.ACTIVE, wheel).powers, 0, selected.diagnostic());
        }
    }

    @Test void stopBeforeStartLeavesAllMotorsStopped() throws Exception {
        drive("stop-before-start", .02, 0, (p, t, g, g2) -> g.left_stick_y = -1);
    }

    @Test void aliasesWorkAndMissingRequiredHardwareFailsInit() throws Exception {
        for (String[] names : new String[][] {
                {"front_left_drive", "front_right_drive", "back_left_drive", "back_right_drive"},
                {"frontLeft", "frontRight", "backLeft", "backRight"}}) {
            TeleOpScenario s = new TeleOpScenario("aliases", "DrivetrainTestOpMode",
                    BioBuzzSimHardware.drivetrain(names), .02, 1, 2, (p, t, g, g2) -> g.left_stick_y = -1);
            s.run();
            assertArrayEquals(new double[] {1, 1, 1, 1}, s.trace.at(OpModePhase.ACTIVE, 0).powers, 0);
        }
        TeleOpScenario missing = new TeleOpScenario("missing-drive", "DrivetrainTestOpMode",
                new BioBuzzSimHardware(), .02, 1, 1, GamepadScript.NONE);
        RuntimeException failure = assertThrows(RuntimeException.class, missing::run);
        assertTrue(causes(failure).contains("leftfront"), causes(failure));
    }

    @Test void repeatRunsHaveIdenticalDecodedRobotData() throws Exception {
        GamepadScript script = (p, t, g, g2) -> {
            g.left_stick_y = t < 10 ? -1 : 0; g.right_stick_x = t >= 10 ? .5f : 0;
        };
        TeleOpScenario a = drive("determinism-a", .02, 20, script);
        TeleOpScenario b = drive("determinism-b", .02, 20, script);
        assertEquals(a.trace.decoded(), b.trace.decoded(), a.diagnostic() + "; " + b.diagnostic());
    }

    static String causes(Throwable failure) {
        StringBuilder messages = new StringBuilder();
        for (Throwable t = failure; t != null; t = t.getCause()) messages.append(t).append('\n');
        return messages.toString();
    }
}
