package org.firstinspires.ftc.teamcode.sim;

import org.junit.jupiter.api.Test;
import org.horizon36596.simloop.opmode.OpModePhase;
import static org.junit.jupiter.api.Assertions.*;

class MechanismsSimTest {
    @Test void intakePrioritiesAndStop() throws Exception {
        TeleOpScenario s = new TeleOpScenario("intake", "IntakeTestOpMode",
                new BioBuzzSimHardware().withIntake(), .02, 5, 80, (p,t,a,b) -> {
            a.right_bumper = p == OpModePhase.ACTIVE && t < 40;
            a.left_bumper = p == OpModePhase.ACTIVE && t >= 20 && t < 60;
        });
        s.run();
        assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 10).intakePower);
        assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 30).intakePower);
        assertEquals(-1, s.trace.at(OpModePhase.ACTIVE, 50).intakePower);
        assertEquals(0, s.hardware.intake.getPower());
    }

    @Test void transferFullDwellTimedFeedAndInhibit() throws Exception {
        BioBuzzSimHardware h = new BioBuzzSimHardware().withTransfer();
        TeleOpScenario s = new TeleOpScenario("transfer", "TransferTestOpMode", h, .02, 5, 230, (p,t,a,b) -> {
            h.distance.setSimulatedInputState(p == OpModePhase.ACTIVE && t < 130);
            a.right_bumper = p == OpModePhase.ACTIVE && t >= 140;
            a.left_bumper = p == OpModePhase.ACTIVE && t >= 210;
        });
        s.run();
        assertTrue(s.trace.at(OpModePhase.ACTIVE, 20).transfer1Power > 0);
        assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 20).transfer2Power);
        assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 35).transfer1Power);
        assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 55).transfer1Power);
        assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 120).transfer1Power);
        assertTrue(s.trace.samples.stream().anyMatch(v -> v.transfer2Power > 0 && Math.abs(v.gatePosition - .2) < 1e-9));
        assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 215).transfer1Power);
        assertEquals(.5, s.trace.at(OpModePhase.ACTIVE, 215).gatePosition);
        assertEquals(0, h.transfer1.getPower()); assertEquals(0, h.transfer2.getPower());
        assertEquals(.5, h.gate.getPosition());
    }

    @Test void depollinatorEdgesHomePriorityAndPwm() throws Exception {
        TeleOpScenario s = new TeleOpScenario("depollinator", "DepollinatorTestOpMode",
                new BioBuzzSimHardware().withDepollinator(), .02, 5, 60, (p,t,a,b) -> {
            a.right_stick_button = p == OpModePhase.ACTIVE && t >= 10;
            a.left_stick_button = p == OpModePhase.ACTIVE && t >= 30;
        });
        s.run();
        assertEquals(165.0/355, s.trace.at(OpModePhase.ACTIVE, 20).depollinator, 1e-9);
        assertEquals(30.0/355, s.trace.at(OpModePhase.ACTIVE, 40).depollinator, 1e-9);
        assertEquals(Math.round(500 + 2000 * 30.0/355), s.hardware.depollinator.getPulseWidth(), 1e-6);
    }

    @Test void turretIndependentEncoderHoodEdgesAndFlywheel() throws Exception {
        TeleOpScenario s = new TeleOpScenario("turret", "TurretTestOpMode",
                new BioBuzzSimHardware().withTurret(), .02, 160, 160, (p,t,a,b) -> {
            a.right_stick_x = p == OpModePhase.ACTIVE && t < 60 ? .5f : 0;
            a.right_bumper = p == OpModePhase.ACTIVE && t < 110;
            a.dpad_up = p == OpModePhase.ACTIVE && t < 60;
        });
        s.run();
        assertTrue(Math.abs(s.trace.at(OpModePhase.ACTIVE, 60).yaw) > 5, s.diagnostic());
        assertTrue(s.trace.at(OpModePhase.ACTIVE, 90).flywheelVelocity > 1000, s.diagnostic());
        // Installed home is 8 hood degrees below the original reference; held D-pad adds one 0.5-degree step.
        assertEquals(.5 + (8 + .5) * 2 / 355, s.trace.at(OpModePhase.ACTIVE, 40).hood, 1e-8);
        assertEquals(s.hardware.yaw1.getPosition(), s.hardware.yaw2.getPosition());
        assertEquals(0, s.hardware.flywheel.getPower());
    }

    @Test void benchActualOpModeStartsAndStopsSafely() throws Exception {
        BioBuzzSimHardware h = new BioBuzzSimHardware().withIntake().withTransfer().withTurret();
        TeleOpScenario s = new TeleOpScenario("bench", "IntakeShooterTestOpMode", h, .02, 160, 100,
                (p,t,a,b) -> a.right_bumper = p == OpModePhase.ACTIVE && t > 10 && t < 60);
        s.run();
        assertFalse(s.result.endedBeforeStop(), s.diagnostic());
        assertTrue(s.trace.samples.stream().anyMatch(v -> v.intakePower != 0));
        assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 30).transfer1Power, s.diagnostic());
        assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 30).transfer2Power, s.diagnostic());
        assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 70).transfer1Power, s.diagnostic());
        assertEquals(0, h.intake.getPower()); assertEquals(0, h.flywheel.getPower());
        assertEquals(0, h.transfer1.getPower()); assertEquals(.5, h.gate.getPosition());
    }

    @Test void benchLoadsHoldsFullFeedsAndReloadsThroughActualMotorOutputs() throws Exception {
        BioBuzzSimHardware h = new BioBuzzSimHardware().withIntake().withTransfer().withTurret();
        TeleOpScenario s = new TeleOpScenario("bench-transfer-cycle", "IntakeShooterTestOpMode",
                h, .02, 160, 650, (p,t,a,b) -> {
            boolean active = p == OpModePhase.ACTIVE;
            a.right_bumper = active && ((t >= 10 && t < 70) || (t >= 110 && t < 130) || (t >= 210 && t < 230) || t >= 600);
            a.left_bumper = active && t >= 80 && t < 130;
            h.distance.setSimulatedInputState(active && t >= 140 && t < 270);
            a.right_trigger = active && t >= 340 && t < 570 ? 1 : 0;
            a.left_trigger = active && t >= 420 && t < 570 ? 1 : 0;
        });
        s.run();
        assertFalse(s.result.endedBeforeStop(), s.diagnostic());
        assertTrue(s.result.warnings().isEmpty(), s.diagnostic());
        assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 30).intakePower, s.diagnostic());
        assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 30).transfer1Power, s.diagnostic());
        assertEquals(-1, s.trace.at(OpModePhase.ACTIVE, 90).intakePower, s.diagnostic());
        assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 90).transfer1Power, s.diagnostic());
        assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 120).intakePower, s.diagnostic());
        assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 120).transfer1Power, s.diagnostic());
        assertEquals(-1, s.trace.at(OpModePhase.ACTIVE, 175).intakePower, s.diagnostic());
        assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 175).transfer1Power, s.diagnostic());
        assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 175).transfer2Power, s.diagnostic());
        assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 200).transfer1Power, s.diagnostic());
        assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 220).intakePower, s.diagnostic()); // RB while HIGH/full.
        for (int tick : new int[]{200, 250}) {
            assertEquals(-1, s.trace.at(OpModePhase.ACTIVE, tick).intakePower, s.diagnostic());
            assertEquals(0, s.trace.at(OpModePhase.ACTIVE, tick).transfer1Power, s.diagnostic());
        }
        assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 275).intakePower, s.diagnostic());
        assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 275).transfer1Power, s.diagnostic()); // LOW reloads without feed.
        assertTrue(s.trace.samples.stream().anyMatch(v -> v.phase == OpModePhase.ACTIVE
                && v.tick >= 420 && v.tick < 570 && v.transfer1Power == 1 && v.transfer2Power == 1), s.diagnostic());
        double openedAt = Double.NaN;
        for (TeleOpTrace.Sample v : s.trace.samples) if (v.phase == OpModePhase.ACTIVE) {
            if (Math.abs(v.gatePosition - .2) < 1e-9) {
                if (Double.isNaN(openedAt)) openedAt = v.seconds;
            } else openedAt = Double.NaN;
            if (v.transfer2Power > 0) {
                assertFalse(Double.isNaN(openedAt), s.diagnostic());
                double elapsed = v.seconds - openedAt;
                assertTrue(elapsed >= .10 - 1e-9, s.diagnostic());
                // Stage 2 launches first; stage 1 starts at the midpoint of the 0.50 s open window.
                if (elapsed < .25 - 1e-9) assertEquals(0, v.transfer1Power, s.diagnostic());
                else if (elapsed > .25 + .02) assertEquals(1, v.transfer1Power, s.diagnostic());
            }
            if (v.tick < 420 || v.tick >= 570) assertEquals(0, v.transfer2Power, s.diagnostic());
        }
        assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 580).transfer1Power, s.diagnostic());
        assertEquals(.5, s.trace.at(OpModePhase.ACTIVE, 580).gatePosition, s.diagnostic());
        assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 620).intakePower, s.diagnostic());
        assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 620).transfer1Power, s.diagnostic());
        assertEquals(0, h.intake.getPower()); assertEquals(0, h.flywheel.getPower());
        assertEquals(0, h.transfer1.getPower()); assertEquals(0, h.transfer2.getPower());
        assertEquals(.5, h.gate.getPosition());
    }

    @Test void benchContinuousFeedsImmediatelyUntilTriggerReleaseOrStop() throws Exception {
        BioBuzzSimHardware h = new BioBuzzSimHardware().withIntake().withTransfer().withTurret();
        TeleOpScenario s = new TeleOpScenario("bench-continuous", "IntakeShooterTestOpMode",
                h, .02, 160, 180, (p,t,a,b) -> {
            boolean active = p == OpModePhase.ACTIVE;
            b.right_bumper = active && t >= 10 && t < 16;
            a.right_trigger = active && t >= 20 && (t < 120 || t >= 130) ? 1 : 0;
            a.left_trigger = active && t >= 20 && (t < 100 || t >= 110) ? 1 : 0;
            a.b = active && t >= 140 && t < 145;
        });
        s.run();
        assertFalse(s.result.endedBeforeStop(), s.diagnostic());
        assertTrue(s.result.warnings().isEmpty(), s.diagnostic());
        // At tick 20 the flywheel has only just been commanded: feeding must not wait for speed or gate delay.
        for (int tick : new int[]{20, 21, 70, 99, 110, 119, 130, 139}) {
            TeleOpTrace.Sample v = s.trace.at(OpModePhase.ACTIVE, tick);
            assertEquals(.2, v.gatePosition, 1e-9, s.diagnostic());
            assertEquals(1, v.transfer1Power, s.diagnostic());
            assertEquals(1, v.transfer2Power, s.diagnostic());
        }
        for (int tick : new int[]{100, 105, 120, 125, 140, 145, 170}) {
            TeleOpTrace.Sample v = s.trace.at(OpModePhase.ACTIVE, tick);
            assertEquals(.5, v.gatePosition, 1e-9, s.diagnostic());
            // Releasing LT resumes automatic loading; B/unarmed or held LT still pauses it.
            assertEquals(tick == 100 || tick == 105 ? 1 : 0, v.transfer1Power, s.diagnostic());
            assertEquals(0, v.transfer2Power, s.diagnostic());
        }
        assertEquals(0, h.transfer1.getPower());
        assertEquals(0, h.transfer2.getPower());
        assertEquals(.5, h.gate.getPosition());
    }
}
