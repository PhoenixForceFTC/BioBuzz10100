package org.firstinspires.ftc.teamcode.sim;

import org.junit.jupiter.api.Test;
import org.horizon36596.simloop.opmode.OpModePhase;
import static org.junit.jupiter.api.Assertions.*;

class CompetitionSimTest {
    @Test void syntheticMeasuredProfileFeedsThenOdometryFaultBlocksFurtherShots() throws Exception {
        for (String alliance : new String[]{"Red", "Blue"}) {
            BioBuzzSimHardware h = BioBuzzSimHardware.fullRobot().withCamera(BioBuzzSimHardware.raisedAudienceTags(), true);
            TeleOpScenario s = new TeleOpScenario(alliance + "-SYNTHETIC-calibration", "sim.Synthetic" + alliance + "TeleOp",
                    h, .02, 160, 400, (p,t,a,b) -> {
                a.right_trigger = p == OpModePhase.ACTIVE ? 1 : 0;
                if (p == OpModePhase.ACTIVE && t == 180)
                    h.pinpoint.forceFault(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus.NOT_READY);
            });
            double x = alliance.equals("Red") ? -12.75 : 12.75;
            s.model.drive.setPose(x, -60, Math.PI);
            h.pinpoint.attachChassisAt(x*25.4, -60*25.4, Math.PI);
            s.run();
            assertTrue(s.trace.samples.stream().anyMatch(v -> v.phase == OpModePhase.ACTIVE && v.tick < 180
                    && v.transfer2Power > 0 && Math.abs(v.gatePosition - .2) < 1e-9), s.diagnostic());
            for (TeleOpTrace.Sample v : s.trace.samples) if (v.phase == OpModePhase.ACTIVE && v.tick > 280) {
                assertEquals(0, v.transfer2Power, s.diagnostic()); assertEquals(.5, v.gatePosition, s.diagnostic());
            }
            assertStopped(h);
        }
    }

    @Test void missingRequiredTransferHardwareCleansUpInitializedOutputs() throws Exception {
        BioBuzzSimHardware h = BioBuzzSimHardware.fullRobot();
        h.map.remove(null, "transfer_motor_2", h.transfer2);
        TeleOpScenario s = new TeleOpScenario("partial-init", "RedMecanumTeleOp", h,
                .02, 10, 10, (p,t,a,b) -> {});
        assertThrows(RuntimeException.class, s::run);
        assertStopped(h);
    }

    @Test void bothAlliancesManualDriveQueueFeedAndInhibit() throws Exception {
        for (String alliance : new String[]{"Red", "Blue"}) for (int feedPad : new int[]{1, 2}) {
            BioBuzzSimHardware h = BioBuzzSimHardware.fullRobot();
            TeleOpScenario s = new TeleOpScenario(alliance + "-manual-gp" + feedPad, alliance + "MecanumTeleOp",
                    h, .02, 160, 350, (p,t,a,b) -> {
                boolean active = p == OpModePhase.ACTIVE;
                a.left_stick_y = active && t < 40 ? -.5f : 0;
                a.left_bumper = active && ((t >= 40 && t < 50) || (t >= 60 && t < 70));
                a.right_bumper = active && ((t >= 60 && t < 70) || (t >= 145 && t < 155));
                h.distance.setSimulatedInputState(active && t < 140);
                boolean manualFeed = active && t >= 160 && t < 290;
                a.left_trigger = feedPad == 1 && manualFeed ? 1 : 0;
                b.right_trigger = feedPad == 2 && manualFeed ? 1 : 0;
                b.left_trigger = active && t >= 260 ? 1 : 0;
                b.right_stick_x = active && t < 40 ? .25f : 0;
                b.dpad_up = active && t < 40;
                a.right_stick_button = active && t == 50;
            });
            s.run();
            assertFalse(s.result.endedBeforeStop(), s.diagnostic());
            assertTrue(s.result.warnings().isEmpty(), s.diagnostic());
            assertTrue(s.trace.at(OpModePhase.ACTIVE, 35).xIn > 0, s.diagnostic());
            assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 35).transfer1Power, s.diagnostic());
            assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 35).transfer2Power, s.diagnostic());
            assertEquals(-1, s.trace.at(OpModePhase.ACTIVE, 35).intakePower, s.diagnostic());
            assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 55).transfer1Power, s.diagnostic());
            assertEquals(-1, s.trace.at(OpModePhase.ACTIVE, 45).intakePower, s.diagnostic());
            assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 65).intakePower, s.diagnostic());
            assertEquals(-1, s.trace.at(OpModePhase.ACTIVE, 130).intakePower, s.diagnostic());
            assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 130).transfer1Power, s.diagnostic());
            assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 150).intakePower, s.diagnostic());
            assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 150).transfer1Power, s.diagnostic());
            assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 200).transfer1Power, s.diagnostic());
            assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 200).transfer2Power, s.diagnostic());
            assertTrue(s.trace.samples.stream().anyMatch(v -> v.transfer2Power > 0 && Math.abs(v.gatePosition - .2) < 1e-9), s.diagnostic());
            assertEquals(.5, s.trace.at(OpModePhase.ACTIVE, 270).gatePosition, s.diagnostic());
            assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 270).transfer1Power, s.diagnostic());
            assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 270).transfer2Power, s.diagnostic());
            assertEquals(165.0/355, s.trace.at(OpModePhase.ACTIVE, 55).depollinator, 1e-9);
            assertStopped(h);
        }
    }

    @Test void unknownFieldReferenceInhibitsAutoFireWithoutBlockingDrive() throws Exception {
        for (String alliance : new String[]{"Red", "Blue"}) {
            TeleOpScenario s = new TeleOpScenario(alliance + "-unlocalized", alliance + "MecanumTeleOp",
                    BioBuzzSimHardware.fullRobot(), .01, 2, 400, (p,t,a,b) -> {
                a.right_trigger = p == OpModePhase.ACTIVE ? 1 : 0;
                a.left_stick_y = p == OpModePhase.ACTIVE ? -.2f : 0;
            });
            s.run();
            assertTrue(s.trace.at(OpModePhase.ACTIVE, 100).xIn > 0);
            for (TeleOpTrace.Sample v : s.trace.samples) {
                assertEquals(.5, v.gatePosition, s.diagnostic());
                assertEquals(0, v.transfer2Power, s.diagnostic());
            }
            assertStopped(s.hardware);
        }
    }

    @Test void bothAlliancesThreeMinuteEnduranceAndStopBeforeStart() throws Exception {
        for (String alliance : new String[]{"Red", "Blue"}) for (int ticks : new int[]{0, 9000}) {
            TeleOpScenario s = new TeleOpScenario(alliance + "-duration-" + ticks, alliance + "MecanumTeleOp",
                    BioBuzzSimHardware.fullRobot(), .02, 160, ticks, (p,t,a,b) -> {});
            s.run(); assertStopped(s.hardware);
            assertTrue(s.result.warnings().isEmpty(), s.diagnostic());
            if (ticks > 0) assertTrue(s.trace.at(OpModePhase.ACTIVE, 8999).flywheelVelocity > 1000);
        }
    }

    static void assertStopped(BioBuzzSimHardware h) {
        for (org.horizon36596.simloop.fakehardware.FakeMotor wheel : h.wheels) assertEquals(0, wheel.getPower(), 0.0);
        assertEquals(0, h.intake.getPower(), 0.0); assertEquals(0, h.transfer1.getPower(), 0.0);
        assertEquals(0, h.transfer2.getPower(), 0.0); assertEquals(0, h.flywheel.getPower(), 0.0);
        assertEquals(.5, h.gate.getPosition());
    }
}
