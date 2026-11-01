package org.firstinspires.ftc.teamcode.sim;

import org.junit.jupiter.api.Test;
import org.horizon36596.simloop.opmode.OpModePhase;
import static org.junit.jupiter.api.Assertions.*;

class SensorsSimTest {
    @Test void realDiagnosticsInitializeAndKeepMotorsOffAtStop() throws Exception {
        for (String opMode : new String[]{"PinpointTestOpMode", "LimelightTagTestOpMode", "GoalStateTestOpMode",
                "AutoAimTestOpMode", "PedroDriveTestOpMode", "ShotCalibrationOpMode"}) {
            TeleOpScenario s = new TeleOpScenario("diagnostic-" + opMode, opMode,
                    BioBuzzSimHardware.forOpMode(opMode), .02, 160, 100, (p,t,a,b) -> {});
            s.run();
            assertFalse(s.result.endedBeforeStop(), s.diagnostic());
            assertTrue(s.result.warnings().isEmpty(), s.diagnostic());
            if (s.hardware.wheels[0] != null) for (org.horizon36596.simloop.fakehardware.FakeMotor wheel : s.hardware.wheels) assertEquals(0, wheel.getPower());
        }
    }

    @Test void rawUpstreamTagConventionIsExplicitlyNotRobotValidated() throws Exception {
        BioBuzzSimHardware h = BioBuzzSimHardware.fullRobot().withCamera(BioBuzzSimHardware.raisedAudienceTags());
        TeleOpScenario s = new TeleOpScenario("raw-tag-convention", "RedMecanumTeleOp", h,
                .02, 160, 80, (p,t,a,b) -> a.right_trigger = p == OpModePhase.ACTIVE ? 1 : 0);
        s.model.drive.setPose(-12.75, -60, Math.PI);
        h.pinpoint.attachChassisAt(-12.75*25.4, -60*25.4, Math.PI);
        s.run();
        assertTrue(h.camera.getLatestResult().isValid(), "Geometric detections exist");
        assertFalse(s.trace.at(OpModePhase.ACTIVE, 60).telemetry.contains("Field localization : LOCALIZED"),
                "If raw convention now works, revisit adapter and physical validation before updating this test");
        assertEquals(.5, s.trace.at(OpModePhase.ACTIVE, 60).gatePosition);
    }

    @Test void normalizedTagConventionAcquiresAndSurvivesCameraLoss() throws Exception {
        for (String alliance : new String[]{"Red", "Blue"}) {
            BioBuzzSimHardware h = BioBuzzSimHardware.fullRobot().withCamera(BioBuzzSimHardware.raisedAudienceTags(), true);
            TeleOpScenario s = new TeleOpScenario(alliance + "-vision", alliance + "MecanumTeleOp",
                    h, .02, 160, 200, (p,t,a,b) -> {
                // Real acquisition, then camera loss: local odometry must retain the reference.
                if (p == OpModePhase.ACTIVE && t == 80) h.camera.stop();
                a.right_trigger = p == OpModePhase.ACTIVE ? 1 : 0;
            });
            double x = alliance.equals("Red") ? -12.75 : 12.75;
            s.model.drive.setPose(x, -60, Math.PI);
            h.pinpoint.attachChassisAt(x*25.4, -60*25.4, Math.PI);
            s.run();
            assertTrue(s.trace.samples.stream().anyMatch(v -> v.telemetry.contains("Field localization : LOCALIZED")), s.diagnostic());
            assertTrue(s.trace.at(OpModePhase.ACTIVE, 180).telemetry.contains("Field localization : LOCALIZED"), s.diagnostic());
            assertEquals(.5, s.trace.at(OpModePhase.ACTIVE, 180).gatePosition, s.diagnostic());
            CompetitionSimTest.assertStopped(h);
        }
    }
}
