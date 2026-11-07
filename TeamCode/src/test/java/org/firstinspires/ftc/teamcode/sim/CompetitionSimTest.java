package org.firstinspires.ftc.teamcode.sim;

import org.junit.jupiter.api.Test;
import org.horizon36596.simloop.opmode.OpModePhase;
import static org.junit.jupiter.api.Assertions.*;

class CompetitionSimTest {
    @Test void bothAlliancesLoadWhileAimingPauseForFireAndReportIndependentLights() throws Exception {
        for (String alliance : new String[]{"Red", "Blue"}) {
            BioBuzzSimHardware h = BioBuzzSimHardware.fullRobot().withLights();
            TeleOpScenario s = new TeleOpScenario(alliance + "-aim-collection-lights", alliance + "MecanumTeleOp",
                    h, .02, 160, 170, (p,t,a,b) -> {
                boolean active = p == OpModePhase.ACTIVE;
                a.right_trigger = active ? 1 : 0;
                a.left_trigger = active && t >= 40 && t < 65 ? 1 : 0;
                b.right_trigger = active && t >= 80 && t < 120 ? 1 : 0;
                b.left_trigger = active && t >= 105 && t < 120 ? 1 : 0;
                if (active && t == 130) h.intake.setEncoderMeasuringAnotherMechanism(0, 0);
            });
            s.observer = v -> {
                if (v.phase == OpModePhase.ACTIVE && v.tick == 20) {
                    assertEquals(1700, h.ledIntake.getPulseWidth(), 1);
                    assertEquals(1300, h.ledShooter.getPulseWidth(), 1);
                    assertEquals(1200, h.ledStatus.getPulseWidth(), 1); // Homed, no field reference.
                }
                if (v.phase == OpModePhase.ACTIVE && v.tick == 110)
                    assertEquals(1100, h.ledShooter.getPulseWidth(), 1);
            };
            s.run();
            for (int tick : new int[]{20, 70, 125}) {
                assertEquals(1, s.trace.at(OpModePhase.ACTIVE, tick).intakePower, s.diagnostic());
                assertEquals(1, s.trace.at(OpModePhase.ACTIVE, tick).transfer1Power, s.diagnostic());
                assertEquals(0, s.trace.at(OpModePhase.ACTIVE, tick).transfer2Power, s.diagnostic());
                assertEquals(.5, s.trace.at(OpModePhase.ACTIVE, tick).gatePosition, s.diagnostic());
            }
            for (int tick : new int[]{50, 85, 100, 110})
                assertEquals(0, s.trace.at(OpModePhase.ACTIVE, tick).intakePower, s.diagnostic());
            assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 50).transfer1Power, s.diagnostic());
            assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 100).transfer2Power, s.diagnostic());
            assertTrue(s.trace.at(OpModePhase.ACTIVE, 100).telemetry.contains("Shot : MANUAL FEED"));
            assertEquals(-1, s.trace.at(OpModePhase.ACTIVE, 142).intakePower, s.diagnostic());
            assertEquals(600, h.ledIntake.getPulseWidth(), 1);
            assertEquals(600, h.ledShooter.getPulseWidth(), 1);
            assertEquals(600, h.ledStatus.getPulseWidth(), 1);
            assertStopped(h);
        }
    }

    @Test void bothAlliancesApplyAndClearAutoTrimsWithoutChangingManualDefaults() throws Exception {
        for (String alliance : new String[]{"Red", "Blue"}) {
            BioBuzzSimHardware h = BioBuzzSimHardware.fullRobot().withCamera(BioBuzzSimHardware.raisedAudienceTags(), true);
            TeleOpScenario s = new TeleOpScenario(alliance + "-SYNTHETIC-trims", "sim.Synthetic" + alliance + "TeleOp",
                    h, .02, 160, 190, (p,t,a,b) -> {
                boolean active = p == OpModePhase.ACTIVE;
                a.right_trigger = active && t < 160 ? 1 : 0;
                b.dpad_up = b.y = active && t >= 50 && t < 75;
                b.right_stick_x = active && t >= 50 && t < 60 ? .5f : 0;
                b.dpad_right = active && t >= 110 && t < 135;
            });
            double x = (alliance.equals("Red") ? -12.75 : 12.75) + 8;
            s.model.drive.setPose(x, BioBuzzSimHardware.RAISED_TAG_VIEW_Y_INCHES, Math.PI);
            h.pinpoint.attachChassisAt(x*25.4, BioBuzzSimHardware.RAISED_TAG_VIEW_Y_INCHES*25.4, Math.PI);
            s.run();
            TeleOpTrace.Sample before = s.trace.at(OpModePhase.ACTIVE, 40), trim = s.trace.at(OpModePhase.ACTIVE, 95);
            assertEquals(org.firstinspires.ftc.teamcode.TurretSubsystem.pitchDegreesToServoPos(5.5), trim.hood, 1e-9);
            assertEquals(2850.0 / 60 * 28, trim.flywheelVelocity, 50.0 / 60 * 28, s.diagnostic());
            assertTrue(trim.telemetry.contains("-1.5 / 0.5 / 50"), s.diagnostic());
            assertEquals(before.hood, s.trace.at(OpModePhase.ACTIVE, 145).hood, 1e-9);
            assertTrue(s.trace.at(OpModePhase.ACTIVE, 145).telemetry.contains("0.0 / 0.0 / 0"));
            assertTrue(java.util.regex.Pattern.compile("Flywheel \\(RPM\\) : -?\\d+ / 3500 \\|")
                    .matcher(s.trace.at(OpModePhase.ACTIVE, 185).telemetry).find(), s.diagnostic());
            for (TeleOpTrace.Sample v : s.trace.samples) assertEquals(.5, v.gatePosition, s.diagnostic());
            assertStopped(h);
        }
    }
    @Test void bothAlliancesFieldRehomeInInitAndActiveUsesPressEdgesWithoutCamera() throws Exception {
        for (String alliance : new String[]{"Red", "Blue"}) for (boolean inInit : new boolean[]{false, true}) {
            BioBuzzSimHardware h = BioBuzzSimHardware.fullRobot();
            TeleOpScenario s = new TeleOpScenario(alliance + "-field-rehome-" + inInit,
                    alliance + "MecanumTeleOp", h, .02, 160, 130, (p,t,a,b) -> {
                b.dpad_left = p == OpModePhase.INIT ? inInit && t >= 150
                        : p == OpModePhase.ACTIVE && ((t >= (inInit ? 0 : 20) && t < 60)
                        || (t >= 80 && t < 110));
                a.left_stick_y = p == OpModePhase.ACTIVE ? -.25f : 0;
                // Rehoming remains available while the driver requests automatic aiming.
                a.right_trigger = p == OpModePhase.ACTIVE ? 1 : 0;
            });
            java.util.Map<Integer, double[]> poses = new java.util.HashMap<>();
            s.observer = v -> {
                if (v.phase == OpModePhase.ACTIVE) {
                    org.firstinspires.ftc.robotcore.external.navigation.Pose2D p = h.pinpoint.getPosition();
                    poses.put(v.tick, new double[]{
                            p.getX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.METER),
                            p.getY(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.METER),
                            p.getHeading(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS)});
                }
            };
            s.run();
            double sign = alliance.equals("Blue") ? 1 : -1;
            // Independent placement calculation: 144-in field, rear 262.481 mm, side 158.35 mm.
            double expectedX = sign * (72 * .0254 - .262481);
            double expectedY = -sign * (72 * .0254 - .15835);
            double expectedHeading = alliance.equals("Blue") ? Math.PI : 0;
            int firstPress = inInit ? 0 : 20;
            if (!inInit) assertFalse(s.trace.at(OpModePhase.ACTIVE, 10).telemetry.contains("Field localization : LOCALIZED"));
            for (int tick : new int[]{firstPress, 80}) {
                assertEquals(expectedX, poses.get(tick)[0], inInit && tick == 0 ? .002 : 1e-6, s.diagnostic());
                assertEquals(expectedY, poses.get(tick)[1], 1e-6, s.diagnostic());
                assertEquals(0, org.firstinspires.ftc.teamcode.RobotMath.wrap(poses.get(tick)[2] - expectedHeading), 1e-6);
                assertTrue(s.trace.at(OpModePhase.ACTIVE, tick).telemetry.contains("| EXPLICIT FIELD"), s.diagnostic());
                // A reset deliberately invalidates the first acceleration sample; the next loop must localize.
                if (!(inInit && tick == 0)) {
                    assertFalse(s.trace.at(OpModePhase.ACTIVE, tick).telemetry.contains("Field localization : LOCALIZED"), s.diagnostic());
                }
                assertTrue(s.trace.at(OpModePhase.ACTIVE, tick + 1).telemetry.contains("Field localization : LOCALIZED"), s.diagnostic());
            }
            // Holding through START or for many ACTIVE ticks must allow odometry movement, not pin the pose.
            assertTrue(sign * (poses.get(50)[0] - poses.get(firstPress)[0]) < -.01, s.diagnostic());
            assertTrue(sign * (poses.get(105)[0] - poses.get(80)[0]) < -.01, s.diagnostic());
            assertTrue(s.trace.at(OpModePhase.ACTIVE, 50).powers[0] != 0, s.diagnostic());
            assertEquals(.5, s.trace.at(OpModePhase.ACTIVE, 105).gatePosition, s.diagnostic());
            assertStopped(h);
        }
    }

    @Test void bothAlliancesToggleContinuousAndStaggeredFeedsOnEitherManualControl() throws Exception {
        for (String alliance : new String[]{"Red", "Blue"}) for (int feedPad : new int[]{1, 2}) {
            BioBuzzSimHardware h = BioBuzzSimHardware.fullRobot();
            TeleOpScenario s = new TeleOpScenario(alliance + "-feed-toggle-gp" + feedPad,
                    alliance + "MecanumTeleOp", h, .02, 160, 300, (p,t,a,b) -> {
                boolean active = p == OpModePhase.ACTIVE;
                boolean feed = active && t >= 10 && (t < 246 || t >= 260);
                a.a = feed && feedPad == 1;
                a.left_trigger = active && t < 10 ? 1 : 0; // LT alone no longer requests manual feeding.
                b.right_trigger = feed && feedPad == 2 ? 1 : 0;
                b.b = active && ((t >= 60 && t < 130) || (t >= 150 && t < 200));
                b.left_trigger = active && t >= 230 && t < 246 ? 1 : 0;
                a.left_stick_y = active ? -.2f : 0;
            });
            s.run();
            assertEquals(.5, s.trace.at(OpModePhase.ACTIVE, 5).gatePosition, s.diagnostic());
            assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 5).transfer2Power, s.diagnostic());
            assertTrue(s.trace.at(OpModePhase.ACTIVE, 55).telemetry.contains("CONTINUOUS"), s.diagnostic());
            assertTrue(s.trace.at(OpModePhase.ACTIVE, 120).telemetry.contains("STAGGERED"), s.diagnostic());
            assertTrue(s.trace.at(OpModePhase.ACTIVE, 205).telemetry.contains("CONTINUOUS"), s.diagnostic());
            for (int tick : new int[]{10, 35, 55, 85, 170, 220, 280}) {
                assertEquals(1, s.trace.at(OpModePhase.ACTIVE, tick).transfer1Power, s.diagnostic());
                assertEquals(1, s.trace.at(OpModePhase.ACTIVE, tick).transfer2Power, s.diagnostic());
            }
            // Fresh gate delay after changing modes, then both stages start together.
            assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 62).transfer2Power, s.diagnostic());
            assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 62).transfer1Power, s.diagnostic());
            assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 68).transfer1Power, s.diagnostic());
            assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 68).transfer2Power, s.diagnostic());
            // Timed recovery keeps both stages running with the gate closed; inhibit stops them.
            for (int tick : new int[]{85, 235}) {
                assertEquals(.5, s.trace.at(OpModePhase.ACTIVE, tick).gatePosition, s.diagnostic());
                assertEquals(tick == 85 ? 1 : 0, s.trace.at(OpModePhase.ACTIVE, tick).transfer1Power, s.diagnostic());
                assertEquals(tick == 85 ? 1 : 0, s.trace.at(OpModePhase.ACTIVE, tick).transfer2Power, s.diagnostic());
            }
            assertEquals(.5, s.trace.at(OpModePhase.ACTIVE, 250).gatePosition, s.diagnostic());
            assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 250).transfer2Power, s.diagnostic());
            assertTrue(s.trace.at(OpModePhase.ACTIVE, 220).xIn
                    > s.trace.at(OpModePhase.ACTIVE, 60).xIn, s.diagnostic());
            assertStopped(h);
        }
    }

    @Test void syntheticMeasuredProfileFeedsThenOdometryFaultBlocksFurtherShots() throws Exception {
        for (String alliance : new String[]{"Red", "Blue"}) for (boolean staggered : new boolean[]{false, true}) {
            BioBuzzSimHardware h = BioBuzzSimHardware.fullRobot().withCamera(BioBuzzSimHardware.raisedAudienceTags(), true);
            TeleOpScenario s = new TeleOpScenario(alliance + "-SYNTHETIC-calibration-" + staggered, "sim.Synthetic" + alliance + "TeleOp",
                    h, .02, 160, 480, (p,t,a,b) -> {
                boolean active = p == OpModePhase.ACTIVE;
                a.right_trigger = active && (t < 230 || t >= 260) ? 1 : 0;
                a.left_trigger = active && t >= 80 && (t < 140 || t >= 170) ? 1 : 0;
                b.b = active && staggered && t == 0;
                if (active && t == 320)
                    h.pinpoint.forceFault(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus.NOT_READY);
            });
            // Start off the hive centerline so preparation must move yaw as well as hood/RPM.
            double x = (alliance.equals("Red") ? -12.75 : 12.75) + 8;
            s.model.drive.setPose(x, BioBuzzSimHardware.RAISED_TAG_VIEW_Y_INCHES, Math.PI);
            h.pinpoint.attachChassisAt(x*25.4, BioBuzzSimHardware.RAISED_TAG_VIEW_Y_INCHES*25.4, Math.PI);
            s.run();
            // With valid synthetic calibration, RT prepares all three axes without opening the gate.
            TeleOpTrace.Sample prepared = s.trace.at(OpModePhase.ACTIVE, 75);
            assertEquals(org.firstinspires.ftc.teamcode.TurretSubsystem.pitchDegreesToServoPos(5), prepared.hood, 1e-9);
            assertEquals(2800.0 / 60 * 28, prepared.flywheelVelocity, 100.0 / 60 * 28, s.diagnostic());
            assertTrue(Math.abs(prepared.yaw) > 3, s.diagnostic());
            // Releasing either trigger stops automatic feeding; RT alone continues preparation.
            for (TeleOpTrace.Sample v : s.trace.samples) if (v.phase == OpModePhase.ACTIVE
                    && (v.tick < 80 || (v.tick >= 140 && v.tick < 170) || (v.tick >= 230 && v.tick < 260))) {
                assertEquals(0, v.transfer2Power, s.diagnostic());
                assertEquals(.5, v.gatePosition, s.diagnostic());
            }
            for (int start : new int[]{80, 170, 260}) {
                assertTrue(s.trace.samples.stream().anyMatch(v -> v.phase == OpModePhase.ACTIVE
                        && v.tick >= start && v.tick < start + 60
                        && v.transfer2Power > 0 && Math.abs(v.gatePosition - .2) < 1e-9), s.diagnostic());
            }
            for (TeleOpTrace.Sample v : s.trace.samples) if (v.phase == OpModePhase.ACTIVE && v.tick > 420) {
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
                a.left_bumper = active && ((t >= 40 && t < 50) || (t >= 60 && t < 70)
                        || (t >= 155 && t < 160) || (t >= 220 && t < 230));
                a.right_bumper = active && ((t >= 60 && t < 70) || (t >= 145 && t < 155)
                        || (t >= 160 && t < 175) || (t >= 200 && t < 210));
                b.right_bumper = active && ((t >= 90 && t < 100) || (t >= 155 && t < 160)
                        || t >= 260);
                b.left_bumper = active && ((t >= 105 && t < 115) || (t >= 140 && t < 145));
                h.distance.setSimulatedInputState(active && t < 140);
                boolean manualFeed = active && t >= 160 && t < 290;
                a.a = feedPad == 1 && manualFeed;
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
            // RB overrides confirmed full on GP2 as well as GP1, without running stage 2 or opening the gate.
            assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 95).intakePower, s.diagnostic());
            assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 95).transfer1Power, s.diagnostic());
            assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 95).transfer2Power, s.diagnostic());
            assertEquals(.5, s.trace.at(OpModePhase.ACTIVE, 95).gatePosition, s.diagnostic());
            assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 102).transfer1Power, s.diagnostic());
            // Reverse only powers intake, even with space; conflicting controls across pads stop collection.
            for (int tick : new int[]{110, 142, 157}) {
                assertEquals(tick == 157 ? 0 : -1, s.trace.at(OpModePhase.ACTIVE, tick).intakePower, s.diagnostic());
                assertEquals(0, s.trace.at(OpModePhase.ACTIVE, tick).transfer1Power, s.diagnostic());
                assertEquals(0, s.trace.at(OpModePhase.ACTIVE, tick).transfer2Power, s.diagnostic());
                assertEquals(.5, s.trace.at(OpModePhase.ACTIVE, tick).gatePosition, s.diagnostic());
            }
            assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 150).intakePower, s.diagnostic());
            assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 150).transfer1Power, s.diagnostic());
            assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 200).transfer1Power, s.diagnostic());
            assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 200).transfer2Power, s.diagnostic());
            assertEquals(-1, s.trace.at(OpModePhase.ACTIVE, 225).intakePower, s.diagnostic());
            assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 225).transfer1Power, s.diagnostic());
            assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 225).transfer2Power, s.diagnostic());
            assertTrue(s.trace.samples.stream().anyMatch(v -> v.transfer2Power > 0 && Math.abs(v.gatePosition - .2) < 1e-9), s.diagnostic());
            assertEquals(.5, s.trace.at(OpModePhase.ACTIVE, 270).gatePosition, s.diagnostic());
            assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 270).transfer1Power, s.diagnostic());
            assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 270).transfer2Power, s.diagnostic());
            // Intake is independent, but forward override cannot defeat the priority transfer inhibit.
            assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 270).intakePower, s.diagnostic());
            double openedAt = Double.NaN;
            for (TeleOpTrace.Sample v : s.trace.samples) if (v.phase == OpModePhase.ACTIVE) {
                if (Math.abs(v.gatePosition - .2) < 1e-9) {
                    if (Double.isNaN(openedAt)) openedAt = v.seconds;
                    // Default continuous mode starts both transfers on the opening command.
                    assertEquals(1, v.transfer1Power, s.diagnostic());
                    assertEquals(1, v.transfer2Power, s.diagnostic());
                } else openedAt = Double.NaN;
                if (v.transfer2Power > 0) {
                    assertFalse(Double.isNaN(openedAt), s.diagnostic());
                }
                if (v.tick >= 160 && v.tick < 170) assertEquals(1, v.transfer1Power, s.diagnostic());
            }
            assertEquals(165.0/355, s.trace.at(OpModePhase.ACTIVE, 55).depollinator, 1e-9);
            assertStopped(h);
        }
    }

    @Test void unknownFieldReferenceInhibitsAutoFireWithoutBlockingDrive() throws Exception {
        for (String alliance : new String[]{"Red", "Blue"}) {
            TeleOpScenario s = new TeleOpScenario(alliance + "-unlocalized", alliance + "MecanumTeleOp",
                    BioBuzzSimHardware.fullRobot(), .01, 2, 400, (p,t,a,b) -> {
                a.right_trigger = p == OpModePhase.ACTIVE ? 1 : 0;
                a.left_trigger = p == OpModePhase.ACTIVE ? 1 : 0;
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

    /**
     * Full right stick spins freely without RT; while RT auto-aims, the measured chassis turn rate stays at the
     * turret-trackable cap in both directions. Synthetic drive response only: not the robot's loaded spin rate.
     */
    @Test void bothAlliancesCapChassisTurnRateOnlyWhileAutoAiming() throws Exception {
        double cap = org.firstinspires.ftc.teamcode.AutoAimSpinLimiter.MAX_CHASSIS_RATE_DEG_PER_SECOND;
        for (String alliance : new String[]{"Red", "Blue"}) {
            float turn = alliance.equals("Red") ? 1f : -1f;
            TeleOpScenario s = new TeleOpScenario(alliance + "-aim-spin-cap", alliance + "MecanumTeleOp",
                    BioBuzzSimHardware.fullRobot(), .02, 160, 300, (p,t,a,b) -> {
                boolean active = p == OpModePhase.ACTIVE;
                a.right_stick_x = active && (t >= 20 && t < 120 || t >= 160 && t < 280) ? turn : 0;
                a.right_trigger = active && t >= 160 ? 1 : 0;
            });
            s.run();
            double free = peakTurnRateDegPerSecond(s, 60, 120), capped = peakTurnRateDegPerSecond(s, 200, 280);
            String stats = String.format(java.util.Locale.US, "%s: free %.0f | aiming %.0f | cap %.0f deg/s ",
                    alliance, free, capped, cap);
            System.out.println("Aim turn cap " + stats);
            assertTrue(free > cap * 1.1, "synthetic full stick must exceed the cap " + stats + s.diagnostic());
            assertTrue(capped <= cap * 1.1, stats + s.diagnostic());
            assertTrue(capped >= cap * .7, "aiming must still turn usefully " + stats + s.diagnostic());
            assertStopped(s.hardware);
        }
    }

    /** Five-tick (0.1 s) average plant heading rate, so one integration step cannot dominate. */
    static double peakTurnRateDegPerSecond(TeleOpScenario s, int fromTick, int toTick) {
        double peak = 0;
        for (int t = fromTick; t + 5 <= toTick; t++) {
            double a = s.trace.at(OpModePhase.ACTIVE, t).headingRad, b = s.trace.at(OpModePhase.ACTIVE, t + 5).headingRad;
            peak = Math.max(peak, Math.abs(Math.toDegrees(org.firstinspires.ftc.teamcode.RobotMath.wrap(b - a))) / (5 * .02));
        }
        return peak;
    }

    static void assertStopped(BioBuzzSimHardware h) {
        for (org.horizon36596.simloop.fakehardware.FakeMotor wheel : h.wheels) assertEquals(0, wheel.getPower(), 0.0);
        assertEquals(0, h.intake.getPower(), 0.0); assertEquals(0, h.transfer1.getPower(), 0.0);
        assertEquals(0, h.transfer2.getPower(), 0.0); assertEquals(0, h.flywheel.getPower(), 0.0);
        assertEquals(.5, h.gate.getPosition());
    }
}
