package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.horizon36596.zenith.ivy.AutoFromFile;
import org.horizon36596.zenith.ivy.ZenithRobot;

/** Diagnostic collection experiment, deliberately separate from the match routines. */
@Autonomous(name = "Test: Pollen Navigation (Pedro + Ivy)", group = "Test")
public final class PollenAuto extends AutoFromFile {
    @Override protected String autoName() { return "pollen-navigation-test"; }
    @Override protected ZenithRobot createRobot() { return BioBuzzRobot.pollenTest(); }
}
