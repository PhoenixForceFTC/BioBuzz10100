package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.horizon36596.zenith.ivy.ZenithRobot;

/**
 * Runs the red-authored red-audience-volley-park route point-mirrored for blue: the file's alliance is RED and this
 * robot runs BLUE, so Ivy applies (x, y, h) to (-x, -y, h + pi) once and aim(frame=AUTHORED) swaps AUDIENCE/FAR.
 * Blue Far start: left side on the far wall, back edge exactly on the centre line, facing blue. Wait 10 s, volley the 4 preload at the blue far cell, then park with one corner (about 3 x 3 in) in the blue loading zone.
 */
@Autonomous(name = "DRAFT Auto: Blue Far Delay + Volley + Park", group = "Competition", preselectTeleOp = "Mecanum TeleOp - Blue")
public final class BlueFarVolleyParkAutonomous extends BioBuzzAutoFromFile {
    @Override protected String autoName() { return "red-audience-volley-park"; }
    @Override protected ZenithRobot createRobot() { return new BioBuzzRobot(TurretSubsystem.Alliance.BLUE); }
}
