package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.horizon36596.zenith.ivy.ZenithRobot;

/**
 * Runs the red-authored red-far-volley-park route point-mirrored for blue: the file's alliance is RED and this
 * robot runs BLUE, so Ivy applies (x, y, h) to (-x, -y, h + pi) once and aim(frame=AUTHORED) swaps AUDIENCE/FAR.
 * Blue Audience start: left side on the audience wall, front at the centre line, facing red. Wait 10 s, volley the
 * 4 preload at the blue audience cell, then park with one corner in the blue loading zone.
 */
@Autonomous(name = "DRAFT Auto: Blue Audience Delay + Volley + Park", group = "Competition", preselectTeleOp = "Mecanum TeleOp - Blue")
public final class BlueAudienceVolleyParkAutonomous extends BioBuzzAutoFromFile {
    @Override protected String autoName() { return "red-far-volley-park"; }
    @Override protected ZenithRobot createRobot() { return new BioBuzzRobot(TurretSubsystem.Alliance.BLUE); }
}
