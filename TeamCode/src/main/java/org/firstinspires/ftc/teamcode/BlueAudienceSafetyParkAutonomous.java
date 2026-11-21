package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.horizon36596.zenith.ivy.ZenithRobot;

/**
 * Runs the red-authored red-far-safety-park route point-mirrored for blue: the file's alliance is RED and this
 * robot runs BLUE, so Ivy applies (x, y, h) to (-x, -y, h + pi) once and aim(frame=AUTHORED) swaps AUDIENCE/FAR.
 * Blue Audience start: left side on the audience wall, front exactly on the centre line, facing red. No shooting: leave the start area and park with one corner (about 3 x 3 in) in the blue loading zone.
 */
@Autonomous(name = "DRAFT Auto: Blue Audience Safety (Leave + Park)", group = "Competition", preselectTeleOp = "Mecanum TeleOp - Blue")
public final class BlueAudienceSafetyParkAutonomous extends BioBuzzAutoFromFile {
    @Override protected String autoName() { return "red-far-safety-park"; }
    @Override protected ZenithRobot createRobot() { return new BioBuzzRobot(TurretSubsystem.Alliance.BLUE); }
}
