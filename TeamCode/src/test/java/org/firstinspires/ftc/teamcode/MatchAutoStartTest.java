package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.*;

import java.io.File;
import java.nio.file.Files;
import org.json.JSONObject;
import org.junit.Test;

/**
 * The red-authored match autos start the way the team places the robot: left side exactly on the field edge, and
 * one edge (front or back) exactly on the centre line without crossing it. This pins that to the measured chassis
 * in {@code autos/robot.json}, so a start edit that drifts off the wall or over the line fails here and not on the
 * field. The blue autos are the point-mirror of these files, so they inherit the same placement.
 */
public class MatchAutoStartTest {
    private static final double FIELD_EDGE_IN = 72.0, TOL_IN = 0.01;

    private static JSONObject json(String name) throws Exception {
        return new JSONObject(new String(Files.readAllBytes(new File("../autos/" + name).toPath()), "UTF-8"));
    }

    private void check(String routine, double expectedHeadingRad, boolean farWall, boolean frontOnLine) throws Exception {
        JSONObject footprint = json("robot.json").getJSONObject("footprint").getJSONObject("startIn");
        double length = footprint.getDouble("lengthIn"), width = footprint.getDouble("widthIn");
        JSONObject pose = json(routine + ".auto.json").getJSONObject("start").getJSONObject("pose");
        double x = pose.getDouble("xIn"), y = pose.getDouble("yIn"), h = pose.getDouble("headingRad");
        assertEquals(routine + " heading", expectedHeadingRad, h, 1e-3);
        // Heading 0 or pi: the length runs along X, the width along Y.
        double centreLineEdge = x + length / 2;           // red half is x < 0: this is the edge nearest the line
        assertTrue(routine + ": edge nearest the centre line is " + centreLineEdge + " in (must not be over it)", centreLineEdge <= 0.0);
        assertTrue(routine + ": edge is " + centreLineEdge + " in, not on the line", centreLineEdge >= -TOL_IN);
        double wallEdge = Math.abs(y) + width / 2;
        assertEquals(routine + ": left side on the field edge", FIELD_EDGE_IN, wallEdge, TOL_IN);
        assertEquals(routine + ": left side is on the " + (farWall ? "far" : "audience") + " wall", farWall, y > 0);
        // Facing +X the front is the edge on the line; facing -X it is the back (the team's description of each start).
        assertEquals(routine + ": " + (frontOnLine ? "front" : "back") + " on the line", frontOnLine, Math.cos(h) > 0);
    }

    @Test public void farWallStartsHaveTheFrontOnTheCentreLineAndTheLeftSideOnTheWall() throws Exception {
        check("red-far-volley-park", 0.0, true, true);
        check("red-far-safety-park", 0.0, true, true);
    }

    @Test public void audienceWallStartsHaveTheBackOnTheCentreLineAndTheLeftSideOnTheWall() throws Exception {
        check("red-audience-volley-park", Math.PI, false, false);
        check("red-audience-safety-park", Math.PI, false, false);
    }
}
