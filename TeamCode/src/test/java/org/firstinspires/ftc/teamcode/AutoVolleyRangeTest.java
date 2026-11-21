package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.*;

import java.io.File;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;
import org.junit.Test;

/**
 * Plan guard: every {@code volley} or {@code shoot} in a routine under {@code autos/} should be fired from inside the
 * measured shot table. The profile extrapolates beyond its first and last rows (clamped to mechanism limits), so a
 * shot outside {@link ShootingConfiguration#MEASURED_SHOT_TABLE_IN_RPM_HOOD_DEG} still gets a hood and RPM, but from an
 * unmeasured line.
 *
 * <p>The range is the planner's own: robot pose to the selected hive cell's opening, from the turret pivot
 * ({@code ShootingConfiguration.MOUNT}), walking each routine's steps in order. The robot pose is the end of the last
 * path (heading from its heading mode, so the 50 mm pivot offset is rotated correctly). The cell is the one most
 * recently latched by {@code aim}, {@code selectTarget} or {@code volley(side)}. Routines are checked as authored
 * (their own alliance). Routines titled DRAFT or EXAMPLE carry placeholder coordinates, so for those a violation is
 * printed, not failed: remove that word from the title when the route is measured and the check starts enforcing.
 */
public class AutoVolleyRangeTest {
    private static final double[][] TABLE = ShootingConfiguration.MEASURED_SHOT_TABLE_IN_RPM_HOOD_DEG;
    static final double MIN_IN = TABLE[0][0], MAX_IN = TABLE[TABLE.length - 1][0];

    private static final class Pose {
        double x, y, h;
        Pose(double x, double y, double h) { this.x = x; this.y = y; this.h = h; }
        Pose copy() { return new Pose(x, y, h); }
    }

    /** All shots in {@code auto} whose range is outside the measured table, as readable lines. */
    static List<String> violations(JSONObject auto, JSONObject waypoints) throws JSONException {
        List<String> found = new ArrayList<>();
        TurretSubsystem.Alliance alliance = TurretSubsystem.Alliance.valueOf(auto.getString("alliance"));
        JSONObject start = auto.getJSONObject("start").getJSONObject("pose");
        Pose pose = new Pose(start.getDouble("xIn"), start.getDouble("yIn"), start.optDouble("headingRad", 0));
        TurretSubsystem.HiveSide[] latched = {TurretSubsystem.HiveSide.AUDIENCE};
        walk(list(auto.getJSONArray("steps")), pose, latched, alliance, waypoints, found, auto.getString("name"));
        return found;
    }

    private static List<JSONObject> list(JSONArray array) throws JSONException {
        List<JSONObject> out = new ArrayList<>();
        for (int i = 0; i < array.length(); i++) out.add(array.getJSONObject(i));
        return out;
    }

    private static Pose walk(List<JSONObject> steps, Pose start, TurretSubsystem.HiveSide[] latched,
                             TurretSubsystem.Alliance alliance, JSONObject waypoints, List<String> found, String name)
            throws JSONException {
        Pose pose = start.copy();
        for (int i = 0; i < steps.size(); i++) {
            JSONObject step = steps.get(i);
            switch (step.getString("kind")) {
                case "path":
                    pose = pathEnd(step, pose, waypoints);
                    break;
                case "command": {
                    String command = step.getString("name");
                    JSONObject args = step.optJSONObject("args");
                    String side = args == null ? null : args.optString("side", null);
                    if (command.equals("aim") || command.equals("selectTarget")) latched[0] = side(side);
                    else if (command.equals("volley") || command.equals("shoot")) {
                        if (command.equals("volley") && side != null) latched[0] = side(side);
                        double range = rangeInches(pose, alliance, latched[0]);
                        if (range < MIN_IN || range > MAX_IN)
                            found.add(String.format(java.util.Locale.US,
                                    "%s [%s] %s at the %s cell: %.1f in, outside the measured %.0f-%.0f in",
                                    name, step.optString("id", "?"), command, latched[0], range, MIN_IN, MAX_IN));
                    }
                    break;
                }
                case "sequence":
                    pose = walk(list(step.getJSONArray("steps")), pose, latched, alliance, waypoints, found, name);
                    break;
                case "parallel": {
                    Pose after = pose;
                    for (JSONObject child : list(step.getJSONArray("steps"))) {
                        Pose end = walk(java.util.Collections.singletonList(child), pose, latched, alliance, waypoints, found, name);
                        if (child.getString("kind").equals("path") || child.getString("kind").equals("sequence")) after = end;
                    }
                    pose = after;
                    break;
                }
                case "branch": {
                    // Both arms from the same pose, each continuing into the steps that follow the branch.
                    List<JSONObject> rest = steps.subList(i + 1, steps.size());
                    for (String arm : new String[]{"then", "else"}) {
                        List<JSONObject> path = list(step.optJSONArray(arm) == null ? new JSONArray() : step.getJSONArray(arm));
                        path.addAll(rest);
                        walk(path, pose, latched.clone(), alliance, waypoints, found, name);
                    }
                    return pose; // the continuation was checked inside each arm
                }
                default:
                    break; // wait, transfer
            }
        }
        return pose;
    }

    private static TurretSubsystem.HiveSide side(String s) {
        return "FAR".equals(s) ? TurretSubsystem.HiveSide.FAR : TurretSubsystem.HiveSide.AUDIENCE;
    }

    private static Pose pathEnd(JSONObject step, Pose from, JSONObject waypoints) throws JSONException {
        JSONArray segments = step.getJSONArray("segments");
        double fx = from.x, fy = from.y, tx = fx, ty = fy;
        for (int i = 0; i < segments.length(); i++) {
            JSONObject to = segments.getJSONObject(i).getJSONObject("to");
            if (to.has("ref")) to = waypoints.getJSONObject("waypoints").getJSONObject(to.getString("ref"));
            fx = tx; fy = ty;
            tx = to.getDouble("xIn"); ty = to.getDouble("yIn");
        }
        return new Pose(tx, ty, headingAtEnd(step.getJSONObject("heading"), fx, fy, tx, ty, from.h));
    }

    private static double headingAtEnd(JSONObject heading, double fx, double fy, double tx, double ty, double previous)
            throws JSONException {
        switch (heading.getString("mode")) {
            case "constant": return heading.optDouble("headingRad", previous);
            case "linear": return heading.optDouble("toRad", previous);
            case "tangent": return Math.atan2(ty - fy, tx - fx);
            case "tangentReversed": return Math.atan2(ty - fy, tx - fx) + Math.PI;
            case "piecewise": {
                JSONArray ranges = heading.getJSONArray("ranges");
                return headingAtEnd(ranges.getJSONObject(ranges.length() - 1).getJSONObject("heading"),
                        fx, fy, tx, ty, previous);
            }
            default: return previous; // facePoint and others: keep the previous heading (the 50 mm pivot offset is small)
        }
    }

    /** The planner's launch range, in inches: turret pivot to the opening, minus the muzzle offset. */
    static double rangeInches(Pose pose, TurretSubsystem.Alliance alliance, TurretSubsystem.HiveSide side) {
        GoalTarget target = TeleOpFieldReference.target(alliance, side);
        ShooterMount mount = ShootingConfiguration.MOUNT;
        RobotMath.Vec pivot = new RobotMath.Vec(pose.x * .0254, pose.y * .0254).plus(mount.pivot.rotate(pose.h));
        double distance = new RobotMath.Vec(target.x, target.y).minus(pivot).norm();
        double range = Math.sqrt(distance * distance - mount.muzzle.y * mount.muzzle.y) - mount.muzzle.x;
        return range / .0254;
    }

    @Test public void routinesFireOnlyFromInsideTheMeasuredShotTable() throws Exception {
        File dir = new File("../autos");
        JSONObject waypoints = new JSONObject(read(new File(dir, "waypoints.json")));
        File[] files = dir.listFiles((d, n) -> n.endsWith(".auto.json"));
        assertNotNull(files);
        java.util.Arrays.sort(files);
        List<String> enforced = new ArrayList<>();
        int checked = 0;
        for (File file : files) {
            JSONObject auto = new JSONObject(read(file));
            List<String> found = violations(auto, waypoints);
            String title = auto.optString("title", "");
            boolean placeholder = title.startsWith("DRAFT") || title.startsWith("EXAMPLE");
            checked++;
            for (String line : found) {
                if (placeholder) System.out.println("[volley range, DRAFT not enforced] " + line);
                else enforced.add(line);
            }
        }
        assertTrue("no routines found", checked > 0);
        assertTrue("Shots fired outside the measured shot table (extrapolated, unmeasured):\n  "
                + String.join("\n  ", enforced), enforced.isEmpty());
    }

    @Test public void checkerFlagsAShotBeyondTheTableAndPassesOneInside() throws Exception {
        JSONObject empty = new JSONObject("{\"waypoints\":{}}");
        // RED start (-12, -63) facing +Y: the audience cell (-12.75, -15.1) is about 48 in away.
        String near = auto("[{\"id\":\"v\",\"kind\":\"command\",\"name\":\"volley\",\"args\":{\"side\":\"AUDIENCE\"}}]");
        assertTrue(violations(new JSONObject(near), empty).isEmpty());
        // The far cell is about 78 in away: outside the 12-60 in table.
        String far = auto("[{\"id\":\"v\",\"kind\":\"command\",\"name\":\"volley\",\"args\":{\"side\":\"FAR\"}}]");
        List<String> found = violations(new JSONObject(far), empty);
        assertEquals(1, found.size());
        assertTrue(found.get(0), found.get(0).contains("outside the measured"));
        // A plain shoot uses the cell latched by aim, and a branch checks both arms.
        String latched = auto("[{\"id\":\"a\",\"kind\":\"command\",\"name\":\"aim\",\"args\":{\"side\":\"FAR\"}},"
                + "{\"id\":\"b\",\"kind\":\"branch\",\"condition\":\"commandFailed\","
                + "\"then\":[{\"id\":\"s\",\"kind\":\"command\",\"name\":\"shoot\",\"args\":{\"seconds\":1}}],"
                + "\"else\":[{\"id\":\"w\",\"kind\":\"wait\",\"seconds\":0.1}]}]");
        assertEquals(1, violations(new JSONObject(latched), empty).size());
    }

    private static String auto(String steps) {
        return "{\"name\":\"synthetic\",\"alliance\":\"RED\",\"start\":{\"pose\":{\"xIn\":-12,\"yIn\":-63,\"headingRad\":1.5708}},\"steps\":" + steps + "}";
    }

    private static String read(File f) throws Exception { return new String(Files.readAllBytes(f.toPath()), "UTF-8"); }
}
