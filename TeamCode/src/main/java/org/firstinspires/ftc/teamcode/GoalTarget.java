package org.firstinspires.ftc.teamcode;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

/** One explicitly selected opening and hive state. Visibility never changes the selected side/state. */
public final class GoalTarget {
    public enum Frame { FIELD, LOCAL_ODOMETRY }
    public final Frame frame;
    public enum HiveState { UNKNOWN, RAISED, LOWERED }
    public static final class Tag {
        public final int id;
        public final double x, y, height;
        public Tag(int id, double x, double y, double height) {
            if (!RobotMath.finite(x, y, height) || height <= 0) throw new IllegalArgumentException("Invalid tag geometry");
            this.id = id; this.x = x; this.y = y; this.height = height;
        }
    }
    public final TurretSubsystem.Alliance alliance;
    public final TurretSubsystem.HiveSide side;
    public final HiveState state;
    public final double x, y, height;
    public final boolean calibrated;
    public final String geometryId;
    public final Map<Integer, Tag> tags;
    public GoalTarget(TurretSubsystem.Alliance alliance, TurretSubsystem.HiveSide side, HiveState state,
                      double x, double y, double height, boolean calibrated, String geometryId, Tag... tags) {
        this(alliance,side,state,x,y,height,calibrated,geometryId,Frame.FIELD,tags);
    }
    private GoalTarget(TurretSubsystem.Alliance alliance, TurretSubsystem.HiveSide side, HiveState state,
                      double x, double y, double height, boolean calibrated, String geometryId, Frame frame, Tag... tags) {
        this.frame = frame;
        if (alliance == null || side == null || state == null || geometryId == null || geometryId.isEmpty()
                || !RobotMath.finite(x, y, height) || height <= 0) throw new IllegalArgumentException("Invalid target");
        this.alliance = alliance; this.side = side; this.state = state;
        this.x = x; this.y = y; this.height = height; this.calibrated = calibrated; this.geometryId = geometryId;
        Map<Integer, Tag> map = new LinkedHashMap<>();
        for (Tag tag : tags) {
            if (tag == null || TurretSubsystem.getHiveSideForTag(alliance, tag.id) != side || map.put(tag.id, tag) != null)
                throw new IllegalArgumentException("Tag does not match selected opening");
        }
        this.tags = Collections.unmodifiableMap(map);
    }
    public GoalTarget inLocalFrame() {
        return new GoalTarget(alliance,side,state,x,y,height,calibrated,geometryId,Frame.LOCAL_ODOMETRY,tags.values().toArray(new Tag[0]));
    }
    // Field aim geometry does not require a visible tag or a localization tag map.
    public boolean scoreable() { return calibrated && state == HiveState.RAISED
            && (frame == Frame.FIELD || !tags.isEmpty()); }
    public String identity() { return frame + "/" + alliance + "/" + side + "/" + state + "/" + geometryId; }
    public static GoalTarget unknown(TurretSubsystem.Alliance alliance, TurretSubsystem.HiveSide side) {
        return new GoalTarget(alliance, side, HiveState.UNKNOWN,
                TurretSubsystem.getHiveOpeningCenterXInches(alliance) * .0254,
                TurretSubsystem.getHiveOpeningCenterYInches(side) * .0254,
                ShootingConfiguration.HIGH_GOAL_CENTER_HEIGHT_METERS, false, "UNMEASURED");
    }
}
