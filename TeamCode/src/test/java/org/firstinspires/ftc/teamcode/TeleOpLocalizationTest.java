package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import static org.junit.Assert.*;
import org.firstinspires.ftc.teamcode.RobotMath.Vec;

public class TeleOpLocalizationTest {
    private static final ShooterMount MOUNT = ShootingConfiguration.MOUNT;
    private static MotionSample local(double time, double x, double y, double heading, double vx, double vy) {
        return new MotionSample(time,x,y,heading,vx,vy,.2,.3,.4,1,true,false,"Local Pinpoint").withReference(7);
    }
    private static MotionSample field(double time, double x, double y, double heading) {
        return new MotionSample(time,x,y,heading,0,0,0,0,.02,true,true,"Synthetic field ground truth");
    }
    /** Project a known field tag and its outward normal into the rotating, pitched optical camera. */
    private static TagObservation observation(GoalTarget target, int id, MotionSample actual, double turretYaw) {
        GoalTarget.Tag known = target.tags.get(id);
        double cameraYaw = actual.heading+MOUNT.zeroYaw+turretYaw+MOUNT.cameraYaw;
        Vec ray = new Vec(known.x,known.y).minus(MOUNT.cameraPosition(actual,turretYaw)).rotate(-cameraYaw);
        double up = known.height-MOUNT.cameraHeight;
        double pitch = MOUNT.cameraPitch;
        double direction = target.side == TurretSubsystem.HiveSide.AUDIENCE ? -1 : 1;
        Vec normal = new Vec(0,direction*Math.cos(Math.toRadians(FieldGeometry.HIVE_TILT_DEG))).rotate(-cameraYaw);
        double normalUp = Math.sin(Math.toRadians(FieldGeometry.HIVE_TILT_DEG));
        double nx = -normal.y;
        double ny = normal.x*Math.sin(pitch)-normalUp*Math.cos(pitch);
        double nz = normal.x*Math.cos(pitch)+normalUp*Math.sin(pitch);
        return new TagObservation(id,-ray.y,ray.x*Math.sin(pitch)-up*Math.cos(pitch),
                ray.x*Math.cos(pitch)+up*Math.sin(pitch),Math.asin(-ny),Math.atan2(nx,nz),0,1);
    }
    private static GoalTarget audience() { return TeleOpFieldReference.targetForTag(38); }
    private static ShotProfile calibrated(GoalTarget target) {
        return new ShotProfile(target.geometryId,true,new ShotProfile.Row(.2,1,3000),new ShotProfile.Row(5,10,3400));
    }
    private static double turretFacing(GoalTarget target, MotionSample actual) {
        return RobotMath.wrap(Math.atan2(target.y-actual.y,target.x-actual.x)-actual.heading-MOUNT.zeroYaw);
    }
    @Test public void everyMappedTagAcquiresUnknownPositionAndHeadingWithoutCornerAssumption() {
        for (int id = 30; id <= 45; id++) {
            GoalTarget target = TeleOpFieldReference.targetForTag(id);
            PoseHistory history = new PoseHistory();
            double y = target.side == TurretSubsystem.HiveSide.AUDIENCE ? -1.3 : 1.3;
            double heading = id % 2 == 0 ? 2.8 : -2.9;
            for (int i = 0; i < PoseHistory.ACQUISITION_FRAMES; i++) {
                double time = 1+i*.02;
                MotionSample actual = field(time,.85,y,heading);
                double turretYaw = turretFacing(target,actual);
                history.add(local(time,0,0,0,0,0),turretYaw);
                TagObservation tag = observation(target,id,actual,turretYaw);
                assertTrue(tag.usable());
                boolean accepted = history.acquireFieldReference3d(time,i,time,tag,target,MOUNT);
                assertEquals(history.reason(),i == PoseHistory.ACQUISITION_FRAMES-1,accepted);
                assertEquals(accepted,history.current(time).fieldReferenced);
            }
            MotionSample result = history.current(1.04);
            assertEquals(.85,result.x,1e-8); assertEquals(y,result.y,1e-8);
            assertEquals(0,RobotMath.wrap(result.heading-heading),1e-8);
        }
        assertNull(TeleOpFieldReference.targetForTag(29));
        assertNull(TeleOpFieldReference.targetForTag(46));
    }
    @Test public void captureHistoryCompensatesMotionAndRotatesVelocityIntoFieldAxes() {
        GoalTarget target = audience(); PoseHistory history = new PoseHistory();
        double rotation = 2.8;
        for (int i = 0; i <= 12; i++) {
            double t = 1+i*.01;
            history.add(local(t,.2*(t-1),-.1*(t-1),.2*(t-1),.2,-.1),.4+.3*(t-1));
        }
        for (int i = 0; i < 3; i++) {
            double capture = 1.04+i*.02;
            Vec position = new Vec(.2*(capture-1),-.1*(capture-1)).rotate(rotation).plus(new Vec(.8,-1.3));
            MotionSample actual = field(capture,position.x,position.y,rotation+.2*(capture-1));
            double yaw = .4+.3*(capture-1);
            boolean accepted = history.acquireFieldReference3d(1.12,i,capture,observation(target,38,actual,yaw),target,MOUNT);
            assertEquals(history.reason(),i == 2,accepted);
        }
        // No camera for the rest of a match: the same transform continues to track measured motion.
        for (int i = 13; i <= 18000; i++) {
            double t = 1+i*.01;
            history.add(local(t,.2*(t-1),-.1*(t-1),.2*(t-1),.2,-.1),.4);
        }
        MotionSample result = history.current(181);
        Vec expected = new Vec(36,-18).rotate(rotation).plus(new Vec(.8,-1.3));
        Vec velocity = new Vec(.2,-.1).rotate(rotation);
        assertTrue(result.fieldReferenced); assertTrue(result.fresh(181,.1));
        assertEquals(expected.x,result.x,1e-8); assertEquals(expected.y,result.y,1e-8);
        assertEquals(velocity.x,result.vx,1e-8); assertEquals(velocity.y,result.vy,1e-8);
        assertEquals(0,RobotMath.wrap(result.heading-rotation-36),1e-8);
        assertEquals(.2,result.omega,0); assertEquals(.3,result.acceleration,0);
        assertEquals(.4,result.angularAcceleration,0);
        assertFalse(result.fresh(182,.1));
        history.add(local(181.02,0,0,0,0,0).withReference(8),0);
        assertFalse(history.current(181.02).fieldReferenced);
    }
    @Test public void duplicateStaleInconsistentAndWrongHeightFramesCannotBootstrap() {
        GoalTarget target = audience(); PoseHistory history = new PoseHistory();
        MotionSample actual = field(1,.8,-1.3,2.8);
        double yaw = turretFacing(target,actual);
        TagObservation good = observation(target,38,actual,yaw);
        for (int i = 0; i <= 50; i++) history.add(local(1+i*.01,0,0,0,0,0),yaw);
        assertFalse(history.acquireFieldReference3d(1.1,1,1.05,good,target,MOUNT));
        for (int i = 0; i < 5; i++) assertFalse(history.acquireFieldReference3d(1.1,1,1.05,good,target,MOUNT));
        assertFalse(history.current(1.1).fieldReferenced);
        assertFalse(history.acquireFieldReference3d(1.5,2,1,good,target,MOUNT));
        assertFalse(history.acquireFieldReference3d(1.5,3,1.6,good,target,MOUNT));
        assertFalse(history.acquireFieldReference3d(1.5,4,.5,good,target,MOUNT));
        TagObservation low = new TagObservation(38,good.x,good.y+.5,good.z,good.roll,good.pitch,good.yaw,1);
        assertFalse(history.acquireFieldReference3d(1.5,5,1.45,low,target,MOUNT));
        assertFalse(history.acquireFieldReference3d(1.5,6,1.45,good,target,MOUNT));
        TagObservation shifted = observation(target,38,field(1,.8,-1.6,2.8),yaw);
        assertFalse(history.acquireFieldReference3d(1.5,7,1.46,shifted,target,MOUNT));
        assertFalse(history.acquireFieldReference3d(1.5,8,1.47,good,target,MOUNT));
        assertFalse(history.current(1.5).fieldReferenced);
        ShotPlanner.Solution blocked = new ShotPlanner(MOUNT).solve(1.5,history.current(1.5),target,calibrated(target));
        assertFalse(blocked.aimable); assertTrue(blocked.reason.contains("field pose"));
    }
    @Test public void subsequentTranslationCorrectionsPreserveHeadingAndReferenceAcrossTargetOrTurretChanges() {
        GoalTarget target = audience(); PoseHistory history = new PoseHistory();
        MotionSample actual = field(1,.8,-1.3,2.8);
        double yaw = turretFacing(target,actual);
        TagObservation tag = observation(target,38,actual,yaw);
        for (int i = 0; i < 3; i++) {
            double time = 1+i*.02;
            history.add(local(time,0,0,0,.2,0),yaw);
            history.acquireFieldReference3d(time,i,time,tag,target,MOUNT);
        }
        ShotPlanner.Solution localized = new ShotPlanner(MOUNT).solve(1.04,history.current(1.04),target,calibrated(target));
        assertTrue(localized.reason,localized.valid);
        history.invalidateObservation(false); // Nearest scoring cell changed.
        assertTrue(history.current(1.04).fieldReferenced);
        history.invalidateObservation(true); // Turret rehomed; capture history must be rebuilt.
        history.add(local(1.06,.1,0,0,.2,0),yaw);
        Vec drifted = new Vec(.1,0).rotate(2.8).plus(new Vec(.8,-1.3));
        assertTrue(history.observe3d(1.06,3,1.06,tag,target,MOUNT));
        MotionSample corrected = history.current(1.06);
        assertEquals(.8+.65*(drifted.x-.8),corrected.x,1e-8);
        assertEquals(-1.3+.65*(drifted.y+1.3),corrected.y,1e-8);
        assertEquals(2.8,corrected.heading,1e-8);
        TagObservation jump = observation(target,38,field(1,.8,-1.3-.8,2.8),yaw);
        assertFalse(history.observe3d(1.06,4,1.06,jump,target,MOUNT));
        assertEquals(corrected.x,history.current(1.06).x,0);
    }
}
