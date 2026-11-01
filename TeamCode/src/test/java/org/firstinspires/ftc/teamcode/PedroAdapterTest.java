package org.firstinspires.ftc.teamcode;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.api.Paths;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import org.junit.Test;
import static org.junit.Assert.*;

public class PedroAdapterTest {
    @Test public void plannerDriveNumbersAgreeWithTheInstalledRobotConfiguration() throws Exception {
        org.json.JSONObject robot = new org.json.JSONObject(new String(java.nio.file.Files.readAllBytes(
                new java.io.File("../autos/robot.json").toPath()), java.nio.charset.StandardCharsets.UTF_8));
        org.json.JSONObject k = robot.getJSONObject("kinematics"), f = k.getJSONObject("follower");
        String[] motionKeys = {"maxForwardVelInPerS","maxStrafeVelInPerS","forwardDecelInPerS2",
                "strafeDecelInPerS2","defaultPathSpeedFraction"};
        double[] motionValues = {PedroConfiguration.MAX_FORWARD_VELOCITY_IN_PER_SEC,
                PedroConfiguration.MAX_STRAFE_VELOCITY_IN_PER_SEC,PedroConfiguration.FORWARD_DECELERATION_IN_PER_SEC_SQUARED,
                PedroConfiguration.STRAFE_DECELERATION_IN_PER_SEC_SQUARED,PedroConfiguration.DEFAULT_PATH_SPEED_FRACTION};
        String[] followerKeys = {"brakeLinearForwardS","brakeLinearStrafeS","brakeQuadraticForwardS2PerIn",
                "brakeQuadraticStrafeS2PerIn","endHeadingToleranceRad","endTranslationalToleranceIn","endVelocityToleranceInPerS"};
        double[] followerValues = {PedroConfiguration.FORWARD_LINEAR_BRAKE_SECONDS,PedroConfiguration.STRAFE_LINEAR_BRAKE_SECONDS,
                PedroConfiguration.FORWARD_QUADRATIC_BRAKE_SEC_SQUARED_PER_IN,PedroConfiguration.STRAFE_QUADRATIC_BRAKE_SEC_SQUARED_PER_IN,
                PedroConfiguration.END_HEADING_TOLERANCE_RAD,PedroConfiguration.END_TRANSLATIONAL_TOLERANCE_IN,
                PedroConfiguration.END_VELOCITY_TOLERANCE_IN_PER_SEC};
        for (int i=0;i<motionKeys.length;i++) assertEquals(motionKeys[i],motionValues[i],k.getJSONObject(motionKeys[i]).getDouble("value"),1e-12);
        for (int i=0;i<followerKeys.length;i++) assertEquals(followerKeys[i],followerValues[i],f.getJSONObject(followerKeys[i]).getDouble("value"),1e-12);
    }
    @Test public void pinpointConversionPreservesFieldAxesAndConvertsOnlyLinearUnits() {
        MotionSample sample=ControllerFixtures.pose(1,.0254,-.0508,Math.PI/2,.254,-.508,.3);
        Pose[] seeded={null};
        PedroPinpointAdapter localizer=new PedroPinpointAdapter(() -> sample,p -> seeded[0]=p);
        localizer.update();
        assertEquals(1,localizer.pose().x(),1e-9); assertEquals(-2,localizer.pose().y(),1e-9);
        assertEquals(Math.PI/2,localizer.pose().heading(),1e-9);
        assertEquals(10,localizer.velocity().vx,1e-9); assertEquals(-20,localizer.velocity().vy,1e-9);
        assertEquals(.3,localizer.velocity().omega,1e-9);
        localizer.setPose(new Pose(12,-24,Math.PI));
        assertEquals(12,seeded[0].x(),0); assertEquals(-24,seeded[0].y(),0);
        assertEquals(0,localizer.velocity().vx,0);
    }
    @Test public void mecanumMixUsesPedroLeftAndCcwAndBoundsWheelPower() {
        double[][] output={null};
        PedroMecanumAdapter drive=new PedroMecanumAdapter(p -> output[0]=p,.35);
        drive.drive(new DrivePowers(.2,0,0),false); assertArrayEquals(new double[]{.2,.2,.2,.2},output[0],1e-9);
        drive.drive(new DrivePowers(0,.2,0),false); assertArrayEquals(new double[]{-.2,.2,.2,-.2},output[0],1e-9);
        drive.drive(new DrivePowers(0,0,.2),false); assertArrayEquals(new double[]{-.2,.2,-.2,.2},output[0],1e-9);
        assertEquals(.625,drive.maxScaling(new DrivePowers(.1,0,0),new DrivePowers(.4,0,0)),1e-9);
        drive.drive(new DrivePowers(1,1,1),false);
        for(double p:output[0]) assertTrue(Math.abs(p)<=.35+1e-9);
        drive.drive(new DrivePowers(Double.NaN,0,0),false); assertArrayEquals(new double[4],output[0],0);
    }
    @Test public void installedForesightDrivesHoldsAndAppliesPathSpeedModifiers() {
        MotionSample sample=ControllerFixtures.pose(1,0,0,0,0,0,0);
        PedroPinpointAdapter localizer=new PedroPinpointAdapter(() -> sample,p -> {});
        double[][] output={null};
        PedroMecanumAdapter drive=new PedroMecanumAdapter(p -> output[0]=p,.35);
        ForesightConfig config=PedroConfiguration.create();
        Follower follower=new Follower(localizer,drive,new PedroForesight(config));
        follower.hold(Pose.zero()); follower.update(.02);
        assertArrayEquals(new double[4],output[0],0);
        follower.hold(new Pose(0,0,.1)); follower.update(.02);
        assertTrue(output[0][0]<0); assertTrue(output[0][1]>0);
        follower.follow(Paths.line(Pose.zero(),new Pose(12,0,0)).constant(0).with(config.setPathSpeed(.2)));
        assertEquals(PedroConfiguration.MAX_FORWARD_VELOCITY_IN_PER_SEC*.2,config.maxVelocityConstraint.get(),1e-9);
        follower.update(.02);
        for(double p:output[0]) assertTrue(Double.isFinite(p));
        follower.stop(); drive.stop();
        assertEquals(Double.POSITIVE_INFINITY,config.maxVelocityConstraint.get(),0);
        assertArrayEquals(new double[4],output[0],0);
    }
}
