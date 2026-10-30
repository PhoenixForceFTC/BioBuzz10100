package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import org.junit.Test;
import static org.junit.Assert.*;

public class PollenPathFollowerTest {
    @Test public void repeatedFramesDoNotRestartAndACompletedShortApproachCanReplan() {
        CompetitionAutonomousTest.World world = new CompetitionAutonomousTest.World();
        PollenPathFollower navigation = new PollenPathFollower(world.follower, (path,speed) -> path, () -> world.following=false);
        navigation.update(PollenNavigator.Motion.FOLLOW,new Pose(12,0,0),1_000_000_000L);
        navigation.update(PollenNavigator.Motion.FOLLOW,new Pose(12.2,0,0),1_300_000_000L);
        assertEquals(1,world.followed.size());
        // Pedro arrived, but a fresh observation still needs a small correction, below the replan threshold.
        world.following=false; world.pose=new Pose(11.5,0,0);
        navigation.update(PollenNavigator.Motion.FOLLOW,new Pose(12.2,0,0),1_400_000_000L);
        assertEquals(2,world.followed.size());
        assertEquals(11.5,world.followed.get(1).getSegments().get(0).get(0).x(),1e-9);
    }
    @Test public void targetLossStopsImmediatelyAndAlignmentUsesHoldInsteadOfAZeroLengthPath() {
        CompetitionAutonomousTest.World world = new CompetitionAutonomousTest.World();
        int[] stopped={0};
        PollenPathFollower navigation = new PollenPathFollower(world.follower, (path,speed) -> path,
                () -> { stopped[0]++; world.following=false; });
        navigation.update(PollenNavigator.Motion.FOLLOW,new Pose(12,0,0),1_000_000_000L);
        navigation.update(PollenNavigator.Motion.STOP,Pose.zero(),1_010_000_000L);
        assertEquals(1,stopped[0]); assertFalse(world.following);
        navigation.update(PollenNavigator.Motion.HOLD,new Pose(0,0,-Math.PI/2),1_020_000_000L);
        assertEquals(1,world.holds); assertEquals(1,world.followed.size());
    }
}
