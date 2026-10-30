package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import static org.junit.Assert.*;

public class PollenVisibilityTest {
    private final PollenConfiguration config=PollenConfiguration.diagnostic();
    private final PollenCameraModel model=PollenCameraModel.approximate(config);
    /** Independently projects a world floor contact using the confirmed mount. */
    private double[] pixel(double forward,double right) {
        double p=Math.toRadians(config.pitchDeg), f=forward-config.forwardMm, r=right-config.rightMm;
        double z=f*Math.cos(p)+config.heightMm*Math.sin(p), down=-f*Math.sin(p)+config.heightMm*Math.cos(p);
        return new double[]{model.cx+model.fx*r/z,model.cy+model.fy*down/z};
    }
    private PollenTarget target(long timestamp,double forward) {
        double[] px=pixel(forward,0);
        return model.project(config,timestamp,px[0],px[1]-10,px[0],px[1],500);
    }
    @Test public void old350mmTransitionIsInvisibleAndNewExampleIsVisibleButStillUncalibrated() {
        double near=model.nearestPickupVisibleMm(config);
        assertTrue(near>540 && near<570);
        assertNull(target(1,350)); assertNotNull(target(1,650));
        assertTrue(model.pickupVisible(config)); assertFalse(config.autonomousEligible(true));
    }
    @Test public void syntheticFloorProjectionsRoundTripWithOffsetsAndCameraPitch() {
        for(double f:new double[]{600,800,1200,2000}) for(double r:new double[]{-50,0,50}) {
            double[] px=pixel(f,r);
            PollenTarget t=model.project(config,1,px[0],px[1]-10,px[0],px[1],500);
            assertNotNull(t); assertEquals(f,t.forwardMm,1e-8); assertEquals(r,t.rightMm,1e-8);
        }
    }
    @Test public void validDistortionIsRemovedBeforeFloorIntersectionAndOriginalPixelsArePreserved() {
        double[] d={.12,-.03,.002,-.001,.005};
        PollenCameraModel distorted=new PollenCameraModel(320,240,model.fx,model.fy,model.cx,model.cy,d);
        double[] ideal=pixel(700,100); double x=(ideal[0]-160)/model.fx,y=(ideal[1]-120)/model.fy;
        double r2=x*x+y*y,radial=1+d[0]*r2+d[1]*r2*r2+d[4]*r2*r2*r2;
        double px=160+model.fx*(x*radial+2*d[2]*x*y+d[3]*(r2+2*x*x));
        double py=120+model.fy*(y*radial+d[2]*(r2+2*y*y)+2*d[3]*x*y);
        PollenTarget t=distorted.project(config,1,px,py-10,px,py,500);
        assertNotNull(t); assertEquals(700,t.forwardMm,1e-5); assertEquals(100,t.rightMm,1e-5);
        assertEquals(px,t.contactXPx,0); assertEquals(py,t.contactYPx,0);
    }
    @Test public void activeIntrinsicsChangeVisibilityAndEnabledInvisibleConfigurationIsRejected() {
        PollenCameraModel zoom=new PollenCameraModel(320,240,500,500,160,120,null);
        assertTrue(zoom.nearestPickupVisibleMm(config)>model.nearestPickupVisibleMm(config));
        assertFalse(zoom.pickupVisible(config));
        assertThrows(IllegalArgumentException.class,()->new PollenConfiguration(195,10,248,0,60,500,500,160,120,
                true,true,true,true,false,20,40,80,80,350,200));
        assertThrows(IllegalArgumentException.class,()->new PollenCameraModel(320,240,0,200,160,120,null));
        assertThrows(IllegalArgumentException.class,()->new PollenCameraModel(320,240,200,200,160,120,new double[]{Double.NaN,0,0,0}));
    }
    @Test public void visibleConfirmationCommitsBlindPickupButPrematureLossDoesNot() {
        PollenNavigator n=new PollenNavigator(config); long start=1_000_000_000L;
        n.start(start,0,0);
        n.update(start,target(start,640),0,0,0,true,false);
        n.update(start+70_000_000L,target(start+70_000_000L,640),0,0,0,true,false);
        n.update(start+150_000_000L,target(start+150_000_000L,640),0,0,0,true,false);
        assertEquals(PollenNavigator.State.COLLECTING,n.getState());
        n.update(start+200_000_000L,null,50,0,2,true,false); assertEquals(PollenNavigator.Motion.FOLLOW,n.getMotion());
        n.update(start+400_000_000L,null,config.pickupTravelMm+1,0,0,true,false); assertEquals(PollenNavigator.Motion.STOP,n.getMotion());
        assertEquals(PollenNavigator.State.SETTLING,n.getState());
        n.start(start,0,0); n.update(start,target(start,800),0,0,0,true,false);
        n.update(start+50_000_000L,null,10,0,0,true,false);
        assertEquals(PollenNavigator.State.APPROACHING,n.getState()); assertEquals(PollenNavigator.Motion.STOP,n.getMotion());
    }
    @Test public void committedPickupStillStopsForStaleCameraInvalidOdomFullAndStop() {
        for(int mode=0;mode<4;mode++) {
            PollenNavigator n=new PollenNavigator(config); long start=1_000_000_000L; n.start(start,0,0);
            for(long dt:new long[]{0,70_000_000L,150_000_000L}) n.update(start+dt,target(start+dt,640),0,0,0,true,false);
            if(mode==3) n.stop(start+200_000_000L);
            else n.update(start+200_000_000L,null,mode==1?Double.NaN:50,0,0,mode!=0,mode==2);
            assertEquals(PollenNavigator.Motion.STOP,n.getMotion()); assertEquals(0,n.getIntakePower(),0);
        }
    }
}
