package org.firstinspires.ftc.teamcode;

/** Active pinhole model with optional OpenCV Brown-Conrady rational distortion (4/5/8 coefficients).
 * Pixel bounds match the full-contour clipping policy, not just the blob's center.
 */
public final class PollenCameraModel {
    public static final int CLIP_MARGIN_PX = 2;
    public final int width, height;
    public final double fx, fy, cx, cy;
    private final double[] distortion;
    public PollenCameraModel(int width, int height, double fx, double fy, double cx, double cy, double[] distortion) {
        if (width < 10 || height < 10 || !RobotMath.finite(fx,fy,cx,cy) || fx <= 0 || fy <= 0 || cx < 0 || cy < 0 || cx >= width || cy >= height)
            throw new IllegalArgumentException("Invalid active camera intrinsics");
        this.width = width; this.height = height; this.fx = fx; this.fy = fy; this.cx = cx; this.cy = cy;
        this.distortion = distortion == null ? new double[0] : distortion.clone();
        if (this.distortion.length != 0 && this.distortion.length != 4 && this.distortion.length != 5 && this.distortion.length != 8)
            throw new IllegalArgumentException("Expected 4, 5 or 8 distortion coefficients");
        if (!RobotMath.finite(this.distortion)) throw new IllegalArgumentException("Nonfinite distortion");
    }
    private double coefficient(int index) { return index < distortion.length ? distortion[index] : 0; }
    private double[] distort(double x, double y) {
        double r2=x*x+y*y, r4=r2*r2, r6=r4*r2;
        double radial=(1+coefficient(0)*r2+coefficient(1)*r4+coefficient(4)*r6)
                /(1+coefficient(5)*r2+coefficient(6)*r4+coefficient(7)*r6);
        return new double[]{x*radial+2*coefficient(2)*x*y+coefficient(3)*(r2+2*x*x),
                y*radial+coefficient(2)*(r2+2*y*y)+2*coefficient(3)*x*y};
    }
    public double[] undistort(double pixelX, double pixelY) {
        double xd=(pixelX-cx)/fx, yd=(pixelY-cy)/fy, x=xd, y=yd;
        for (int i=0;i<20;i++) {
            double[] projected=distort(x,y);
            double dx=xd-projected[0], dy=yd-projected[1];
            x+=dx; y+=dy;
            if (Math.hypot(dx,dy)<1e-10) break;
        }
        double[] check=distort(x,y);
        if (!RobotMath.finite(x,y) || Math.hypot(check[0]-xd,check[1]-yd)>1e-6) return null;
        return new double[]{cx+fx*x,cy+fy*y};
    }
    public boolean distortionApplied() { return distortion.length > 0; }
    public PollenTarget project(PollenConfiguration config, long time, double centerX, double centerY, double x, double y, double area) {
        if (x < CLIP_MARGIN_PX || x >= width-CLIP_MARGIN_PX || y < CLIP_MARGIN_PX || y >= height-CLIP_MARGIN_PX) return null;
        double[] point=undistort(x,y);
        if (point == null) return null;
        PollenTarget target = PollenTarget.fromFloorContact(time,centerX,centerY,point[0],point[1],area,fx,fy,cx,cy,
                config.heightMm,config.pitchDeg,config.forwardMm,config.rightMm);
        return target == null ? null : target.withContactPixels(x,y);
    }
    /** Conservative near boundary across the central pickup alignment cone. */
    public double nearestPickupVisibleMm(PollenConfiguration config) {
        double near=0;
        for (int i=-1;i<=1;i++) {
            PollenTarget point=project(config,1,cx,cy,cx+i*.08*fx,height-CLIP_MARGIN_PX-1,100);
            if (point == null) return Double.POSITIVE_INFINITY;
            near=Math.max(near,point.forwardMm);
        }
        return near;
    }
    public double farthestPickupVisibleMm(PollenConfiguration config) {
        double far = 2200;
        for (int i=-1;i<=1;i++) {
            PollenTarget point=project(config,1,cx,cy,cx+i*.08*fx,CLIP_MARGIN_PX,100);
            if (point != null) far=Math.min(far,point.forwardMm);
        }
        return far;
    }
    public boolean pickupVisible(PollenConfiguration config) {
        // Require a small pixel/range margin so qualification can occur before contour clipping.
        return config.pickupStartMm >= nearestPickupVisibleMm(config)+20 && config.pickupStartMm <= farthestPickupVisibleMm(config)-20;
    }
    public static PollenCameraModel approximate(PollenConfiguration config) {
        double focal=320/(2*Math.tan(Math.toRadians(config.horizontalFovDeg/2)));
        return new PollenCameraModel(320,240,config.intrinsicsMeasured?config.fx:focal,
                config.intrinsicsMeasured?config.fy:focal,config.intrinsicsMeasured?config.cx:160,
                config.intrinsicsMeasured?config.cy:120,config.distortionCoefficients());
    }
}
