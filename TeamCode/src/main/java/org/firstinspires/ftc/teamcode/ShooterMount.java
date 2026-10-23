package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.RobotMath.Vec;

/** Separate robot->pivot, pivot->camera and pivot->muzzle offsets (meters). */
public final class ShooterMount {
    public final Vec pivot, camera, muzzle;
    public final double zeroYaw, cameraYaw, cameraPitch, cameraHeight;
    public final boolean measured;
    public ShooterMount(Vec pivot, Vec camera, Vec muzzle, double zeroYaw, double cameraYaw,
                        double cameraPitch, double cameraHeight, boolean measured) {
        if (pivot == null || camera == null || muzzle == null || !RobotMath.finite(pivot.x, pivot.y,
                camera.x, camera.y, muzzle.x, muzzle.y, zeroYaw, cameraYaw, cameraPitch, cameraHeight)
                || cameraHeight <= 0) throw new IllegalArgumentException("Invalid shooter mount");
        this.pivot = pivot; this.camera = camera; this.muzzle = muzzle; this.zeroYaw = zeroYaw;
        this.cameraYaw = cameraYaw; this.cameraPitch = cameraPitch; this.cameraHeight = cameraHeight;
        this.measured = measured;
    }
    public Vec cameraPosition(MotionSample pose, double turretYaw) {
        return new Vec(pose.x, pose.y).plus(pivot.rotate(pose.heading))
                .plus(camera.rotate(pose.heading + zeroYaw + turretYaw));
    }
    /** tx is right positive; ty up positive. Uses this tag's height, never the opening height. */
    public Vec cameraToTag(GoalTarget.Tag tag, double tx, double ty, double fieldCameraYaw) {
        if (!RobotMath.finite(tx, ty, fieldCameraYaw) || Math.abs(tx) >= Math.PI / 2 || Math.abs(ty) >= Math.PI / 2) return null;
        // Camera ray: optical forward, left=-tan(tx), up=tan(ty). Pitch before field yaw.
        double forward = Math.cos(cameraPitch) - Math.tan(ty) * Math.sin(cameraPitch);
        double up = Math.sin(cameraPitch) + Math.tan(ty) * Math.cos(cameraPitch);
        double scale = (tag.height - cameraHeight) / up;
        if (!RobotMath.finite(scale) || scale <= 0 || forward <= 0) return null;
        return new Vec(forward * scale, -Math.tan(tx) * scale).rotate(fieldCameraYaw);
    }
}
