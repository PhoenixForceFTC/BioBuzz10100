package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

//usage: create this class in the main code. every loop, pass in the tag you want to face and
// add the returned power to the drive's turn input. pass null when no tag is seen and it returns 0.
// it doesn't touch any motors itself, so the driver can still drive/strafe while it aims,
// and the same class can drive a turret motor later.
// it can also take a plain angle error in degrees, which is how odometry aiming uses the same PID.

public class RotateToLimelight {

    // ---- Target angle to hold, in degrees ----
    // 0 = tag dead center in the camera. if the camera is mounted off-center from the shooter, offset this.
    private double targetAngle = 0;

    // ---- PID gains: tune these ----
    private double kP = 0.012;
    private double kI = 0.0;

    private double kD = 0.0008;

    // Max turn power the PID loop is allowed to command, 0-1.
    private double MAX_POWER = 0.6;

    // Within this many degrees counts as "aimed".
    private double TOLERANCE_DEG = 1.0;

    private double integralSum = 0;
    private double lastError = 0;
    private boolean atTarget = false;
    private boolean active = false;

    private ElapsedTime pidTimer = new ElapsedTime();

    // Returns turn power in the same convention as gamepad right_stick_x: positive = turn right.
    public double getTurnPower(LimelightData d) {

        if (d == null) {
            reset();
            return 0;
        }

        // Limelight tx (angleX) is positive when the tag is right of center, so turning right shrinks it.
        return getTurnPower(d.getAngleX() - targetAngle);
    }

    // Same PID, for any aiming source. error is in degrees: positive = target is to the right.
    public double getTurnPower(double error) {

        // fresh start: lastError = error makes the first derivative 0, so the first loop doesn't jerk the robot.
        if (!active) {
            active = true;
            integralSum = 0;
            lastError = error;
            pidTimer.reset();
        }

        atTarget = Math.abs(error) <= TOLERANCE_DEG;
        if (atTarget) {
            return 0;
        }

        double dt = pidTimer.seconds();
        pidTimer.reset();
        if (dt <= 0) dt = 1e-3;

        integralSum += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double turnPower = (kP * error) + (kI * integralSum) + (kD * derivative);
        return clip(turnPower, -MAX_POWER, MAX_POWER);
    }

    public boolean isAtTarget() {
        return atTarget;
    }

    public void reset() {
        active = false;
        atTarget = false;
    }

    private double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

}
