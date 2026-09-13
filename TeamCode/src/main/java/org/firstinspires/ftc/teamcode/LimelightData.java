package org.firstinspires.ftc.teamcode;

public class LimelightData {
    private int id;
    private double angleX;
    private double angleY;
    private double x;
    private double y;
    private double z;
    private double distance;

    public LimelightData(int i, double ax, double ay, double x, double y, double z){
        id = i;
        angleX = ax;
        angleY = ay;
        this.x = x;
        this.y = y;
        this.z = z;
        distance = Math.sqrt(x * x + y * y + z * z);
    }

    public int getId() {
        return id;
    }

    public double getAngleX() {
        return angleX;
    }

    public double getAngleY() {
        return angleY;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }

    public double getDistance() {
        return distance;
    }



}
