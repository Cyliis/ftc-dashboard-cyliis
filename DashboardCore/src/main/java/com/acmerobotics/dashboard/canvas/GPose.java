package com.acmerobotics.dashboard.canvas;

// Global Pose Class
public class GPose {
    private final double x, y, heading;

    public GPose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }
}
