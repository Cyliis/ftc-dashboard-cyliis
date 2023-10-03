package org.firstinspires.ftc.teamcode.Math;

public class LowPassFilter {

    private double t;
    private double lastValue;

    public LowPassFilter (double t, double initialValue){
        this.t = t;
        this.lastValue = initialValue;
    }

    public double getValue(double rawValue){
        double newValue = lastValue * (1.0-t) + t * rawValue;
        this.lastValue = newValue;
        return newValue;
    }
}
