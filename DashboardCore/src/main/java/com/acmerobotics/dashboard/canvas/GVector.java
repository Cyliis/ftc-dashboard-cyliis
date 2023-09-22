package com.acmerobotics.dashboard.canvas;

public class GVector {
    private double x,y,z;

    public GVector(double x, double y, double z){
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public GVector(double x, double y){
        this(x,y,0);
    }

    public GVector(){
        this(0,0,0);
    }

    public GVector(GVector otherGVector){
        this.x = otherGVector.x;
        this.y = otherGVector.y;
        this.z = otherGVector.z;
    }

    public static GVector fromAngleAndMagnitude(double t, double m){
        return new GVector(m*Math.cos(t), m*Math.sin(t));
    }

    //t1 is angle in xy plane, t2 is angle with xy plane
    public static GVector fromAngleAndMagnitude(double t1, double t2, double m){
        return new GVector(Math.cos(t1)*Math.cos(t2)*m, Math.sin(t1)*Math.cos(t2)*m, Math.sin(t2)*m);
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

    public double getMagnitude(){
        return Math.sqrt(x*x + y*y + z*z);
    }

    public GVector plus(GVector other){
        return new GVector(x + other.getX(), y + other.getY(), z + other.getZ());
    }

    public void scaleToMagnitude(double targetMagnitude){
        double currentMagnitude = getMagnitude();
        scaleBy(1.0/currentMagnitude);
        scaleBy(targetMagnitude);
    }

    public void scaleBy(double a){
        x = x * a;
        y = y * a;
        z = z * a;
    }

    public GVector scaledBy(double a){
        return new GVector(x * a, y * a, z * a);
    }

    public GVector scaledToMagnitude(double targetMagnitude){
        GVector aux = new GVector(this);
        aux.scaleToMagnitude(targetMagnitude);
        return aux;
    }

    public static GVector rotateBy(GVector GVector, double theta){
        return new GVector(Math.cos(theta) * GVector.getX() + Math.sin(theta) * GVector.getY(), Math.cos(theta) * GVector.getY() - Math.sin(theta) * GVector.getX());
    }

    @Override
    public String toString(){
        return String.valueOf(x) + " " + String.valueOf(y) + " " + String.valueOf(z);
    }
}
