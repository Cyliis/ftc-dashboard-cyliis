package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.canvas.GPose;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utils.Pose;
import org.firstinspires.ftc.teamcode.Utils.Vector;

@Config
public class PredictiveLocalizer {
    public static double xDeceleration = 120, yDeceleration = 300;
    private Vector velocity = new Vector(0,0,0);
    public Vector glideDelta = new Vector(0,0,0);
    private Pose lastPose = new Pose(0,0,0);
    private final Localizer localizer;

    public PredictiveLocalizer(Localizer localizer) {
        this.localizer = localizer;
        velocityTimer.startTime();
        velocityTimer.reset();
    }

    public Pose getPoseEstimate(){
        return new Pose(localizer.pose.getX() + glideDelta.getX(), localizer.pose.getY() + glideDelta.getY(), localizer.pose.getHeading());
    }

    public Vector getVelocity(){
        return velocity;
    }

    private ElapsedTime velocityTimer = new ElapsedTime();

    public void update(){
        velocity = new Vector(localizer.pose.getX() - lastPose.getX(), localizer.pose.getY() - lastPose.getY()).scaledBy(1.0/velocityTimer.seconds());
        velocityTimer.reset();
        Vector predictedGlideVector = Vector.rotateBy(velocity,-localizer.pose.getHeading());
        glideDelta = Vector.rotateBy(new Vector(Math.pow(predictedGlideVector.getX(),2)/(2.0*xDeceleration) * Math.signum(predictedGlideVector.getX()),
                Math.pow(predictedGlideVector.getY(),2)/(2.0*yDeceleration) * Math.signum(predictedGlideVector.getY())), localizer.pose.getHeading());
        lastPose = localizer.pose;
    }
}
