package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.canvas.GPose;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.Utils.IRobotModule;
import org.firstinspires.ftc.teamcode.Utils.Pose;
import org.firstinspires.ftc.teamcode.Wrappers.CoolIMU;
import org.firstinspires.ftc.teamcode.drive.FunnyLocalizer;

import java.util.ArrayList;

@Config
public class Localizer implements IRobotModule {
    protected Pose pose;
    private FunnyLocalizer localizer;
    public CoolIMU imu;
    private ArrayList<Pose>poses = new ArrayList<>();

    public Localizer(HardwareMap hm, Pose initialPose){
        this.pose = initialPose;
        this.imu = new CoolIMU(hm);
        this.localizer = new FunnyLocalizer(hm, imu);
        localizer.setPoseEstimate(new Pose2d(initialPose.getX(), initialPose.getY(), initialPose.getHeading()));
    }

    public Localizer(HardwareMap hm,DcMotorEx parallelEncoder, DcMotorEx perpendicularEncoder, Pose initialPose){
        this.pose = initialPose;
        this.imu = new CoolIMU(hm);
        this.localizer = new FunnyLocalizer(parallelEncoder, perpendicularEncoder, imu);
        localizer.setPoseEstimate(new Pose2d(initialPose.getX(), initialPose.getY(), initialPose.getHeading()));
    }

    public void setPose(Pose pose){
        this.pose = pose;
    }

    public Pose getPoseEstimate(){
        return pose;
    }

    public ArrayList<GPose> getAllGPoses(){
        ArrayList<GPose> gPoses = new ArrayList<>();
        for(Pose pose: poses){
            gPoses.add(new GPose(pose.getX(),pose.getY(),pose.getHeading()));
        }
        return  gPoses;
    }

    @Override
    public void update() {
        localizer.update();
        Pose2d pose2d = localizer.getPoseEstimate();
        pose = new Pose(pose2d.getX(), pose2d.getY(), pose2d.getHeading());
        poses.add(pose);
    }

    @Override
    public void emergencyStop() {

    }
}
