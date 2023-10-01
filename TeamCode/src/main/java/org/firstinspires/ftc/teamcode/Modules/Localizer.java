package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.canvas.GPose;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Utils.Pose;
import org.firstinspires.ftc.teamcode.Wrappers.CoolIMU;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.FunnyLocalizer;

import java.util.ArrayList;

@Config
public class Localizer implements IRobotModule {

    public static boolean ENABLED = true;

    protected Pose pose;
    private FunnyLocalizer localizer;
    public CoolIMU imu;
    private ArrayList<Pose> poses = new ArrayList<>();

    public Localizer(Hardware hardware, Pose initialPose) {
        this.pose = initialPose;
        this.imu = hardware.imu;
        this.localizer = new FunnyLocalizer(hardware.mch0, hardware.mch3, hardware.imu);
        localizer.setPoseEstimate(new Pose2d(initialPose.getX(), initialPose.getY(), initialPose.getHeading()));
    }

    public Localizer(Hardware hardware) {
        this.pose = new Pose();
        this.imu = hardware.imu;
        this.localizer = new FunnyLocalizer(hardware.mch0, hardware.mch3, hardware.imu);
        localizer.setPoseEstimate(new Pose2d());
    }

    public void setPose(Pose pose) {
        this.pose = pose;
    }

    public Pose getPoseEstimate() {
        return pose;
    }

    public ArrayList<GPose> getAllGPoses() {
        ArrayList<GPose> gPoses = new ArrayList<>();
        for (Pose pose : poses) {
            gPoses.add(new GPose(pose.getX(), pose.getY(), pose.getHeading()));
        }
        return gPoses;
    }

    @Override
    public void update() {
        if(!ENABLED) return;

        localizer.update();
        Pose2d pose2d = localizer.getPoseEstimate();
        pose = new Pose(pose2d.getX(), pose2d.getY(), pose2d.getHeading());
        poses.add(pose);
    }
}