package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.GPose;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Follower;
import org.firstinspires.ftc.teamcode.Modules.Localizer;
import org.firstinspires.ftc.teamcode.Modules.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.TrajectoryStuff.CubicBezierTangentHeadingTrajectorySegment;
import org.firstinspires.ftc.teamcode.TrajectoryStuff.Trajectory;
import org.firstinspires.ftc.teamcode.TrajectoryStuff.TrajectoryBuilder;
import org.firstinspires.ftc.teamcode.Utils.Pose;

import java.util.List;

@Config
@TeleOp(name = "Sample")
public class SampleOpMode extends LinearOpMode {
    List<LynxModule> hubs;

    Hardware hardware;

    MecanumDrive drive;
    Follower follower;
    Localizer localizer;

    FtcDashboard dash;

    CubicBezierTangentHeadingTrajectorySegment segment1 = new CubicBezierTangentHeadingTrajectorySegment(
            new Pose(0,0),
            new Pose(24,0),
            new Pose(72,24),
            new Pose(72,0)
    );

    CubicBezierTangentHeadingTrajectorySegment segment2 = new CubicBezierTangentHeadingTrajectorySegment(
            segment1,
            new Pose(0, -48),
            new Pose(0,0,PI/2)
    );

    Trajectory lol = new TrajectoryBuilder(segment1)
            .addSegment(segment2)
            .build();

    @Override
    public void runOpMode() throws InterruptedException {

        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        hardware = new Hardware(hardwareMap);

        drive = new MecanumDrive(hardware, localizer);
        localizer = new Localizer(hardware,  new Pose());
        drive.setLocalizer(localizer);
        follower = new Follower(drive, localizer);
        follower.setTrajectory(lol, 2);

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        waitForStart();

        localizer.imu.startIMUThread(this);

        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.startTime();

        while (opModeIsActive() && !isStopRequested()) {
            for (LynxModule hub : hubs)
                hub.clearBulkCache();
            localizer.update();
            follower.update();
            drive.update();
            TelemetryPacket packet = new TelemetryPacket();

            GPose gp = follower.predictiveLocalizer.getPoseEstimate().getGPose();
            packet.fieldOverlay()
                    .strokeDesiredPath(lol.getAllGPoses())
                    .strokeActualPath(localizer.getAllGPoses())
                    .setStroke("#42f54b")
                    .strokeRobot(gp);
            ;
            dash.sendTelemetryPacket(packet);

        }
    }
}