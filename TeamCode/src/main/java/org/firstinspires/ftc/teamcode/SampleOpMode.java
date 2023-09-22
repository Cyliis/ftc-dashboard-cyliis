package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
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
import org.firstinspires.ftc.teamcode.TrajectoryStuff.CubicBezierTangentHeadingTrajectorySegment;
import org.firstinspires.ftc.teamcode.TrajectoryStuff.Trajectory;
import org.firstinspires.ftc.teamcode.TrajectoryStuff.TrajectoryBuilder;
import org.firstinspires.ftc.teamcode.Utils.Pose;

import java.util.List;

@Config
@TeleOp(name = "Sample")
public class SampleOpMode extends LinearOpMode {
    List<LynxModule> hubs;

    MecanumDrive drive;
    Follower follower;
    Localizer localizer;

    FtcDashboard dash;

    CubicBezierTangentHeadingTrajectorySegment segment1 = new CubicBezierTangentHeadingTrajectorySegment(
            new Pose(0, 0, 0),
            new Pose(48, 0, 0),
            new Pose(48, -24, 0),
            new Pose(96, -24, 0)
    );

    Trajectory lol = new TrajectoryBuilder(segment1)
            .build();

    @Override
    public void runOpMode() throws InterruptedException {

        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        drive = new MecanumDrive(hardwareMap, localizer);
        localizer = new Localizer(hardwareMap, drive.frontLeft.motor, hardwareMap.get(DcMotorEx.class, "lift2"), new Pose());
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
            packet.fieldOverlay().strokeDesiredPath(lol.getAllGPoses()).strokeActualPath(localizer.getAllGPoses());


        }
    }
}