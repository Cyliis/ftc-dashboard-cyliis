package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.Follower;
import org.firstinspires.ftc.teamcode.Modules.Localizer;
import org.firstinspires.ftc.teamcode.Modules.MecanumDrive;
import org.firstinspires.ftc.teamcode.TrajectoryStuff.CubicBezierTrajectorySegment;
import org.firstinspires.ftc.teamcode.TrajectoryStuff.Trajectory;
import org.firstinspires.ftc.teamcode.TrajectoryStuff.TrajectoryBuilder;
import org.firstinspires.ftc.teamcode.Utils.Pose;

@Config
@Autonomous(name = "Correction PID Tuning")
public class CorrectionPIDTuning extends LinearOpMode {

    MecanumDrive drive;
    Localizer localizer;

    FtcDashboard dash;

    @Override
    public void runOpMode() throws InterruptedException {

        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        localizer = new Localizer(hardwareMap, new Pose());
        drive = new MecanumDrive(hardwareMap, localizer, MecanumDrive.RunMode.PID);

        waitForStart();

        localizer.imu.startIMUThread(this);

        drive.setTargetPose(new Pose());

        while(opModeIsActive() && !isStopRequested()){
            localizer.update();
            drive.update();

            telemetry.addData("Pose X", localizer.getPoseEstimate().getX());
            telemetry.addData("Pose Y", localizer.getPoseEstimate().getY());
            telemetry.addData("Pose Heading", localizer.getPoseEstimate().getHeading());
            telemetry.update();
        }
    }
}
