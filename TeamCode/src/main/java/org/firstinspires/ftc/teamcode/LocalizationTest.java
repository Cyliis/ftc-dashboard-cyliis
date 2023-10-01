package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.Follower;
import org.firstinspires.ftc.teamcode.Modules.Localizer;
import org.firstinspires.ftc.teamcode.Modules.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.TrajectoryStuff.CubicBezierTrajectorySegment;
import org.firstinspires.ftc.teamcode.TrajectoryStuff.Trajectory;
import org.firstinspires.ftc.teamcode.TrajectoryStuff.TrajectoryBuilder;
import org.firstinspires.ftc.teamcode.Utils.Pose;
import org.firstinspires.ftc.teamcode.Utils.Vector;

@Config
@TeleOp(name = "Localization Test Buru")
public class LocalizationTest extends LinearOpMode {

    Hardware hardware;

    MecanumDrive drive;
    Localizer localizer;

    FtcDashboard dash;

    @Override
    public void runOpMode() throws InterruptedException {

        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        hardware = new Hardware(hardwareMap);

        localizer = new Localizer(hardware, new Pose());
        drive = new MecanumDrive(hardware, localizer);

        waitForStart();

        localizer.imu.startIMUThread(this);

        while(opModeIsActive() && !isStopRequested()){

            drive.setTargetVector(new Vector(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.left_trigger - gamepad1.right_trigger));

            localizer.update();
            drive.update();

            telemetry.addData("Pose X", localizer.getPoseEstimate().getX());
            telemetry.addData("Pose Y", localizer.getPoseEstimate().getY());
            telemetry.addData("Pose Heading", localizer.getPoseEstimate().getHeading());
            telemetry.addData("velocity", drive.getLocalizer().getVelocity());
            telemetry.addData("predicted glide X", drive.getLocalizer().glideDelta.getX());
            telemetry.addData("predicted glide Y", drive.getLocalizer().glideDelta.getY());
            telemetry.update();
        }
    }
}
