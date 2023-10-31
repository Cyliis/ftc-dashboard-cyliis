package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Localizer;
import org.firstinspires.ftc.teamcode.Modules.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.GampadControllers.BuruDriveTrainControl;
import org.firstinspires.ftc.teamcode.Robot.GampadControllers.BuruSebiGamepadControl;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.RobotModules;
import org.firstinspires.ftc.teamcode.Utils.Pose;

import java.util.ArrayList;
import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp 🐟")
public class TeleOp extends LinearOpMode {

    List<LynxModule> hubs;

    FtcDashboard dash;

    Hardware hardware;

    MecanumDrive drive;
    Localizer localizer;
    RobotModules robotModules;

    BuruSebiGamepadControl gamepadControl;
    BuruDriveTrainControl driveTrainControl;

    @Override
    public void runOpMode() throws InterruptedException {

        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        hardware = new Hardware(hardwareMap);

        localizer = new Localizer(hardware,  new Pose());
        drive = new MecanumDrive(hardware, null, MecanumDrive.RunMode.Vector);
        drive.setLocalizer(localizer);

        robotModules = new RobotModules(hardware);

        gamepadControl = new BuruSebiGamepadControl(robotModules, gamepad1, gamepad2);
        driveTrainControl = new BuruDriveTrainControl(gamepad1, drive);

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

            gamepadControl.update();
            driveTrainControl.update();

            localizer.update();
            drive.update();

            robotModules.update();

            robotModules.telemetry(telemetry);

            telemetry.addData("Imu angle", drive.getLocalizer().getPoseEstimate().getHeading());
            telemetry.addData("Hz", 1.0/loopTimer.seconds());
            loopTimer.reset();

            telemetry.update();
        }
    }
}
