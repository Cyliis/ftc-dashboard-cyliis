package org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.*;
import org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import java.util.ArrayList;

public class RobotModules implements IRobotModule {

    public ActiveIntake activeIntake;
    public Intake intake;
    public LeftGripper leftGripper;
    public Lift lift;
    public Outtake outtake;
    public OuttakeArm outtakeArm;
    public Pitch pitch;
    public RightGripper rightGripper;

    public RobotModules(Hardware hardware){
        activeIntake = new ActiveIntake(hardware, ActiveIntake.State.IDLE);
        leftGripper = new LeftGripper(hardware, LeftGripper.State.CLOSED);
        lift = new Lift(hardware, Lift.State.GOING_DOWN);
        outtakeArm = new OuttakeArm(hardware, OuttakeArm.State.INTAKE);
        pitch = new Pitch(hardware, Pitch.State.INTAKE);
        rightGripper = new RightGripper(hardware, RightGripper.State.CLOSED);

        intake = new Intake(activeIntake, leftGripper, rightGripper, Intake.State.IDLE);
        outtake = new Outtake(lift, outtakeArm, pitch, Outtake.State.DOWN);
    }

    public void telemetry(Telemetry telemetry){
        telemetry.addData("Lift level", Lift.level);
        if(lift.encoder!=null) telemetry.addData("Lift current position", outtake.lift.encoder.getCurrentPosition());
        telemetry.addData("Lift current state", lift.getState());
        if(lift.encoder!=null) telemetry.addData("Lift ground", Lift.groundPos);
        telemetry.addData("Intake state", intake.getState());
        telemetry.addData("Outtake state", outtake.getState());
        telemetry.addData("Outtake arm state", outtakeArm.getState());
        if(outtakeArm.leftServo != null) telemetry.addData("Outtake arm position", outtakeArm.leftServo.cachedPosition);
        if(outtakeArm.leftServo != null) telemetry.addData("Outtake arm time to motion end", outtakeArm.leftServo.getTimeToMotionEnd());
        telemetry.addData("Outtake arm target position", outtakeArm.getState().position);
    }

    @Override
    public void initUpdate() {
        if(Intake.ENABLED) intake.initUpdate();
        if(Outtake.ENABLED) outtake.initUpdate();
    }

    @Override
    public void atStart() {
        if(Intake.ENABLED) intake.atStart();
        if(Outtake.ENABLED) outtake.atStart();
    }

    @Override
    public void update() {
        if(Intake.ENABLED) intake.update();
        if(Outtake.ENABLED) outtake.update();
    }

    @Override
    public void emergencyStop() {
        if(Intake.ENABLED) intake.emergencyStop();
        if(Outtake.ENABLED) outtake.emergencyStop();
    }
}
