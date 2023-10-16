package org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.teamcode.Modules.*;

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

    @Override
    public void initUpdate() {
        intake.initUpdate();
        outtake.initUpdate();
    }

    @Override
    public void atStart() {
        intake.atStart();
        outtake.atStart();
    }

    @Override
    public void update() {
        intake.update();
        outtake.update();
    }

    @Override
    public void emergencyStop() {
        intake.emergencyStop();
        outtake.emergencyStop();
    }
}
