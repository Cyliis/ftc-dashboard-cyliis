package org.firstinspires.ftc.teamcode.Robot.GampadControllers;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Modules.*;
import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Robot.RobotModules;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;
import org.opencv.core.Mat;

public class BuruSebiGamepadControl implements IRobotModule {

    private final RobotModules robotModules;
    private final StickyGamepad stickyGamepad1, stickyGamepad2;
    private final Gamepad gamepad1, gamepad2;

    public BuruSebiGamepadControl(RobotModules robotModules, Gamepad gamepad1, Gamepad gamepad2){
        this.robotModules = robotModules;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.stickyGamepad1 = new StickyGamepad(gamepad1);
        this.stickyGamepad2 = new StickyGamepad(gamepad2);
    }

    public static double triggerThreshold = 0.1;

    private void intakeControl(){
        if(robotModules.outtake.getState()!= Outtake.State.DOWN) return;
        if(gamepad2.right_trigger > triggerThreshold && (robotModules.intake.getState() != Intake.State.INTAKE && robotModules.intake.getState() != Intake.State.OPENING_GRIPPERS_FOR_INTAKE)) {
            robotModules.intake.setState(Intake.State.OPENING_GRIPPERS_FOR_INTAKE);
        }
        if(gamepad2.right_trigger < triggerThreshold && (robotModules.intake.getState() == Intake.State.INTAKE || robotModules.intake.getState() == Intake.State.OPENING_GRIPPERS_FOR_INTAKE)){
            robotModules.intake.setState(Intake.State.STOP_INTAKE);
        }
        if(gamepad2.left_trigger > triggerThreshold && (robotModules.intake.getState() == Intake.State.INTAKE || robotModules.intake.getState() == Intake.State.OPENING_GRIPPERS_FOR_INTAKE)){
            robotModules.intake.setState(Intake.State.STOP_INTAKE_THEN_REVERSE);
        }
        if(gamepad2.left_trigger > triggerThreshold && (robotModules.intake.getState() != Intake.State.INTAKE && robotModules.intake.getState() != Intake.State.OPENING_GRIPPERS_FOR_INTAKE)){
            robotModules.intake.setState(Intake.State.REVERSE);
        }
        if(gamepad2.left_trigger < triggerThreshold && robotModules.intake.getState() == Intake.State.REVERSE){
            robotModules.intake.setState(Intake.State.IDLE);
        }
    }

    public void outtakeControl(){
        if(robotModules.intake.getState() != Intake.State.IDLE) return;
        if(stickyGamepad2.x){
            if(robotModules.outtake.getState() == Outtake.State.UP) robotModules.outtake.setState(Outtake.State.GOING_DOWN);
            else if(robotModules.outtake.getState() == Outtake.State.DOWN) robotModules.outtake.setState(Outtake.State.GOING_UP);
        }
        if(stickyGamepad2.dpad_up){
            Lift.level = Math.min(8, Lift.level + 1);
            if(robotModules.outtake.getState() == Outtake.State.UP || robotModules.outtake.getState() == Outtake.State.LIFT_CHANGING_SCORING_POSITION) robotModules.outtake.setState(Outtake.State.LIFT_CHANGING_SCORING_POSITION);
        }
        if(stickyGamepad2.dpad_down){
            Lift.level = Math.max(0, Lift.level - 1);
            if(robotModules.outtake.getState() == Outtake.State.UP || robotModules.outtake.getState() == Outtake.State.LIFT_CHANGING_SCORING_POSITION) robotModules.outtake.setState(Outtake.State.LIFT_CHANGING_SCORING_POSITION);
        }
    }

    public void grippersControl(){
        if(robotModules.outtake.getState() != Outtake.State.UP && robotModules.outtake.getState() != Outtake.State.LIFT_GOING_UP) return;
        if(robotModules.leftGripper.getState() == LeftGripper.State.CLOSED && stickyGamepad1.x) robotModules.leftGripper.setState(LeftGripper.State.OPENING);
        if(robotModules.rightGripper.getState() == RightGripper.State.CLOSED && stickyGamepad1.x) robotModules.rightGripper.setState(RightGripper.State.OPENING);
    }

    @Override
    public void update() {
        stickyGamepad1.update();
        stickyGamepad2.update();

        intakeControl();
        outtakeControl();
        grippersControl();
    }
}
