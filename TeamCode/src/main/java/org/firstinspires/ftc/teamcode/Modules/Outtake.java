package org.firstinspires.ftc.teamcode.Modules;

import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Robot.IStateBasedModule;

public class Outtake implements IStateBasedModule, IRobotModule {

    public static boolean ENABLED = true;

    public enum State{
        GOING_UP, ARM_LIFTING_UP, LIFT_GOING_UP, ARM_GOING_OUT, LIFT_TO_SCORE, UP,
        GOING_DOWN, LIFT_GOING_PASSTHROUGH, ARM_GOING_BACK, LIFT_GOING_DOWN, ARM_TO_INTAKE, DOWN,
        LIFT_CHANGING_SCORING_POSITION
    }

    State state;

    public void setState(State newState){
        if(state == newState) return;

        state = newState;

        switch (newState){
            case LIFT_CHANGING_SCORING_POSITION:
                lift.setState(Lift.State.GOING_UP);
                break;
            case GOING_UP:
                setState(State.ARM_LIFTING_UP);
                break;
            case ARM_LIFTING_UP:
                arm.setState(OuttakeArm.State.GOING_PASSTHROUGH);
                break;
            case LIFT_GOING_UP:
                pitch.setState(Pitch.State.GOING_OUTTAKE);
                if(Lift.State.UP.position >= Lift.State.PASSTHROUGH.position) lift.setState(Lift.State.GOING_UP);
                else lift.setState(Lift.State.GOING_PASSTHROUGH);
                break;
            case ARM_GOING_OUT:
                arm.setState(OuttakeArm.State.GOING_OUTTAKE);
                break;
            case LIFT_TO_SCORE:
                if(lift.getState() != Lift.State.UP) lift.setState(Lift.State.GOING_UP);
                else setState(State.UP);
                break;
            case GOING_DOWN:
                if(lift.encoder.getCurrentPosition() <= Lift.State.PASSTHROUGH.position) setState(State.LIFT_GOING_PASSTHROUGH);
                else setState(State.ARM_GOING_BACK);
                break;
            case LIFT_GOING_PASSTHROUGH:
                lift.setState(Lift.State.GOING_PASSTHROUGH);
                break;
            case ARM_GOING_BACK:
                arm.setState(OuttakeArm.State.GOING_PASSTHROUGH);
                pitch.setState(Pitch.State.GOING_INTAKE);
                break;
            case LIFT_GOING_DOWN:
                lift.setState(Lift.State.GOING_DOWN);
                break;
            case ARM_TO_INTAKE:
                arm.setState(OuttakeArm.State.GOING_INTAKE);
                break;
        }
    }

    public State getState(){
        return state;
    }

    public final Lift lift;
    public final OuttakeArm arm;
    public final Pitch pitch;

    public Outtake(Lift lift, OuttakeArm arm, Pitch pitch, State initialState){
        this.lift = lift;
        this.arm = arm;
        this.pitch = pitch;
        this.state = initialState;
    }

    @Override
    public void update() {
        if(!ENABLED) return;

        updateState();
        updateHardware();
    }

    @Override
    public void updateState() {
        switch (state){
            case LIFT_CHANGING_SCORING_POSITION:
            case LIFT_TO_SCORE:
                if(lift.getState() == Lift.State.UP)
                    setState(State.UP);
                break;
            case ARM_LIFTING_UP:
                if(arm.getState() == OuttakeArm.State.PASSTHROUGH)
                    setState(State.LIFT_GOING_UP);
                break;
            case LIFT_GOING_UP:
                arm.predictiveProfile.setMotion(arm.leftServo.profile.getPosition(), OuttakeArm.State.GOING_OUTTAKE.position, arm.leftServo.profile.getSignedVelocity());
                if(lift.profile.getTimeTo(Lift.State.PASSTHROUGH.position) <= arm.predictiveProfile.getTimeTo(OuttakeArm.State.VERTICAL.position))
                    setState(State.ARM_GOING_OUT);
                break;
            case ARM_GOING_OUT:
                if(arm.getState() == OuttakeArm.State.OUTTAKE)
                    setState(State.LIFT_TO_SCORE);
                break;
            case LIFT_GOING_PASSTHROUGH:
                arm.predictiveProfile.setMotion(arm.leftServo.profile.getPosition(), OuttakeArm.State.PASSTHROUGH.position, arm.leftServo.profile.getSignedVelocity());
                if(lift.profile.getTimeToMotionEnd() <= arm.predictiveProfile.getTimeTo(OuttakeArm.State.VERTICAL.position))
                    setState(State.ARM_GOING_BACK);
                break;
            case ARM_GOING_BACK:
                lift.predictiveProfile.setMotion(lift.profile.getPosition(), Lift.State.DOWN.position, lift.profile.getSignedVelocity());
                if(arm.leftServo.getTimeToMotionEnd() <= lift.predictiveProfile.getTimeToMotionEnd())
                    setState(State.LIFT_GOING_DOWN);
                break;
            case LIFT_GOING_DOWN:
                if(lift.getState() == Lift.State.DOWN)
                    setState(State.ARM_TO_INTAKE);
                break;
            case ARM_TO_INTAKE:
                if(arm.getState() == OuttakeArm.State.INTAKE)
                    setState(State.DOWN);
                break;
        }
    }

    @Override
    public void updateHardware() {
        lift.update();
        arm.update();
        pitch.update();
    }
}
