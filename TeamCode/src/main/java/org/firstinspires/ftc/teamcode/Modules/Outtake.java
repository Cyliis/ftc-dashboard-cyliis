package org.firstinspires.ftc.teamcode.Modules;

import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Robot.IStateBasedModule;

public class Outtake implements IStateBasedModule, IRobotModule {

    public static boolean ENABLED = true;

    public enum State{
        DOWN, LIFT_GOING_DOWN, ARM_GOING_BACK, GOING_DOWN, LIFT_GOING_PASSTHROUGH_UP, LIFT_GOING_PASSTHROUGH_DOWN,
        UP, LIFT_GOING_UP, ARM_GOING_OUT, GOING_UP, ARM_TO_INTAKE, ARM_LIFTING_UP
    }

    State state;

    public void setState(State newState){
        if(state == newState) return;

        state = newState;

        switch (newState){
            case LIFT_GOING_DOWN:
                lift.setState(Lift.State.GOING_DOWN);
                break;
            case ARM_GOING_BACK:
                arm.setState(OuttakeArm.State.GOING_PASSTHROUGH);
                pitch.setState(Pitch.State.GOING_INTAKE);
                break;
            case GOING_DOWN:
                setState(State.LIFT_GOING_PASSTHROUGH_DOWN);
                break;
            case LIFT_GOING_UP:
                lift.setState(Lift.State.GOING_UP);
                break;
            case ARM_GOING_OUT:
                arm.setState(OuttakeArm.State.GOING_OUTTAKE);
                pitch.setState(Pitch.State.GOING_OUTTAKE);
                break;
            case GOING_UP:
                setState(State.ARM_LIFTING_UP);
                break;
            case ARM_LIFTING_UP:
                arm.setState(OuttakeArm.State.PASSTHROUGH);
                break;
            case ARM_TO_INTAKE:
                arm.setState(OuttakeArm.State.GOING_INTAKE);
                break;
            case LIFT_GOING_PASSTHROUGH_UP:
            case LIFT_GOING_PASSTHROUGH_DOWN:
                lift.setState(Lift.State.GOING_PASSTHROUGH);
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
            case GOING_UP:
                if(arm.getState() == OuttakeArm.State.PASSTHROUGH)
                    setState(State.ARM_GOING_OUT);
                break;
            case LIFT_GOING_DOWN:
                if(lift.getState() == Lift.State.DOWN)
                    setState(State.DOWN);
                break;
            case ARM_GOING_BACK:
                if(arm.getState() == OuttakeArm.State.PASSTHROUGH)
                    setState(State.LIFT_GOING_DOWN);
                break;
            case LIFT_GOING_UP:
                if(lift.getState() == Lift.State.UP)
                    setState(State.UP);
                break;
            case ARM_GOING_OUT:
                if(arm.getState() == OuttakeArm.State.OUTTAKE)
                    setState(State.LIFT_GOING_UP);
                break;
            case ARM_TO_INTAKE:
                if(arm.getState() == OuttakeArm.State.INTAKE)
                    setState(State.DOWN);
                break;
            case ARM_LIFTING_UP:
                if(arm.getState() == OuttakeArm.State.PASSTHROUGH)
                    setState(State.LIFT_GOING_PASSTHROUGH_UP);
                break;
            case LIFT_GOING_PASSTHROUGH_UP:
                if(lift.getState() == Lift.State.PASSTHROUGH)
                    setState(State.ARM_GOING_OUT);
                break;
            case LIFT_GOING_PASSTHROUGH_DOWN:
                if(lift.getState() == Lift.State.PASSTHROUGH)
                    setState(State.ARM_GOING_BACK);
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
