package org.firstinspires.ftc.teamcode.Modules;

import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Robot.IStateBasedModule;

public class Outtake implements IStateBasedModule, IRobotModule {

    public static boolean ENABLED = false;

    public enum State{
        DOWN, LIFT_GOING_DOWN, ARM_GOING_BACK, GOING_DOWN,
        UP, LIFT_GOING_UP, ARM_GOING_OUT, GOING_UP
    }

    State state;

    public void setState(State newState){
        if(state == newState) return;

        switch (state){
            case LIFT_GOING_DOWN:
                lift.setState(Lift.State.GOING_DOWN);
                break;
            case ARM_GOING_BACK:
                lift.setState(Lift.State.GOING_PASSTHROUGH);
                arm.setState(OuttakeArm.State.GOING_INTAKE);
                pitch.setState(Pitch.State.GOING_INTAKE);
                break;
            case GOING_DOWN:
                setState(State.ARM_GOING_BACK);
                break;
            case LIFT_GOING_UP:
                lift.setState(Lift.State.GOING_UP);
                break;
            case ARM_GOING_OUT:
                lift.setState(Lift.State.GOING_PASSTHROUGH);
                arm.setState(OuttakeArm.State.GOING_OUTTAKE);
                pitch.setState(Pitch.State.GOING_OUTTAKE);
                break;
            case GOING_UP:
                setState(State.ARM_GOING_OUT);
                break;
        }

        state = newState;
    }

    public State getState(){
        return state;
    }

    Lift lift;
    OuttakeArm arm;
    Pitch pitch;

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
            case LIFT_GOING_DOWN:
                if(lift.getState() == Lift.State.DOWN)
                    setState(State.DOWN);
                break;
            case ARM_GOING_BACK:
                if(arm.getState() == OuttakeArm.State.INTAKE)
                    setState(State.LIFT_GOING_DOWN);
                break;
            case LIFT_GOING_UP:
                if(lift.getState() == Lift.State.UP)
                    setState(State.UP);
                break;
            case ARM_GOING_OUT:
                if(arm.getState() == OuttakeArm.State.OUTTAKE)
                    setState(State.GOING_UP);
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
