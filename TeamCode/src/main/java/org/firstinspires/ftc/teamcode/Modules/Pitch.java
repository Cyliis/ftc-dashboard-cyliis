package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Robot.IStateBasedModule;
import org.firstinspires.ftc.teamcode.Wrappers.CoolServo;

@Config
public class Pitch implements IStateBasedModule, IRobotModule {

    public static boolean ENABLED = false;

    private final CoolServo servo;
    public static boolean reversedServo = false;

    public static double intakePosition = 0.5, outtakePosition = 0.5;

    public static double profileMaxVelocity = 1, profileAcceleration = 1;

    public enum State{
        INTAKE(intakePosition), GOING_INTAKE(intakePosition, INTAKE), OUTTAKE(outtakePosition), GOING_OUTTAKE(outtakePosition, OUTTAKE);

        public double position;
        public final State nextState;

        State(double position){
            this.position = position;
            this.nextState = this;
        }

        State(double position, State nextState){
            this.position = position;
            this.nextState = nextState;
        }
    }

    private void updateStateValues(){
        State.INTAKE.position = intakePosition;
        State.GOING_INTAKE.position = intakePosition;
        State.OUTTAKE.position = outtakePosition;
        State.GOING_OUTTAKE.position = outtakePosition;
    }

    private State state;

    private final ElapsedTime timer = new ElapsedTime();

    public State getState(){
        return state;
    }

    public void setState(State newState){
        if(newState == state) return;
        this.state = newState;
        timer.reset();
    }

    public Pitch(Hardware hardware, State initialState){
        servo = new CoolServo(hardware.sch0, reversedServo, profileMaxVelocity, profileAcceleration, initialState.position);
        timer.startTime();
        setState(initialState);
    }

    @Override
    public void initUpdate() {
        update();
    }

    @Override
    public void update() {
        if(!ENABLED) return;

        updateStateValues();
        updateState();
        updateHardware();
    }

    @Override
    public void updateState() {
        if(servo.getTimeToMotionEnd() == 0) state = state.nextState;
    }

    @Override
    public void updateHardware() {
        servo.setPosition(state.position);

        servo.update();
    }
}
