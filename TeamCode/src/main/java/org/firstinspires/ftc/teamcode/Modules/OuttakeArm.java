package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Robot.IStateBasedModule;
import org.firstinspires.ftc.teamcode.Wrappers.CoolServo;

@Config
public class OuttakeArm implements IStateBasedModule, IRobotModule {

    public static boolean ENABLED = false;

    public final CoolServo leftServo, rightServo;
    public static boolean reversedLeftServo = false, reversedRightServo = true;

    public static double intakePosition = 0.5, outtakePosition = 0.5;

    public static double profileMaxVelocity = 1, profileAcceleration = 1, profileDeceleration = 1;

    public enum State{
        INTAKE(intakePosition), GOING_INTAKE(intakePosition), OUTTAKE(outtakePosition), GOING_OUTTAKE(outtakePosition);

        public double position;

        State(double position){
            this.position = position;
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

    public OuttakeArm(Hardware hardware, State initialState){
        leftServo = new CoolServo(hardware.sch0, reversedLeftServo, profileMaxVelocity, profileAcceleration, profileDeceleration, initialState.position);
        rightServo = new CoolServo(hardware.sch0, reversedRightServo, profileMaxVelocity, profileAcceleration, profileDeceleration, initialState.position);
        timer.startTime();
        setState(initialState);
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
        switch (state){
            case GOING_INTAKE:
                if(leftServo.getTimeToMotionEnd() == 0)
                    setState(State.INTAKE);
                break;
            case GOING_OUTTAKE:
                if(leftServo.getTimeToMotionEnd() == 0)
                    setState(State.OUTTAKE);
                break;
        }
    }

    @Override
    public void updateHardware() {
        leftServo.setPosition(state.position);
        rightServo.setPosition(state.position);

        leftServo.update();
        rightServo.update();
    }
}
