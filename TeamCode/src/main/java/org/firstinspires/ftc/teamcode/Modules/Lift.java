package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.checkerframework.checker.signedness.qual.Constant;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Robot.IStateBasedModule;
import org.firstinspires.ftc.teamcode.Wrappers.CoolEncoder;
import org.firstinspires.ftc.teamcode.Wrappers.CoolMotor;

@Config
public class Lift implements IStateBasedModule, IRobotModule {

    CoolMotor leftMotor, rightMotor;
    public static boolean leftMotorReversed = false, rightMotorReversed = true;
    CoolEncoder encoder;
    public static boolean encoderReversed = false;

    public static int groundPos = 0, firstLevel = 50, increment = 50, level = 0, positionThresh = 2;

    public static double resetPower = -0.5, resetVelocityThresh = 1;

    public static PIDCoefficients pid = new PIDCoefficients(0,0,0);
    public static double ff1 = 0, ff2 = 0;

    public enum State{
        DOWN(groundPos), RESETTING(groundPos, DOWN), GOING_DOWN(groundPos, RESETTING),
        UP(groundPos + firstLevel + increment * (level - 1)), GOING_UP(groundPos + firstLevel + increment * (level - 1), UP);

        public int position;
        public final State nextState;

        State(int position){
            this.position = position;
            this.nextState = this;
        }

        State(int position, State nextState){
            this.position = position;
            this.nextState = nextState;
        }
    }

    private State state;

    public State getState(){
        return state;
    }

    public void setState(State state){
        this.state = state;
    }

    private void updateStateValues(){
        State.DOWN.position = groundPos;
        State.GOING_DOWN.position = groundPos;
        State.UP.position = groundPos + firstLevel + increment * (level - 1);
        State.GOING_UP.position = groundPos + firstLevel + increment * (level - 1);
    }

    public Lift(Hardware hardware, State initialState){
        leftMotor = new CoolMotor(hardware.meh3, CoolMotor.RunMode.PID, leftMotorReversed);
        rightMotor = new CoolMotor(hardware.meh2, CoolMotor.RunMode.PID, rightMotorReversed);

        encoder = new CoolEncoder(hardware.meh3, encoderReversed);

        this.state = initialState;
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
        if(state == State.RESETTING){
            if(Math.abs(encoder.getVelocity()) <= resetVelocityThresh){
                groundPos = encoder.getCurrentPosition();
                state = state.nextState;
            }
        }
        else {
            if(Math.abs(state.position - encoder.getCurrentPosition()) <= positionThresh)
                state = state.nextState;
        }
    }

    @Override
    public void updateHardware() {
        if(state == State.RESETTING){
            leftMotor.setMode(CoolMotor.RunMode.RUN);
            rightMotor.setMode(CoolMotor.RunMode.RUN);
            leftMotor.setPower(resetPower);
            rightMotor.setPower(resetPower);
        }else{
            leftMotor.setMode(CoolMotor.RunMode.PID);
            rightMotor.setMode(CoolMotor.RunMode.PID);
            leftMotor.setPIDF(pid, ff1 + ff2 * (double)(encoder.getCurrentPosition() - groundPos));
            rightMotor.setPIDF(pid, ff1 + ff2 * (double)(encoder.getCurrentPosition() - groundPos));
            leftMotor.calculatePower(encoder.getCurrentPosition(), state.position);
            rightMotor.calculatePower(encoder.getCurrentPosition(), state.position);
        }

        leftMotor.update();
        rightMotor.update();
    }
}
