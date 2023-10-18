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

    public static boolean ENABLED = true;

    private final CoolMotor leftMotor, rightMotor;
    public static boolean leftMotorReversed = false, rightMotorReversed = true;
    public final CoolEncoder encoder;
    public static boolean encoderReversed = false;

    public static int groundPos = 0, firstLevel = 20, increment = 70, level = 0, positionThresh = 4, passthroughPosition = 250;

    public static double resetPower = -0.5, resetVelocityThresh = 1;

    public static PIDCoefficients pid = new PIDCoefficients(0.013,0.16,0.0006);
    public static double ff1 = 0.1, ff2 = 0.0003;

    public enum State{
        DOWN(groundPos), RESETTING(groundPos, DOWN), GOING_DOWN(groundPos, RESETTING), PASSTHROUGH(passthroughPosition), GOING_PASSTHROUGH(passthroughPosition, PASSTHROUGH),
        UP(groundPos + firstLevel + increment * level), GOING_UP(groundPos + firstLevel + increment * level, UP);

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

    public void setState(State newState){
        if(state == newState) return;
        this.state = newState;
    }

    private void updateStateValues(){
        State.DOWN.position = groundPos;
        State.GOING_DOWN.position = groundPos;
        State.UP.position = groundPos + firstLevel + increment * (level - 1);
        State.GOING_UP.position = groundPos + firstLevel + increment * (level - 1);
    }

    public Lift(Hardware hardware, State initialState){
        if(!ENABLED) leftMotor = null;
        else leftMotor = new CoolMotor(hardware.meh3, CoolMotor.RunMode.PID, leftMotorReversed);
        if(!ENABLED) rightMotor = null;
        else rightMotor = new CoolMotor(hardware.meh2, CoolMotor.RunMode.PID, rightMotorReversed);

        if(!ENABLED) encoder = null;
        else encoder = new CoolEncoder(hardware.mch1, encoderReversed);

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
            leftMotor.setPIDF(pid, ff1 + ff2 * (double)(state.position));
            rightMotor.setPIDF(pid, ff1 + ff2 * (double)(state.position));
            leftMotor.calculatePower(encoder.getCurrentPosition(), state.position);
            rightMotor.calculatePower(encoder.getCurrentPosition(), state.position);
        }

        leftMotor.update();
        rightMotor.update();
    }
}
