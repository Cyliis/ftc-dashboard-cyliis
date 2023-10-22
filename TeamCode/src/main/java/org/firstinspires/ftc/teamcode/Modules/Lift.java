package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Math.AsymmetricMotionProfile;
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

    public static int groundPos = 0, firstLevel = 140, increment = 69, level = 0, positionThresh = 4, passthroughPosition = 300;

    public static double resetPower = -0.05, velocityThreshold = 0;

    public static PIDCoefficients pid = new PIDCoefficients(0.015,0.24,0.0003);
    public static double ff1 = 0.14, ff2 = 0.0002;

    public static double maxVelocity = 12000, acceleration = 8000, deceleration = 2500;
    public AsymmetricMotionProfile profile = new AsymmetricMotionProfile(maxVelocity, acceleration, deceleration);
    public AsymmetricMotionProfile predictiveProfile = new AsymmetricMotionProfile(maxVelocity, acceleration, deceleration);

    public static double outtakeArmPosition = OuttakeArm.State.INTAKE.position;
    private final double angleMultiplier = Math.toRadians(270);

    public static double armLength = 260;
    public static double armFloor = 40;

    double passthroughPosition(double armPosition){
        double outtakeArmAngle = (armPosition - OuttakeArm.State.VERTICAL.position) * angleMultiplier;
        if(outtakeArmAngle <= 0){
            return groundPos + Math.cos(outtakeArmAngle) * armLength + armFloor;
        }
        return groundPos + Math.min(passthroughPosition(0), Math.max(state.position, groundPos + Math.cos(outtakeArmAngle) * armLength + armFloor));
    }

    public enum State{
        DOWN(groundPos), RESETTING(groundPos, DOWN), GOING_DOWN(groundPos, RESETTING), PASSTHROUGH(passthroughPosition), GOING_PASSTHROUGH(passthroughPosition, PASSTHROUGH),
        UP(groundPos + firstLevel + increment * level), GOING_UP(groundPos + firstLevel + increment * level, UP), ADAPTABLE_PASSTHROUGH(groundPos);

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
        updateStateValues();
        profile.setMotion(encoder.getCurrentPosition(), newState.position, encoder.getVelocity());
        if(state == newState) return;
        this.state = newState;
    }

    private void updateStateValues(){

        State.DOWN.position = groundPos;
        State.GOING_DOWN.position = groundPos;
        State.UP.position = groundPos + firstLevel + increment * level;
        State.GOING_UP.position = groundPos + firstLevel + increment * level;
        State.GOING_PASSTHROUGH.position = (int)passthroughPosition(OuttakeArm.State.VERTICAL.position);
        State.PASSTHROUGH.position = (int)passthroughPosition(OuttakeArm.State.VERTICAL.position);
        State.ADAPTABLE_PASSTHROUGH.position = (int)passthroughPosition(outtakeArmPosition);
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
            if(Math.abs(encoder.getVelocity()) <= velocityThreshold){
                groundPos = encoder.getCurrentPosition();
                state = state.nextState;
            }
        }
        else if(Math.abs(state.nextState.nextState.position - encoder.getCurrentPosition()) <= positionThresh/* && encoder.getVelocity() <= velocityThreshold*/)
            state = state.nextState;
    }

    public int target = 0;

    @Override
    public void updateHardware() {
        if(profile.finalPosition != state.position) profile.setMotion(profile.getPosition(), state.position, profile.getSignedVelocity());

        profile.update();

        if(state == State.RESETTING){
            leftMotor.setMode(CoolMotor.RunMode.RUN);
            rightMotor.setMode(CoolMotor.RunMode.RUN);
            leftMotor.setPower(resetPower);
            rightMotor.setPower(resetPower);
        }else{
            target = (int)profile.getPosition();

            leftMotor.setMode(CoolMotor.RunMode.PID);
            rightMotor.setMode(CoolMotor.RunMode.PID);
            leftMotor.setPIDF(pid, ff1 + ff2 * (double)(target));
            rightMotor.setPIDF(pid, ff1 + ff2 * (double)(target));
            leftMotor.calculatePower(encoder.getCurrentPosition(), target);
            rightMotor.calculatePower(encoder.getCurrentPosition(), target);
        }
        leftMotor.update();
        rightMotor.update();
    }
}
