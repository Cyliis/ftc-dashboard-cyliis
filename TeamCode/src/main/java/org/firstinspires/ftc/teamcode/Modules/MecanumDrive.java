package org.firstinspires.ftc.teamcode.Modules;

import static java.lang.Math.PI;
import static java.lang.Math.pow;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Wrappers.CoolMotor;
import org.firstinspires.ftc.teamcode.Utils.IRobotModule;
import org.firstinspires.ftc.teamcode.Utils.Pose;
import org.firstinspires.ftc.teamcode.Utils.Vector;

@Config
public class MecanumDrive implements IRobotModule {

    private PredictiveLocalizer localizer;

    public final CoolMotor frontLeft, frontRight, backLeft, backRight;
    public static String frontLeftMotorName = "mfl", frontRightMotorName = "mfr",
            backLeftMotorName = "mbl", backRightMotorName = "mbr";
    public static boolean frontLeftMotorReversed = false, frontRightMotorReversed = true, backLeftMotorReversed = false, backRightMotorReversed = true;

    public static PIDCoefficients translationalPID = new PIDCoefficients(0.15,0.05,0.02),
            headingPID = new PIDCoefficients(2,0.3,0.2);
    public final PIDController tpid= new PIDController(0,0,0), hpid = new PIDController(1.5,0.2,0.05);

    public static double lateralMultiplier = 1.2;

    public enum RunMode{
        PID, Vector
    }

    private RunMode runMode;

    public MecanumDrive(HardwareMap hm, Localizer localizer, RunMode runMode){
        this.localizer = new PredictiveLocalizer(localizer);
        frontLeft = new CoolMotor(hm, frontLeftMotorName, CoolMotor.RunMode.RUN, frontLeftMotorReversed);
        frontRight = new CoolMotor(hm, frontRightMotorName, CoolMotor.RunMode.RUN, frontRightMotorReversed);
        backLeft = new CoolMotor(hm, backLeftMotorName, CoolMotor.RunMode.RUN, backLeftMotorReversed);
        backRight = new CoolMotor(hm, backRightMotorName, CoolMotor.RunMode.RUN, backRightMotorReversed);

        frontLeft.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.FLOAT);

        this.runMode = runMode;
    }

    public MecanumDrive(HardwareMap hm, Localizer localizer){
        this(hm, localizer, RunMode.Vector);
    }

    public void setLocalizer(Localizer localizer){
        this.localizer = new PredictiveLocalizer(localizer);
    }

    public Vector powerVector = new Vector();
    private Pose targetPose = new Pose();
    public Vector targetVector = new Vector();

    public void setTargetPose(Pose pose){
        this.targetPose = pose;
    }

    public void setTargetVector(Vector Vector){
        this.targetVector = Vector;
    }

    public RunMode getRunMode() {
        return runMode;
    }

    public PredictiveLocalizer getLocalizer(){
        return localizer;
    }

    public void setRunMode(RunMode runMode){
        this.runMode = runMode;
    }

    private void updatePowerVector(){
        switch (runMode){
            case Vector:
                powerVector = new Vector(targetVector.getX(), targetVector.getY() * lateralMultiplier, targetVector.getZ());
                powerVector = Vector.rotateBy(powerVector, localizer.getPoseEstimate().getHeading());
                powerVector = new Vector(powerVector.getX(), powerVector.getY(), targetVector.getZ());
                break;
            case PID:
                Pose currentPose = localizer.getPoseEstimate();

                double xDiff = targetPose.getX() - currentPose.getX();
                double yDiff = targetPose.getY() - currentPose.getY();

                double distance = Math.sqrt(xDiff * xDiff + yDiff * yDiff);

                tpid.setPID(translationalPID.p, translationalPID.i, translationalPID.d);

                double translationalPower = tpid.calculate(-distance, 0);

                powerVector = new Vector(translationalPower * Math.cos(Math.atan2(yDiff, xDiff)), translationalPower * Math.sin(Math.atan2(yDiff, xDiff)));
                powerVector = Vector.rotateBy(powerVector, currentPose.getHeading());

                double headingDiff = (targetPose.getHeading() - currentPose.getHeading()) % (2*PI);

                if(headingDiff > PI) headingDiff -= 2.0*PI;
                if(headingDiff < -PI) headingDiff += 2.0*PI;

                hpid.setPID(headingPID.p, headingPID.i, headingPID.d);

                double headingPower = hpid.calculate(-headingDiff, 0);

                powerVector= new Vector(powerVector.getX(),powerVector.getY(),headingPower);
                break;
        }
        if(Math.abs(powerVector.getX()) + Math.abs(powerVector.getY()) + Math.abs(powerVector.getZ()) > 1)
            powerVector.scaleToMagnitude(1);
    }

    private void updateMotors(){
        frontLeft.setPower(powerVector.getX() - powerVector.getY() - powerVector.getZ());
        frontRight.setPower(powerVector.getX() + powerVector.getY() + powerVector.getZ());
        backLeft.setPower(powerVector.getX() + powerVector.getY() - powerVector.getZ());
        backRight.setPower(powerVector.getX() - powerVector.getY() + powerVector.getZ());

        frontLeft.update();
        frontRight.update();
        backLeft.update();
        backRight.update();
    }

    public Pose getPoseEstimate(){
        return localizer.getPoseEstimate();
    }

    @Override
    public void update() {
        localizer.update();
        updatePowerVector();
        updateMotors();
    }

    @Override
    public void emergencyStop() {
        powerVector = new Vector();
        updateMotors();
    }
}
