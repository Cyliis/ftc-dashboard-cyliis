package org.firstinspires.ftc.teamcode.Modules;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TrajectoryStuff.Trajectory;
import org.firstinspires.ftc.teamcode.Utils.IRobotModule;
import org.firstinspires.ftc.teamcode.Utils.Pose;
import org.firstinspires.ftc.teamcode.Utils.Vector;

@Config
public class Follower implements IRobotModule {


    public static double followingCoefficient = 1, correctionCoefficient = 1, centripetalCorrectionCoefficient = 21, headingPIDCoefficient = 1;
    public static int segmentsPerUnit = 100;

    private final MecanumDrive drive;
    private final Localizer localizer;
    public final PredictiveLocalizer predictiveLocalizer;

    private Trajectory trajectory;
    public double currentFollowedPoint = 0;
    public double predictedFollowedPoint = 0;

    private final PIDController headingPIDController = new PIDController(0,0,0);

    public static double defaultPIDThreshold = 2.5;

    private double PIDThreshold = defaultPIDThreshold;

    private boolean pid = false;

    public static PIDCoefficients correctionPIDCoefficients = new PIDCoefficients(0,0,0);
    private PIDController correctionPID = new PIDController(0,0,0);

    public Follower(MecanumDrive drive, Localizer localizer){
        this.drive = drive;
        this.localizer = localizer;
        this.predictiveLocalizer = new PredictiveLocalizer(localizer);
    }

    public void setTrajectory(Trajectory newTrajectory, double PIDThreshold){
        this.trajectory = newTrajectory;
        this.PIDThreshold = PIDThreshold;
        this.pid = false;
        this.drive.setRunMode(MecanumDrive.RunMode.Vector);
        this.currentFollowedPoint = 0.001;

        timer.startTime();
        timer.reset();
    }

    public Trajectory getTrajectory(){
        return trajectory;
    }

    public Vector tangentVelocityVector = new Vector();
    public Vector predictedTangentVelocityVector = new Vector();
    public Vector driveVector = new Vector();
    public Vector correctingVector = new Vector();

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void update() {
        if(trajectory == null) return;
        if(pid) return;

        predictiveLocalizer.update();

        if(trajectory.getLength() - trajectory.getLengthAt(currentFollowedPoint) <= drive.getLocalizer().glideDelta.getMagnitude()){
            pid = true;
            drive.setRunMode(MecanumDrive.RunMode.PID);
            drive.setTargetPose(trajectory.getPose(1));
        }

        Pose currentPose = localizer.getPoseEstimate();
        Pose predictedPose = predictiveLocalizer.getPoseEstimate();

        currentFollowedPoint = trajectory.getFollowedPoint(currentPose, currentFollowedPoint);
        predictedFollowedPoint = trajectory.getFollowedPoint(currentPose, currentFollowedPoint);

        tangentVelocityVector = trajectory.getTangentVelocity(currentFollowedPoint).scaledBy(followingCoefficient);
        predictedTangentVelocityVector = trajectory.getTangentVelocity(predictedFollowedPoint).scaledBy(followingCoefficient);

        Pose trajectoryPose = trajectory.getPose(currentFollowedPoint);

        correctionPIDCoefficients = MecanumDrive.translationalPID;
        correctionPID.setPID(correctionPIDCoefficients.p, correctionPIDCoefficients.i, correctionPIDCoefficients.d);

        correctingVector = new Vector(predictedPose.getX() - currentPose.getX(), predictedPose.getY() - currentPose.getY(), 0);
        double correctionPower = correctionPID.calculate(-correctingVector.getMagnitude(),0);
        correctingVector.scaleToMagnitude(correctionPower);
        correctingVector.scaleBy(correctionCoefficient);

        Vector centripetalCorrectionVector = new Vector(Math.cos(Math.atan2(predictedTangentVelocityVector.getY(), predictedTangentVelocityVector.getX()) + PI/2.0),
                Math.sin(Math.atan2(predictedTangentVelocityVector.getY(), predictedTangentVelocityVector.getX()) + PI/2.0))
                .scaledBy(trajectory.getCurvature(predictedFollowedPoint) * centripetalCorrectionCoefficient);

        timer.reset();

        headingPIDController.setPID(MecanumDrive.headingPID.p, MecanumDrive.headingPID.i, MecanumDrive.headingPID.d);
        double headingDelta = trajectoryPose.getHeading() - currentPose.getHeading();
        headingDelta%=2.0 * PI;
        if(headingDelta > PI) headingDelta -= 2.0*PI;
        if(headingDelta < -PI) headingDelta += 2.0*PI;

        Vector turningVector = new Vector(0, 0, headingPIDController.calculate(-headingDelta, 0)).scaledBy(headingPIDCoefficient);

        driveVector = tangentVelocityVector.plus(correctingVector).plus(centripetalCorrectionVector).plus(turningVector);

        drive.setTargetVector(tangentVelocityVector.plus(turningVector).plus(centripetalCorrectionVector).plus(correctingVector));
    }


    @Override
    public void emergencyStop() {

    }
}
