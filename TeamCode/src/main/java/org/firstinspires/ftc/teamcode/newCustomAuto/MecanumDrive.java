package org.firstinspires.ftc.teamcode.newCustomAuto;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;

@Config
public class MecanumDrive implements IRobotModule {

    private RobotHardware robot = RobotHardware.getInstance();
    public static boolean ENABLED = true;

    private Localizer localizer;

    public static boolean frontLeftMotorReversed = true, frontRightMotorReversed = false, backLeftMotorReversed = false, backRightMotorReversed = true;

    public static PIDCoefficients translationalPID = new PIDCoefficients(0.105,0.04 ,0.003),
            headingPID = new PIDCoefficients(0.8,0,0.006);
    public final PIDController tpid= new PIDController(0,0,0), hpid = new PIDController(0,0,0);

    public static double lateralMultiplier = SampleMecanumDrive.LATERAL_MULTIPLIER;

    public double overallMultiplier = 1;

    public double velocityThreshold = 0.5;

    public enum RunMode{
        PID, Vector
    }

    private RunMode runMode;

    AutoMotor frontLeft, frontRight, backLeft, backRight;

    public MecanumDrive(Localizer localizer, RunMode runMode, boolean brake){
        if(!ENABLED) {
            this.localizer = null;
            frontLeft = null;
            frontRight = null;
            backLeft = null;
            backRight = null;
            this.runMode = runMode;
            return;
        }

        this.localizer = localizer;
        frontLeft = new AutoMotor(robot.dtFrontLeftMotor, AutoMotor.RunMode.RUN, frontLeftMotorReversed);
        frontRight = new AutoMotor(robot.dtFrontRightMotor, AutoMotor.RunMode.RUN, frontRightMotorReversed);
        backLeft = new AutoMotor(robot.dtBackLeftMotor, AutoMotor.RunMode.RUN, backLeftMotorReversed);
        backRight = new AutoMotor(robot.dtBackRightMotor, AutoMotor.RunMode.RUN, backRightMotorReversed);

        if(brake){
            frontLeft.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        this.runMode = runMode;
    }

    public MecanumDrive(Localizer localizer, boolean brake){
        this(localizer, RunMode.Vector, brake);
    }

    public void setLocalizer(Localizer localizer){
        this.localizer = localizer;
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

    public Localizer getLocalizer(){
        return localizer;
    }

    public Pose getTargetPose(){
        return targetPose;
    }

    public void setRunMode(RunMode runMode){
        this.runMode = runMode;
    }

    public boolean reachedTarget(double tolerance){
        if(runMode == RunMode.Vector) return false;
        return localizer.getPoseEstimate().getDistance(targetPose) <= tolerance;
    }

    public boolean reachedHeading(double tolerance){
        if(runMode == RunMode.Vector) return false;
        return Math.abs((localizer.getPoseEstimate().getHeading() - targetPose.getHeading())%(2*PI)) <= tolerance;
    }

    public boolean stopped(){
        return localizer.getVelocity().getMagnitude() <= velocityThreshold;
    }

    private void updatePowerVector(){
        switch (runMode){
            case Vector:
                powerVector = new Vector(targetVector.getX(), targetVector.getY(), targetVector.getZ());
                powerVector = Vector.rotateBy(powerVector, localizer.getHeading());
                powerVector = new Vector(powerVector.getX(), powerVector.getY() * lateralMultiplier, targetVector.getZ());
                break;
            case PID:
                Pose currentPose = localizer.getPredictedPoseEstimate();

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

                powerVector= new Vector(powerVector.getX(),powerVector.getY() * lateralMultiplier, headingPower);
                break;
        }
        if(Math.abs(powerVector.getX()) + Math.abs(powerVector.getY()) + Math.abs(powerVector.getZ()) > 1)
            powerVector.scaleToMagnitude(1);
        powerVector.scaleBy(overallMultiplier);
    }

    private void updateMotors(){
        robot.dtFrontLeftMotor.setPower(powerVector.getX() - powerVector.getY() - powerVector.getZ());
        robot.dtFrontRightMotor.setPower(powerVector.getX() + powerVector.getY() + powerVector.getZ());
        robot.dtBackLeftMotor.setPower(powerVector.getX() + powerVector.getY() - powerVector.getZ());
        robot.dtBackRightMotor.setPower(powerVector.getX() - powerVector.getY() + powerVector.getZ());

        frontLeft.update();
        frontRight.update();
        backLeft.update();
        backRight.update();
    }

    @Override
    public void update() {
        if(!ENABLED) return;

        updatePowerVector();
        updateMotors();
    }

    @Override
    public void emergencyStop() {
        powerVector = new Vector();
        updateMotors();
    }
}