package org.firstinspires.ftc.teamcode.newCustomAuto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.roadrunner.StandardTrackingWheelLocalizer;

import java.util.ArrayList;

@Config
public class Localizer implements IRobotModule {

    public static boolean ENABLED = true;

    protected Pose pose;
    public final StandardTrackingWheelLocalizer localizer;
    public IMU imu;

    public Localizer(RobotHardware robot, Pose initialPose) {
        this.pose = initialPose;
        this.imu = robot.imu;
        this.localizer = new StandardTrackingWheelLocalizer(robot);
        localizer.setPoseEstimate(new Pose2d(initialPose.getX(), initialPose.getY(), initialPose.getHeading()));
    }

    public Localizer(RobotHardware robot) {
        this.pose = new Pose();
        this.imu = robot.imu;
        this.localizer = new StandardTrackingWheelLocalizer(robot);
        localizer.setPoseEstimate(new Pose2d());
    }

    public void setPose(Pose pose) {
        this.pose = pose;
    }

    public Pose getPoseEstimate() {
        return pose;
    }

    public Pose getPredictedPoseEstimate(){
        return new Pose(pose.getX() + glideDelta.getX(), pose.getY() + glideDelta.getY(), pose.getHeading());
    }

    public double getHeading(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    private Vector velocity;
    public Vector glideDelta;

    public static double filterParameter = 0.8;
    private final LowPassFilter xVelocityFilter = new LowPassFilter(filterParameter, 0),
            yVelocityFilter = new LowPassFilter(filterParameter, 0);

    public static double xDeceleration = 45, yDeceleration = 120;

    public Vector getVelocity(){
        return velocity;
    }

    @Override
    public void update() {
        if(!ENABLED) return;

        localizer.update();
        Pose2d pose2d = localizer.getPoseEstimate();
        pose = new Pose(pose2d.getX(), pose2d.getY(), pose2d.getHeading());

        velocity = new Vector(xVelocityFilter.getValue(localizer.getPoseVelocity().getX()), yVelocityFilter.getValue(localizer.getPoseVelocity().getY()));
        Vector predictedGlideVector = new Vector(Math.signum(velocity.getX()) * velocity.getX() * velocity.getX() / (2.0 * xDeceleration), Math.signum(velocity.getY()) * velocity.getY() * velocity.getY() / (2.0 * yDeceleration));
        glideDelta = Vector.rotateBy(predictedGlideVector, -pose.getHeading());
    }
}