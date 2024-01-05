package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

@Config
public class AutoConstants {
    //Coordinates
    public static double strafeForPurplePixel = 52;
    public   double placePixelPoseX = 51.4, placePixelPoseY = -28.8;
    public static double ROBOT_ERROR_INTAKE_Y = -1.5, TEMP = 7;;
    public static double ROBOT_ERROR_INTAKE_X = -0.2;
    //WAIT TIME
    public static final double WAIT = .5 ;


    //Poses
    public Pose2d startPoseRedLeft = new Pose2d(-36, -62, Math.toRadians(90)); // ok
    public Pose2d placePixelPose = new Pose2d(51.4, -28.8, Math.toRadians(0)); // ok
    public Pose2d stageDoorStartPose = new Pose2d(30, -15, Math.toRadians(0));  // ok
    public Pose2d stageDoorEndPose = new Pose2d(12, -9, Math.toRadians(0));
    public Pose2d park = new Pose2d(51, -5, Math.toRadians(90));
    public Pose2d intakePixelVector = new Pose2d(-38.8, -8.4);
    public static Pose2d currentPose = new Pose2d(0,0);
}