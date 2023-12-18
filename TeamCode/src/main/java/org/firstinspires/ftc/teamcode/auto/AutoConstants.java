package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

@Config
public class AutoConstants {
    //Coordinates
    public static double startX = -34, startY = -60, startHeading = 90;
    public static double strafeForPurplePixel = 52;
    public static  double placePixelPoseX = 45, placePixelPoseY = -26;
    public static double stageDoorStartPoseX = 30, stageDoorStartPoseY = -15;
    public static double stageDoorMidPoseX = -15, stageDoorMidPoseY = -12, stageDoorMidPoseHeading = 90;
    public static double stageDoorEndPoseX = 12, stageDoorEndPoseY = -5;
    public static double parkX = 51, parkY = -5, parkHeading = 90;
    public static double intakePixelVectorX = -40, intakePixelVectorY = -8;

    //WAIT TIME
    public static final double WAIT = .5;

    //Poses
    public Pose2d startPose = new Pose2d(startX, startY, Math.toRadians(startHeading));
    public Pose2d placePixelPose = new Pose2d(placePixelPoseX, placePixelPoseY, Math.toRadians(0));
    public Pose2d stageDoorStartPose = new Pose2d(stageDoorStartPoseX, stageDoorStartPoseY, Math.toRadians(0));
    public Pose2d stageDoorMidPose = new Pose2d(stageDoorMidPoseX, stageDoorMidPoseY, Math.toRadians(stageDoorMidPoseHeading));
    public Pose2d stageDoorEndPose = new Pose2d(stageDoorEndPoseX, stageDoorEndPoseY, Math.toRadians(0));
    public Pose2d park = new Pose2d(parkX, parkY, Math.toRadians(parkHeading));
    public Pose2d intakePixelVector = new Pose2d(intakePixelVectorX, intakePixelVectorY);
    public Pose2d intakePixelVector1 = new Pose2d(intakePixelVectorX , intakePixelVectorY-10);

    public static Pose2d currentPose = new Pose2d(0,0);
}