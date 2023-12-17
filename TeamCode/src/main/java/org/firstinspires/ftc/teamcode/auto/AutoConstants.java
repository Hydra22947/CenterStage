package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class AutoConstants {
    //Coordinates
    public static final double startX = -34, startY = -60, startHeading = 90;
    public static final double strafeForPurplePixel = 55;
    public static  double placePixelPoseX = 45, placePixelPoseY = -33;
    public static final double stageDoorStartPoseX = 30, stageDoorStartPoseY = -21.5;
    public static final double stageDoorMidPoseX = 15, stageDoorMidPoseY = 0, stageDoorMidPoseHeading = 90;
    public static final double stageDoorEndPoseX = 25, stageDoorEndPoseY = 0;
    public static final double intakePixelVectorX = -34, intakePixelVectorY = 0;

    //WAIT TIME
    public static final double WAIT_EXTENSION = .5;

    //Poses
    public static final Pose2d startPose = new Pose2d(startX, startX, Math.toRadians(startHeading));
    public static final Pose2d placePixelPose = new Pose2d(placePixelPoseX, placePixelPoseY, Math.toRadians(0));
    public static final Pose2d stageDoorStartPose = new Pose2d(stageDoorStartPoseX, stageDoorStartPoseY, Math.toRadians(0));
    public static final Pose2d stageDoorMidPose = new Pose2d(stageDoorMidPoseX, stageDoorMidPoseY, Math.toRadians(stageDoorMidPoseHeading));
    public static final Pose2d stageDoorEndPose = new Pose2d(stageDoorEndPoseX, stageDoorEndPoseY, Math.toRadians(0));
    public static final Vector2d intakePixelVector = new Vector2d(intakePixelVectorX, intakePixelVectorY);

}