package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;

public class PoseStorage {

    public static double placePurpleStrafe = 30, stageDoorStrafe = 8;
    public static final double startX = 12, startY = -60 , startH = Math.toRadians(0);;
    public static final double placePixelX = 45.0, placePixelY = -33  ;
    public static final double stageDoorPoseX = 30, stageDoorPoseY = -21.5;
    public static final double stageDoorVectorX = 0, stageDoorVectorY = 12;
    public static final double intakePixelVectorX = -34, intakePixelVectorY = -12;


    Pose2d startPoseLeft = new Pose2d(startX , startY , startH);
    //Pose2d placePixelPose = new Pose2d(placePixelX, placePixelY, placeFirstPixelH);

}
