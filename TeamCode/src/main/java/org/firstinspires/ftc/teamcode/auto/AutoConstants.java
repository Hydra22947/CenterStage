package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class AutoConstants {

    public static double placePurpleStrafe = 30, stageDoorStrafe = 8;
    public static final double startX = -34, startY = -60, startH = Math.toRadians(0);
    ;
    public static final double placePixelX = 45.0, placePixelY = -33, placePixelH = Math.toRadians(0);
    public static final double stageDoorPoseX = 30, stageDoorPoseY = -21.5, stageDoorPoseH = Math.toRadians(0);
    public static final double stageDoorVectorX = 0, stageDoorVectorY = 12;
    public static final double intakePixelVectorX = -34, intakePixelVectorY = -12;
    public static final double strafeLeftToLine = 30, strafeLeftToStageDoor = 8;
    public static final double WAIT_TIME = .5;

    public static Pose2d startPoseLeft = new Pose2d(startX, startY, startH);
    public static Pose2d placePixelPose = new Pose2d(placePixelX, -placePixelY, placePixelH);
    public static Pose2d stageDoorPose = new Pose2d(stageDoorPoseX, stageDoorPoseY, stageDoorPoseH);
    public static Vector2d stageDoorVector = new Vector2d(stageDoorVectorX, stageDoorVectorY);

    public static Vector2d intakePixelVector = new Vector2d(intakePixelVectorX, intakePixelVectorY);
}
