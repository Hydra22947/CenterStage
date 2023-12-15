package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

@Config
public class AutoConstants {

    public static double placePurpleStrafe = 30, stageDoorStrafe = 8;
    public static double startX = 34, startY = 60, startH = Math.toRadians(90);
    ;
    public static double placePixelX = 45.0, placePixelY = -33, placePixelH = Math.toRadians(0);
    public static double stageDoorPoseX = 30, stageDoorPoseY = -21.5, stageDoorPoseH = Math.toRadians(0);
    public static double stageDoorVectorX = 0, stageDoorVectorY = 12;
    public static double intakePixelVectorX = -34, intakePixelVectorY = -12;
    public static double strafeLeftToLine = 30, strafeLeftToStageDoor = 38;
    public static double WAIT_TIME = .5;

    public static Pose2d startPoseLeft = new Pose2d(startX, startY, startH);
    public static Pose2d placePixelPose = new Pose2d(placePixelX, -placePixelY, placePixelH);
    public static Vector2d stageDoorVector1 = new Vector2d(stageDoorPoseX, stageDoorPoseY);
    public static Vector2d stageDoorVector2 = new Vector2d(stageDoorVectorX, stageDoorVectorY);
    public static Vector2d intakePixelVector = new Vector2d(intakePixelVectorX, intakePixelVectorY);

    public Pose2d currentPose;
}
