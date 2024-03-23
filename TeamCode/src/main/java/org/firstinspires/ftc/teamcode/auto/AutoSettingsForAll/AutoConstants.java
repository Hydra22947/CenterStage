package org.firstinspires.ftc.teamcode.auto.AutoSettingsForAll;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

@Config
public class AutoConstants {
    //Poses
    public static Pose2d startPoseRedLeft = new Pose2d(-40, -62, Math.toRadians(90));
    public static Pose2d startPoseRedRight = new Pose2d(16, -62, Math.toRadians(90));
    public static Pose2d startPoseBlueLeft = new Pose2d(16, 62, Math.toRadians(-90));
    public static Pose2d startPoseBlueRight = new Pose2d(-40, 62, Math.toRadians(-90));

}