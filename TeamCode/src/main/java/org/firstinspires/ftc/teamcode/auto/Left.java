package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.slf4j.Marker;

public class Left extends LinearOpMode {

    RobotHardware robot = RobotHardware.getInstance();
    Markers markers = new Markers();



    MecanumDrive drive = new MecanumDrive(hardwareMap , new Pose2d(0,0,0), robot );

    // TODO: add red trajectorys , shouldnt take long.

    @Override
    public void runOpMode() throws InterruptedException {

    Action runRight = drive.actionBuilder(new Pose2d(PoseStorage.startX , PoseStorage.startY, PoseStorage.startH))
            .afterTime(5,markers.score())
            .build();

    }
}
