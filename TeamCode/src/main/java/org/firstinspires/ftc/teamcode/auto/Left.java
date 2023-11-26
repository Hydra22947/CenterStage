package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class Left extends LinearOpMode {

    RobotHardware robot = RobotHardware.getInstance();
    Markers markers = new Markers();



    MecanumDrive drive = new MecanumDrive(hardwareMap , new Pose2d(0,0,0), robot );

    // TODO: add red trajectorys , shouldnt take long.

    @Override
    public void runOpMode() throws InterruptedException {

    Action placeYellowPixel = drive.actionBuilder(AutoConstants.startPoseLeft)
            //Place purple pixel
            .lineToX(AutoConstants.strafeLeftToLine)

            //Going for backdrop
            .waitSeconds(AutoConstants.WAIT_TIME)
            .lineToX(AutoConstants.strafeLeftToStageDoor)
            .splineToConstantHeading(AutoConstants.stageDoorVector, Math.toRadians(0))
            .splineToLinearHeading(AutoConstants.placePixelPose, Math.toRadians(0))

            //Going for intake
            .waitSeconds(AutoConstants.WAIT_TIME)
          //  .lineToSplineHeading(AutoConstants.stageDoorPose)
            .splineToConstantHeading(AutoConstants.intakePixelVector, Math.toRadians(180))
            .build();

    }
}
