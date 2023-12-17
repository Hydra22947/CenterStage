package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.testing.harman.PoseStorage;

@Config
@Autonomous(name = "Auto Red Left")
public class AutoRedLeft extends LinearOpMode
{

    AutoConstants autoConstants;

    @Override
    public void runOpMode() throws InterruptedException {
        autoConstants = new AutoConstants();
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);
        drivetrain.setPoseEstimate(autoConstants.startPose);

        TrajectorySequence placePurplePixel = drivetrain.trajectorySequenceBuilder(autoConstants.startPose)
                .forward(AutoConstants.strafeForPurplePixel)
                .waitSeconds(AutoConstants.WAIT_EXTENSION)
                .build();
        TrajectorySequence placePreload = drivetrain.trajectorySequenceBuilder(placePurplePixel.end())
                //Going for backdrop
                .lineToLinearHeading(autoConstants.stageDoorMidPose)
                .splineToSplineHeading(autoConstants.stageDoorEndPose, Math.toRadians(0))
                .splineToLinearHeading(autoConstants.placePixelPose, Math.toRadians(0))
                .waitSeconds(1)
                .build();

        TrajectorySequence park = drivetrain.trajectorySequenceBuilder(placePreload.end())
                //Going for backdrop
                .lineToLinearHeading(autoConstants.park)
                .build();

        TrajectorySequence goIntake = drivetrain.trajectorySequenceBuilder(placePreload.end())
                .waitSeconds(AutoConstants.WAIT_EXTENSION)
                .lineToSplineHeading(autoConstants.stageDoorStartPose)
                .splineToConstantHeading(autoConstants.intakePixelVector, Math.toRadians(180))
                .build();
        waitForStart();
        if (isStopRequested()) return;
        drivetrain.followTrajectorySequence(placePurplePixel);
        drivetrain.followTrajectorySequence(placePreload);
        drivetrain.followTrajectorySequence(park);
        while(opModeIsActive());
    }
}