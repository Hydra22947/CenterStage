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


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);
        drivetrain.setPoseEstimate(AutoConstants.startPose);

        TrajectorySequence placePurplePixel = drivetrain.trajectorySequenceBuilder(AutoConstants.startPose)
                .forward(AutoConstants.strafeForPurplePixel)
                .waitSeconds(AutoConstants.WAIT_EXTENSION)
                .build();
        TrajectorySequence placePreload = drivetrain.trajectorySequenceBuilder(placePurplePixel.end())
                //Going for backdrop
                .lineToLinearHeading(AutoConstants.stageDoorMidPose)
                .splineToSplineHeading(AutoConstants.stageDoorEndPose, Math.toRadians(0))
                .splineToLinearHeading(AutoConstants.placePixelPose, Math.toRadians(0))
                .build();

        TrajectorySequence goIntake = drivetrain.trajectorySequenceBuilder(placePreload.end())
                .waitSeconds(AutoConstants.WAIT_EXTENSION)
                .lineToSplineHeading(AutoConstants.stageDoorStartPose)
                .splineToConstantHeading(AutoConstants.intakePixelVector, Math.toRadians(180))
                .build();
        waitForStart();
        if (isStopRequested()) return;
        drivetrain.followTrajectorySequence(placePurplePixel);
        drivetrain.followTrajectorySequence(placePreload);
        while(opModeIsActive());
    }
}