package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "Auto Red Left FSM")
public class AutoRedLeftFSM extends LinearOpMode
{
    enum State
    {
      PLACE_PURPLE_PIXEL,
      PLACE_PRELOAD,
      INTAKE,
      PARK
    };
    AutoConstants autoConstants;

    @Override
    public void runOpMode() throws InterruptedException {
        autoConstants = new AutoConstants();
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

        drivetrain.setPoseEstimate(autoConstants.startPose);
        State currentState = State.PLACE_PURPLE_PIXEL;

        TrajectorySequence placePurplePixel = drivetrain.trajectorySequenceBuilder(autoConstants.startPose)
                .forward(AutoConstants.strafeForPurplePixel)
                .waitSeconds(AutoConstants.WAIT)
                .build();
        TrajectorySequence placePreloadAndIntake = drivetrain.trajectorySequenceBuilder(placePurplePixel.end())
                //Going for backdrop
                .lineToSplineHeading(autoConstants.stageDoorMidPose)
                .lineToSplineHeading(autoConstants.stageDoorEndPose)
                .splineToLinearHeading(autoConstants.placePixelPose, Math.toRadians(0))
                .waitSeconds(1)
                .turn(Math.toRadians(-90))
                .waitSeconds(1)
                .build();
        TrajectorySequence intake = drivetrain.trajectorySequenceBuilder(placePreloadAndIntake.end())
                .waitSeconds(AutoConstants.WAIT)
                .lineToSplineHeading(autoConstants.stageDoorStartPose)
                .splineToSplineHeading(autoConstants.intakePixelVector, Math.toRadians(180))
                .waitSeconds(AutoConstants.WAIT)

                .build();
        TrajectorySequence park = drivetrain.trajectorySequenceBuilder(intake.end())
                //Going for backdrop
                .lineToLinearHeading(autoConstants.park)
                .build();

        waitForStart();
        if (isStopRequested()) return;

        drivetrain.followTrajectorySequenceAsync(placePurplePixel);
        while(opModeIsActive() && !isStopRequested())
        {

            switch (currentState)
            {
                case PLACE_PURPLE_PIXEL:
                    if(!drivetrain.isBusy())
                    {
                        currentState = State.PLACE_PRELOAD;
                        drivetrain.followTrajectorySequenceAsync(placePreloadAndIntake);
                    }
                    break;
                case PLACE_PRELOAD:
                    if(!drivetrain.isBusy())
                    {
                        currentState = State.INTAKE;
                        drivetrain.followTrajectorySequenceAsync(intake);
                    }
                    break;
                case INTAKE:
                    if(!drivetrain.isBusy())
                    {
                        currentState = State.PARK;
                        drivetrain.followTrajectorySequenceAsync(park);
                    }
                    break;
                case PARK:

                    break;
            }
        }

        drivetrain.update();
        AutoConstants.currentPose = drivetrain.getPoseEstimate();

    }
}