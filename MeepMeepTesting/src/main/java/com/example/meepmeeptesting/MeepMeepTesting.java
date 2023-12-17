package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.QuinticSpline;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public static double WAIT_TIME = .75;
    public static void setAnimation(MeepMeep meepMeep, RoadRunnerBotEntity redLeftBot)
    {
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(redLeftBot)
                //  .addEntity(redRightBot)
                .start();
    }


    public static RoadRunnerBotEntity redRightTraj(DefaultBotBuilder myBot)
    {
        Pose2d placePixelPose = new Pose2d(42,-35);
        //    Pose2d intakePixelPose = new Pose2d(-55,-35);
        Pose2d intakePixelPose = new Pose2d(-34,-35);
        Pose2d parkPose = new Pose2d(50, -55);
        return myBot.setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -60, 0))
                                .strafeLeft(25)
                                .waitSeconds(.5)
                                .lineToLinearHeading(placePixelPose)
                                .waitSeconds(WAIT_TIME)
                                .lineToLinearHeading(intakePixelPose)
                                .waitSeconds(WAIT_TIME)
                                .lineToLinearHeading(placePixelPose)
                                .waitSeconds(WAIT_TIME)
                                .lineToLinearHeading(intakePixelPose)
                                .waitSeconds(WAIT_TIME)
                                .lineToLinearHeading(placePixelPose)
                                .waitSeconds(WAIT_TIME)
                                .lineToLinearHeading(intakePixelPose)
                                .waitSeconds(WAIT_TIME)
                                .lineToLinearHeading(placePixelPose)
                                .waitSeconds(WAIT_TIME)
                                .lineToLinearHeading(intakePixelPose)
                                .waitSeconds(WAIT_TIME)
                                .lineToLinearHeading(placePixelPose)
                                .waitSeconds(WAIT_TIME)
                                .build());
    }
    public static RoadRunnerBotEntity redLeftTraj(DefaultBotBuilder myBot)
    {
        Pose2d placePixelPose = new Pose2d(45, -33, Math.toRadians(0));
        Pose2d stageDoorStartPose = new Pose2d(40, -27, Math.toRadians(0));
        Vector2d stageDoorVector = new Vector2d(0, -12);
        Pose2d stageDoorMidPose = new Pose2d(5, -12, Math.toRadians(0));
        Pose2d stageDoorEndPose = new Pose2d(12, -12, Math.toRadians(0));
        Pose2d middleIntakePixelVector = new Pose2d(-45, -25);
        Pose2d intakePixelVector = new Pose2d(-40, -8);


        return myBot.setConstraints(60, 60, Math.toRadians(360), Math.toRadians(180), 12.81)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-34, -60, Math.toRadians(90)))
                                        //Place purple pixel
                                        // .strafeLeft(30)
                                        .forward(40)
                                        .waitSeconds(.5)



                                        //Going for backdrop
                                        .splineToSplineHeading(stageDoorMidPose, Math.toRadians(0))
                                        .splineToSplineHeading(stageDoorEndPose,Math.toRadians(0))
                                        .splineToLinearHeading(placePixelPose, Math.toRadians(0))

                                        //Going for intake
                                        .waitSeconds(WAIT_TIME)
                                        .lineToSplineHeading(stageDoorStartPose)
                                        .splineToSplineHeading(intakePixelVector, Math.toRadians(180))

/*
                                //Going for backdrop
                                .waitSeconds(WAIT_TIME)
                                .splineToConstantHeading(stageDoorVector, Math.toRadians(0))
                                .splineToLinearHeading(placePixelPose, Math.toRadians(0))

                                //Going for intake
                                .waitSeconds(WAIT_TIME)
                                .lineToSplineHeading(stageDoorPose)
                                .splineToConstantHeading(intakePixelVector, Math.toRadians(180))

                                //Going for backdrop
                                .waitSeconds(WAIT_TIME)
                                .splineToConstantHeading(stageDoorVector, Math.toRadians(0))
                                .splineToSplineHeading(placePixelPose, Math.toRadians(0))

                                //Going for intake
                                .waitSeconds(WAIT_TIME)
                                .lineToSplineHeading(stageDoorPose)
                                //.splineToConstantHeading(intakePixelVector, Math.toRadians(180))
                                .splineToSplineHeading(middleIntakePixelVector, Math.toRadians(180))

                                //Going for backdrop + park
                                .waitSeconds(WAIT_TIME)
                                //  .splineToConstantHeading(stageDoorVector, Math.toRadians(0))
                                .splineToLinearHeading(stageDoorPose, Math.toRadians(0))
                                .splineToSplineHeading(placePixelPose, Math.toRadians(0))
*/
                                        .build()
                );
    }


    public static void main(String[] args)
    {
        MeepMeep meepMeep = new MeepMeep(700);
        RoadRunnerBotEntity leftRedBot = redLeftTraj(new DefaultBotBuilder(meepMeep));
        RoadRunnerBotEntity rightRedBot = redRightTraj(new DefaultBotBuilder(meepMeep));
        leftRedBot.setDimensions(14, 15.75);
        setAnimation(meepMeep, leftRedBot);
    }
}