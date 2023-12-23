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
                                .lineToLinearHeading(parkPose)
                                .build());
    }
    public static RoadRunnerBotEntity redLeftTraj(DefaultBotBuilder myBot)
    {
        Pose2d placePixelPose = new Pose2d(47, -26, Math.toRadians(0));
        Pose2d stageDoorStartPose = new Pose2d(30, -15, Math.toRadians(0));
        Pose2d stageDoorEndPose = new Pose2d(12, -5, Math.toRadians(0));
        Pose2d intakePixel = new Pose2d(-40, -8);
        Pose2d parkPose = new Pose2d(51, -5, Math.toRadians(90));


        return myBot.setConstraints(65, 65, Math.toRadians(360), Math.toRadians(360), 12.81)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-36, -62, Math.toRadians(90)))
                                        .forward(52)
                                        .waitSeconds(.6)
                                        .lineToLinearHeading(new Pose2d(intakePixel.getX(), intakePixel.getY(),Math.toRadians(0)))
                                        .waitSeconds(1.7)
                                        .lineToSplineHeading(stageDoorEndPose)
                                        .splineToLinearHeading(placePixelPose, Math.toRadians(0))
                                        .waitSeconds(.3)
                                        .lineToSplineHeading(stageDoorStartPose)
                                        //.splineToLinearHeading(intakePixel, Math.toRadians(180))
                                        .waitSeconds(0.5)
                                        .lineToSplineHeading(stageDoorEndPose)
                                        .splineToLinearHeading(placePixelPose, Math.toRadians(0))
                                        .waitSeconds(.3)
                                        .lineToSplineHeading(stageDoorStartPose)
                                        .splineToLinearHeading(intakePixel, Math.toRadians(180))
                                        //Going for backdrop
                                        .waitSeconds(0.5)
                                        .lineToSplineHeading(stageDoorEndPose)
                                        .splineToLinearHeading(placePixelPose, Math.toRadians(0))
                                        //Going for intake
                                        .waitSeconds(.3)
                                        .lineToSplineHeading(stageDoorStartPose)
                                        .splineToLinearHeading(intakePixel, Math.toRadians(180))
                                        //Going for backdrop + park
                                        .waitSeconds(0.5)
                                        .lineToSplineHeading(stageDoorEndPose)
                                        .splineToLinearHeading(placePixelPose, Math.toRadians(0))
                                        .lineToLinearHeading(parkPose)
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