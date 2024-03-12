package com.example.meepmeeppima;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(90), 12.3139452958)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(16, 62, -90))
                .strafeToLinearHeading(new Vector2d(34.7,  32), Math.toRadians(0))
                .waitSeconds(1.5)
                .setTangent(0)
                //Place Preload on board
                .splineToLinearHeading(new Pose2d(50.25, 34, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(.5)

                //Intake 54
                .setTangent(Math.toRadians(-120))
                .splineToConstantHeading(new Vector2d(30, 9.5), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-28, 10.84, Math.toRadians(0)), Math.toRadians(180))
                .waitSeconds(.7)
                .strafeToLinearHeading(new Vector2d(-33, 10.84), Math.toRadians(0))
                .waitSeconds(1.5)
                .waitSeconds(0.5)
                .waitSeconds(1.25)

                //Deposit
                .strafeToLinearHeading(new Vector2d(30, 9), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(51, 25, Math.toRadians(5)), Math.toRadians(0))
                .waitSeconds(1.9)
                .waitSeconds(0.1)

                //Park
                .strafeToLinearHeading(new Vector2d(46, 33.5), Math.toRadians(-90))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}