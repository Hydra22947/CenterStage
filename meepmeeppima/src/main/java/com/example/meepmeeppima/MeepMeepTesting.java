package com.example.meepmeeppima;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot
                = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(90), 12.3139452958)
                .build();


        BlueLeftMiddle(myBot);
        setAnimation(meepMeep , myBot);
       // BlueLeftLeft(myBot);
       // BlueLeftRight(myBot);



    }

        public static void setAnimation(MeepMeep meepMeep, RoadRunnerBotEntity redLeftBot) {
            meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(redLeftBot)
                    //  .addEntity(redRightBot)
                    .start();
        }


    public static void BlueLeftMiddle( RoadRunnerBotEntity myBot) {

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(10, 58, 0))
                .splineToSplineHeading(new Pose2d(-32, 58, Math.toRadians(0)),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-41.8, 38, Math.toRadians(0)), Math.toRadians(-180))
                .strafeTo(new Vector2d(-41.8 , 42.8))
                .build());

    }

        public static void BlueLeftLeft ( RoadRunnerBotEntity myBot) {

             myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(16, 62, -90))
            .strafeToLinearHeading(new Vector2d(48 ,34.25 ), Math.toRadians(0))
            .waitSeconds(0.5)
            .strafeToLinearHeading(new Vector2d(50,34.25 ), Math.toRadians(0))
            .setTangent(190)
            .splineToSplineHeading(new Pose2d(10, 58, Math.toRadians(0)), Math.toRadians(180))
            .splineToSplineHeading(new Pose2d(-32, 58, Math.toRadians(0)),Math.toRadians(180))
            .splineToLinearHeading(new Pose2d(-42, 42, Math.toRadians(10)), Math.toRadians(-180))
            //Place Preload on board
            //  .splineToLinearHeading(new Pose2d(50.25, 34, Math.toRadians(0)), Math.toRadians(0))
            .waitSeconds(.5)

    //Intake 54

            .strafeToLinearHeading(new Vector2d(-32, 58) , Math.toRadians(0))
            .setTangent(Math.toRadians(0))
            .splineToConstantHeading(new Vector2d(22, 58), Math.toRadians(-180))
            .setTangent(Math.toRadians(0))
            .splineToLinearHeading(new Pose2d(48, 34.25, Math.toRadians(0)) , Math.toRadians(-90))
            .strafeTo(new Vector2d(48.5, 34.25))

            .build());

    }

    public static void BlueLeftRight( RoadRunnerBotEntity myBot) {

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(16, 62, -90))
                .strafeToLinearHeading(new Vector2d(20 ,34.25 ), Math.toRadians(0))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(50,34.25 ), Math.toRadians(0))
                .setTangent(190)
                .splineToSplineHeading(new Pose2d(10, 58, Math.toRadians(0)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-32, 58, Math.toRadians(0)),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-42, 42, Math.toRadians(10)), Math.toRadians(-180))
                //Place Preload on board
                //  .splineToLinearHeading(new Pose2d(50.25, 34, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(.5)

                //Intake 54

                .strafeToLinearHeading(new Vector2d(-32, 58) , Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(22, 58), Math.toRadians(-180))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(48, 34.25, Math.toRadians(0)) , Math.toRadians(-90))
                .strafeTo(new Vector2d(48.5, 34.25))

                .build());

    }
}


