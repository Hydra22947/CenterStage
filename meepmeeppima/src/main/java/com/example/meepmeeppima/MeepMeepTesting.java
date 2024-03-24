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


        BlueRightMiddle(myBot);
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


    public static void BlueRightLeft( RoadRunnerBotEntity myBot) {

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-40, 62, -90))


                .strafeToLinearHeading(new Vector2d(-48, 40), Math.toRadians(-90))

               .strafeToSplineHeading(new Vector2d(-42, 45), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-38, 11, Math.toRadians(-90)), Math.toRadians(-90))

                .strafeToLinearHeading(new Vector2d(-54.5, 11), Math.toRadians(0))


                .waitSeconds(0.25)

                .setTangent(-90)
                .splineToConstantHeading(new Vector2d(-42, 10), Math.toRadians(0))

                //.splineToLinearHeading( new Pose2d(-42, 10, Math.toRadians(0)), Math.toRadians(0))
                //deposit

                .splineToLinearHeading(new Pose2d(0, 8,Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(31, 20, Math.toRadians(24.5)), Math.toRadians(60))
                .splineToLinearHeading(new Pose2d(52, 32, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                .strafeToSplineHeading(new Vector2d(51, 38.2), Math.toRadians(0))

                .setTangent(Math.toRadians(-180))
                .splineToLinearHeading(new Pose2d(24, 9, Math.toRadians(0)), Math.toRadians(180))


                .splineToLinearHeading(new Pose2d(-36, 9.65, Math.toRadians(0)), Math.toRadians(-180))
                .waitSeconds(.2)

                .strafeToLinearHeading(new Vector2d(-44.5, 8.7), Math.toRadians(0))


                .strafeToLinearHeading(new Vector2d(15, 8), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(31, 20, Math.toRadians(24.5)), Math.toRadians(60))
                .splineToLinearHeading(new Pose2d(52, 32, Math.toRadians(0)), Math.toRadians(0))
                .build());

    }

        public static void BlueRightMiddle ( RoadRunnerBotEntity myBot) {

            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-32, 33, Math.toRadians(-90)))
                    .setTangent(-180)
                    .splineToSplineHeading(new Pose2d(-48, 41, Math.toRadians(0)), Math.toRadians(-90))
                    .splineToLinearHeading(new Pose2d(-46, 23, Math.toRadians(0)), Math.toRadians(180))

                    .strafeToLinearHeading(new Vector2d(-53, 23), Math.toRadians(0))
                    .strafeToLinearHeading(new Vector2d(-48, 23), Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(-32, 10), Math.toRadians(0))


                    //deposit
                    .splineToLinearHeading(new Pose2d(15, 8,Math.toRadians(0)), Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(31, 20, Math.toRadians(24.5)), Math.toRadians(60))
                    .splineToLinearHeading(new Pose2d(52, 32, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                    .strafeToSplineHeading(new Vector2d(51, 38.2), Math.toRadians(0))

                    .setTangent(Math.toRadians(-180))
                    .splineToLinearHeading(new Pose2d(24, 9, Math.toRadians(0)), Math.toRadians(180))


                    .splineToLinearHeading(new Pose2d(-36, 9.65, Math.toRadians(0)), Math.toRadians(-180))
                    .waitSeconds(.2)

                    .strafeToLinearHeading(new Vector2d(-44.5, 8.7), Math.toRadians(0))


                    .strafeToLinearHeading(new Vector2d(15, 8), Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(31, 20, Math.toRadians(24.5)), Math.toRadians(60))
                    .splineToLinearHeading(new Pose2d(52, 32, Math.toRadians(0)), Math.toRadians(0))






                    .build());

    }

    public static void BlueRightRight( RoadRunnerBotEntity myBot) {

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-37.5, 37.5, -90))


                .strafeToLinearHeading(new Vector2d(-30, 40), Math.toRadians(-40))
                .strafeToLinearHeading(new Vector2d(-32, 40), Math.toRadians(-40))
                .setTangent(-90)
                .splineToLinearHeading(new Pose2d(-45.5, 22, Math.toRadians(0)), Math.toRadians(180))

                .strafeToLinearHeading(new Vector2d(-53.6, 22 ),  Math.toRadians(0))

                .splineToLinearHeading(new Pose2d(-54.25, 22 ,  Math.toRadians(0)), Math.toRadians(0))

                .splineToLinearHeading( new Pose2d(-42, 10 , Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(15, 8, Math.toRadians(0)),Math.toRadians(0))

                .splineToSplineHeading(new Pose2d(31, 20, Math.toRadians(24.5)), Math.toRadians(60))

                .splineToLinearHeading(new Pose2d(52, 32, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)

                .strafeToSplineHeading(new Vector2d(51, 38), Math.toRadians(0))


                .build());

    }
}


