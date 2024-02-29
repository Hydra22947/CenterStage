package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;

import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public static double WAIT_TIME = .75;

    public static void setAnimation(MeepMeep meepMeep, RoadRunnerBotEntity redLeftBot) {
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(redLeftBot)
                //  .addEntity(redRightBot)
                .start();
    }

    public static void leftBlueTraj(RoadRunnerBotEntity myBot) {
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(40, 62, Math.toRadians(-90)))
                .lineToY(12)
                .waitSeconds(1)
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(-24, 10, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                .splineToSplineHeading(new Pose2d(20, 10, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                .setTangent(0)

                .splineToLinearHeading(new Pose2d(51, 40, Math.toRadians(0)), Math.toRadians(0))

                .build());
    }

    /*

        public static RoadRunnerBotEntity redRightTraj(DefaultBotBuilder myBot)
        {
       */
/*     Pose2d placePixelPose = new Pose2d(42,-35, Math.toRadians());
        //    Pose2d intakePixelPose = new Pose2d(-55,-35);
        Pose2d intakePixelPose = new Pose2d(-34,-35);
        Pose2d parkPose = new Pose2d(50, -55);
        return myBot.setConstraints(65, 45, Math.toRadians(180), Math.toRadians(180), 13.915)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -62, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(35, -20.75, Math.toRadians(0)))

*//*

     */
/*                                .strafeLeft(25)
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
  *//*

     */
/*   .lineToLinearHeading(new Pose2d(50, -30, Math.toRadians(0)))
                                   .waitSeconds(0.5)
                                   .lineToConstantHeading(new Vector2d(28,-32))
                                   .waitSeconds(0.5)
                                   .lineToConstantHeading(new Vector2d(36,-32))
   *//*

                                //\    .lineToLinearHeading(new Pose2d(35, -20.75, Math.toRadians(0)))

     */
/*                           // place yellow and purple pixel distance
                                .lineToLinearHeading(new Pose2d(52.5, -32.75, Math.toRadians(0)))
                                .waitSeconds(1.5)

                                //Intake
                                .lineToSplineHeading(new Pose2d(25,-37))
                                .splineToLinearHeading(intakePixelPose, Math.toRadians(180))
                                .waitSeconds(1.5)

                                //place pixels
                                .forward(80)
                                .waitSeconds(.2)
                                .waitSeconds(.1)
                                .waitSeconds(.1)

                                //Intake
                                //.lineToSplineHeading(new Pose2d(25,-35))
                                .lineToSplineHeading(new Pose2d(-34,-35))
                                //               .splineToLinearHeading(intakePixelPose, Math.toRadians(180))
                                .waitSeconds(1.5)

                                //place pixels
                                .forward(80)
                                .waitSeconds(.2)
                                .waitSeconds(.1)
                                .waitSeconds(.1)

                                //Intake
                                // .lineToSplineHeading(new Pose2d(25,-35))
                                .lineToSplineHeading(new Pose2d(-34,-35))
                                //  .splineToLinearHeading(intakePixelPose, Math.toRadians(180))
                                .waitSeconds(1.5)

                                //place pixels
                                .forward(80)
                                .waitSeconds(.2)
                                .waitSeconds(.1)
                                .waitSeconds(.1)

                                //Intake
                                .lineToConstantHeading(new Vector2d(0,-35))
                                .lineToLinearHeading(new Pose2d(-34,-35, Math.toRadians(-45)))
                                //      .li(new Pose2d(-34,-35, Math.toRadians(-45)), Math.toRadians(180))
                                .waitSeconds(1.5)
                                .build());*//*

    }
    public static RoadRunnerBotEntity redLeftTraj(DefaultBotBuilder myBot)
    {
        myBot.setConstraints(65, 65, Math.toRadians(360), Math.toRadians(360), 12.81).build();
        */
/* Pose2d placePixelPose = new Pose2d(47, -26, Math.toRadians(0));
        Pose2d stageDoorStartPose = new Pose2d(30, -15, Math.toRadians(0));
        Pose2d stageDoorEndPose = new Pose2d(12, -5, Math.toRadians(0));
        Pose2d intakePixel = new Pose2d(-40, -8);
        Pose2d parkPose = new Pose2d(51, -5, Math.toRadians(90));


        return myBot.setConstraints(65, 65, Math.toRadians(360), Math.toRadians(360), 12.81)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder( new Pose2d(-40, -62, Math.toRadians(90)))

                                .lineToLinearHeading(new Pose2d(-45.5, -15, Math.toRadians(80)))
                                .lineToLinearHeading(new Pose2d(-40, -8.2, Math.toRadians(0)))















                                .lineToLinearHeading(new Pose2d(50, -30, Math.toRadians(0)))
                                .waitSeconds(0.5)
                                .lineToConstantHeading(new Vector2d(28,-32))
                                .waitSeconds(0.5)
                                .lineToConstantHeading(new Vector2d(36,-32))



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
                );*//*

    }
*/
    public static void blueLeftLeft(RoadRunnerBotEntity myBot) {
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(16, 62, Math.toRadians(-90)))
                .lineToY(12)
                .setTangent(0)

                .splineToLinearHeading(new Pose2d(45, 40, Math.toRadians(30)), Math.toRadians(0))
                .waitSeconds(.5)
                .lineToXLinearHeading(51, Math.toRadians(0))
                .waitSeconds(1)
                .setTangent(Math.toRadians(90))
                .lineToY(60)
                .build());
    }

    public static void blueLeftMid(RoadRunnerBotEntity myBot) {
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(16, 62, Math.toRadians(-90)))
                .setTangent(0)

                .splineToLinearHeading(new Pose2d(45, 35, Math.toRadians(30)), Math.toRadians(0))
                .waitSeconds(.5)
                .lineToXLinearHeading(51, Math.toRadians(0))
                .waitSeconds(1)
                .setTangent(Math.toRadians(90))
                .lineToY(60)
                .build());
    }

    public static void blueRightLeft(RoadRunnerBotEntity myBot) {
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-36, 62, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(-35, 34), Math.toRadians(180))
                .waitSeconds(.2)
                .setTangent(0)
                .strafeToLinearHeading(new Vector2d(-35, 12), Math.toRadians(0))

                .lineToYLinearHeading(10.9, Math.toRadians(0))
                .waitSeconds(.5)
                .waitSeconds(2)

                .strafeToLinearHeading(new Vector2d(30, 9), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(51.2, 28, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                .waitSeconds(.5)
                .setTangent(Math.toRadians(90))

                .build());
    }

    public static void blueMidRight(RoadRunnerBotEntity myBot) {
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-36, 62, Math.toRadians(-90)))
                //place purple
                .strafeToLinearHeading(new Vector2d(-38, 34), Math.toRadians(-90))

                //intake from mid stack
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-53.45, 26, Math.toRadians(0)), Math.toRadians(180))
                .waitSeconds(.5)

                .strafeToLinearHeading(new Vector2d(30, 9), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(51.2, 28, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                .waitSeconds(.5)
                .setTangent(Math.toRadians(90))

                .build());
    }

    public static void blueRightRight(RoadRunnerBotEntity myBot) {
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-36, 62, Math.toRadians(-90)))
                //place purple
                .strafeToSplineHeading(new Vector2d(-35, 48), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-48, 38, Math.toRadians(-90)), Math.toRadians(-90))


                //intake from left stack
                .strafeToSplineHeading(new Vector2d(-40, 38), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-34, 15, Math.toRadians(-90)), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(-34, 11), Math.toRadians(0))
                .waitSeconds(.5)

/*
                //intake from mid stack
                .strafeToSplineHeading(new Vector2d(-30, 20), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-35, 12, Math.toRadians(0)), Math.toRadians(0))
*/
                //intake from mid stack

                .build());
    }

    public static void blueLeftRight(RoadRunnerBotEntity myBot) {
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-36, 62, Math.toRadians(-90)))
                //place purple
                .strafeToSplineHeading(new Vector2d(-38, 38), Math.toRadians(-40))
                .splineToLinearHeading(new Pose2d(-34, 32, Math.toRadians(0)), Math.toRadians(0))


                //intake from mid stack
                .strafeToLinearHeading(new Vector2d(-53.45, 26), Math.toRadians(0))


                //      .lineToYLinearHeading(10.9, Math.toRadians(0))
                .waitSeconds(.5)
                .waitSeconds(2)

                .strafeToLinearHeading(new Vector2d(30, 9), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(51.2, 28, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                .waitSeconds(.5)
                .setTangent(Math.toRadians(90))

                .build());
    }

    public static void nig(RoadRunnerBotEntity myBot) {
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-36, 62, Math.toRadians(-90)))
                .lineToYLinearHeading(12.5, Math.toRadians(-75))
                .lineToYLinearHeading(14.5, Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(30, 9), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(51.2, 28.8, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                .setTangent(Math.toRadians(90))
                .lineToY(64)

                .build());
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)

                .build();
        blueMidRight(myBot);
        setAnimation(meepMeep, myBot);

    }
}