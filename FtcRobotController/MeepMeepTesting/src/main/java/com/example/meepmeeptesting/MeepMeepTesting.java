package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;

import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public static double WAIT_TIME = .75;

    MeepMeep meepMeep = new MeepMeep(800);

    RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(90), 12.3139452958)
            .build();


    public static void setAnimation(MeepMeep meepMeep, RoadRunnerBotEntity redLeftBot) {
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(redLeftBot)
                //  .addEntity(redRightBot)
                .start();
    }


    public static void Auto4Cycles(RoadRunnerBotEntity myBot) {
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(16, 62, -90))
                .splineToLinearHeading(new Pose2d(40, 26, Math.toRadians(0)), Math.toRadians(0))
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


    }

    public static void leftBlueTraj(RoadRunnerBotEntity myBot) {
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-40, 62, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(-30, 37.5), Math.toRadians(-40))


                .setTangent(-90)
                .splineToLinearHeading(new Pose2d(-46, 22, Math.toRadians(0)), Math.toRadians(180))
                // .stopAndAdd(intake5OpenAction)

                .strafeToLinearHeading(new Vector2d(-54.25, 22), Math.toRadians(0))
                //         .stopAndAdd(intake5CloseAction)
                .splineToLinearHeading(new Pose2d(-42, 10, Math.toRadians(0)), Math.toRadians(0))

//deposit
                //   .afterTime(0, pleaseFixIntake())
                // .afterTime(.75, readyForDepositAction)
                .splineToLinearHeading(new Pose2d(30, 8, Math.toRadians(0)), Math.toRadians(0))

                .splineToLinearHeading(new Pose2d(52, 30, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                //.afterTime(0, depositAction)

                .strafeToSplineHeading(new Vector2d(51, 40), Math.toRadians(0))


                .setTangent(Math.toRadians(-180))
                .splineToLinearHeading(new Pose2d(24, 10, Math.toRadians(0)), Math.toRadians(180))


       //         .afterTime(0.9, intake43OpenAction)
                .splineToLinearHeading(new Pose2d(-36, 10, Math.toRadians(0)), Math.toRadians(-180))
                .waitSeconds(.2)

         //       .afterTime(0.6 ,intake43CloseAction)
                .strafeToLinearHeading(new Vector2d(-44.5, 10), Math.toRadians(0))


  //              .afterTime(0, pleaseFixIntake())
    //            .afterTime(1.25, updateElevatorHeight(1700))

                .strafeToLinearHeading(new Vector2d(30, 12), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(52.75, 28, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
            //    .afterTime(0, deposit43Action)


                .build());
    }


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

    public static void blueLeftMidMax(RoadRunnerBotEntity myBot) {
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(16, 62, Math.toRadians(-90)))
                .splineToLinearHeading(new Pose2d(40, 26, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(50.25, 34, Math.toRadians(0)), Math.toRadians(0))

                .strafeToLinearHeading(new Vector2d(48, 34.25), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(48, 38), Math.toRadians(0))

                .splineToSplineHeading(new Pose2d(10, 58, Math.toRadians(0)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-32, 58, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-42, 36, Math.toRadians(0)), Math.toRadians(-180))

                //      .splineToSplineHeading()

                // .strafeToLinearHeading(new Vector2d(48, 34.25), Math.toRadians(0))

                //    .splineToLinearHeading(new Pose2d(10, 58, Math.toRadians(0)), Math.toRadians(180))

/*
                .setTangent(Math.toRadians(-120))
                .splineToConstantHeading(new Vector2d(30, 9.5), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-28, 10.84, Math.toRadians(0)), Math.toRadians(180))
                .waitSeconds(.25)
*/
                //Getting Closer and fixing angle

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
                .strafeToLinearHeading(new Vector2d(-44.25, 10), Math.toRadians(0))

                .waitSeconds(9.5)

                //deposit
                .strafeToLinearHeading(new Vector2d(15, 8), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(30, 8), Math.toRadians(45))
                .waitSeconds(.5)
                //for no pixels change to 950
                .splineToLinearHeading(new Pose2d(52, 32, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)

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

    public static void MTI(RoadRunnerBotEntity myBot) {
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(16, 62, Math.toRadians(-90)))
                .splineToLinearHeading(new Pose2d(40, 26, Math.toRadians(0)), Math.toRadians(0))
                .setTangent(0)

                //Place Preload on board
                .splineToLinearHeading(new Pose2d(52.25, 33.25, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(.1)
                .strafeToLinearHeading(new Vector2d(-20, 36), Math.toRadians(0))
                .waitSeconds(0.5)

                .strafeToLinearHeading(new Vector2d(52.25, 33.25), Math.toRadians(0))

                //Park
                .setTangent(Math.toRadians(90))
                .strafeTo(new Vector2d(45, 60))
                .turnTo(Math.toRadians(-90))

                .build());


    }

    public static void redRightMid(RoadRunnerBotEntity myBot) {
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(16, -62, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(34.7, -30.25, Math.toRadians(0)), Math.toRadians(0))
                .setTangent(0)
                //Place Preload on board
                .waitSeconds(.1)
                .strafeTo(new Vector2d(50.75, -26.8))
                .waitSeconds(0.5)

                //Intake 54
                .waitSeconds(0.2)
                .setTangent(Math.toRadians(120))
                .splineToConstantHeading(new Vector2d(30, -8), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-32, -12, Math.toRadians(0)), Math.toRadians(180))

                .waitSeconds(.2)
                .strafeToLinearHeading(new Vector2d(-34, -14), Math.toRadians(0))

                .waitSeconds(0.5)

                //Deposit
                .strafeToLinearHeading(new Vector2d(30, -9), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(50.5, -40, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.5)


                //Park
                .setTangent(Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(45, -55), Math.toRadians(90))
                .build());

    }

    public static void redLeftMid(RoadRunnerBotEntity myBot) {
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-40, -62, Math.toRadians(90)))
                //place purple
                .strafeToLinearHeading(new Vector2d(-34.5, -35), Math.toRadians(90))

                //intake from mid stack
                .strafeToLinearHeading(new Vector2d(-53.1, -27), Math.toRadians(0))


                .waitSeconds(.5)

                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-44.25, -10), Math.toRadians(0))

                .waitSeconds(8)

                //deposit
                .strafeToLinearHeading(new Vector2d(30, -8), Math.toRadians(0))

                //for no pixels change to 950
                .splineToLinearHeading(new Pose2d(52, -31.5, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                .waitSeconds(.5)
                .setTangent(Math.toRadians(-90))
                //Park - Close to other board
                .strafeToLinearHeading(new Vector2d(46, -32), Math.toRadians(90))
                .build());

    }

    public static void blueRightRight(RoadRunnerBotEntity myBot) {
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-36, 62, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(-32, 33), Math.toRadians(-90))
                .setTangent(-180)
                .splineToSplineHeading(new Pose2d(-48, 41, Math.toRadians(0)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-46, 23.5, Math.toRadians(0)), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-52.5, 23.5), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-48, 23.5), Math.toRadians(0))

                .strafeToLinearHeading(new Vector2d(-44.25, 10), Math.toRadians(0))
                //deposit

                .strafeToLinearHeading(new Vector2d(30, 8), Math.toRadians(0))

                .splineToLinearHeading(new Pose2d(54, 29.75, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)

                .strafeToLinearHeading(new Vector2d(54, 34), Math.toRadians(0))
                .setTangent(Math.toRadians(-180))

                .splineToConstantHeading(new Vector2d(24, 11.25), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-32, 11.25), Math.toRadians(0))
                .waitSeconds(.25)


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

    public static void redMidLeft(RoadRunnerBotEntity myBot) {
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-40, -62, Math.toRadians(90)))
                //place purple
                .strafeToLinearHeading(new Vector2d(-35, -34), Math.toRadians(90))

                //intake from mid stack
                .strafeToLinearHeading(new Vector2d(-53.45, -26.25), Math.toRadians(0))

                .strafeToLinearHeading(new Vector2d(-38, -8), Math.toRadians(0))


                .strafeToLinearHeading(new Vector2d(30, -6), Math.toRadians(0))

                .splineToLinearHeading(new Pose2d(51.35, -32.2, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                .strafeToLinearHeading(new Vector2d(55, -10), Math.toRadians(0))

                .build());
    }

    public static void redRightLeft(RoadRunnerBotEntity myBot) {
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-40, -62, Math.toRadians(90)))
                .strafeToSplineHeading(new Vector2d(-37.5, -37.5), Math.toRadians(40))
                .splineToLinearHeading(new Pose2d(-34, -32, Math.toRadians(0)), Math.toRadians(0))


                //intake from mid stack
                .strafeToLinearHeading(new Vector2d(-53.45, -26), Math.toRadians(0))


                .strafeToLinearHeading(new Vector2d(-35, -11), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(30, -9), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(51.45, -39, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                .strafeToLinearHeading(new Vector2d(55, -10), Math.toRadians(0))
                .turnTo(Math.toRadians(-90))
                .build());
    }

    public static void redLeftLeft(RoadRunnerBotEntity myBot) {
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-40, -62, Math.toRadians(90)))
                //place purple
                .strafeToSplineHeading(new Vector2d(-35, -48), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-48, -38, Math.toRadians(90)), Math.toRadians(90))


                //intake from left stack
                .strafeToSplineHeading(new Vector2d(-40, -38), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-34, -15, Math.toRadians(90)), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-34, -11), Math.toRadians(0))


                // .strafeToLinearHeading(new Vector2d(-35, -11), Math.toRadians(0))


                //deposit
                .strafeToLinearHeading(new Vector2d(30, -9), Math.toRadians(0))

                .splineToLinearHeading(new Pose2d(51.45, -39, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)

                .setTangent(Math.toRadians(90))
                //Park - Close to other board
                .strafeToLinearHeading(new Vector2d(55, -10), Math.toRadians(0))
                .turnTo(Math.toRadians(-90))
                .build());
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)

                .build();
        leftBlueTraj(myBot);
        setAnimation(meepMeep, myBot);

    }
}