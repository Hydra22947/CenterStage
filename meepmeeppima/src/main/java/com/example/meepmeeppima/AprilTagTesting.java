package com.example.meepmeeppima;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NotNull;

import java.nio.file.Path;
import java.util.Arrays;

import sun.rmi.server.Dispatcher;

public class AprilTagTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot
                = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(90), 12.3139452958)
                .build();


        path(myBot);

        setAnimation(meepMeep , myBot);


    }

        public static void setAnimation(MeepMeep meepMeep, RoadRunnerBotEntity redLeftBot) {
            meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(redLeftBot)
                    //  .addEntity(redRightBot)
                    .start();
        }

    static VelConstraint baseVelConstraint = new VelConstraint() {
        @Override
        public double maxRobotVel(@NotNull Pose2dDual<Arclength> pose2dDual, @NotNull PosePath posePath, double v) {
            if (pose2dDual.position.x.value() > 32 && pose2dDual.position.x.value() < 35) {
                return 5;
            } else {
                return 50.0;
            }
        }
    };

    static AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-10.0, 25.0);
    public static void path( RoadRunnerBotEntity myBot) {

//
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d( -47.5, 44.5, Math.toRadians(15)))
//                .setTangent(Math.toRadians(90))
//                .splineToSplineHeading(new Pose2d(-32, 58, Math.toRadians(0)), Math.toRadians(0))
//                .setTangent(Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(22, 58), Math.toRadians(0))
//                .setTangent(Math.toRadians(0))
//                .splineToSplineHeading(new Pose2d(50.5, 40, Math.toRadians(0)), Math.toRadians(20), baseVelConstraint)
//                .build());

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-40, 62, Math.toRadians(-90)))

                .strafeToLinearHeading(new Vector2d(-44.25, 10), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(15, 8), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(31, 20, Math.toRadians(24.5)), Math.toRadians(60),
                        baseVelConstraint, baseAccelConstraint)




                // detect

                .splineToLinearHeading(new Pose2d(52, 32, Math.toRadians(0)), Math.toRadians(0))

                .waitSeconds(.5)
                .build());




    }



}


