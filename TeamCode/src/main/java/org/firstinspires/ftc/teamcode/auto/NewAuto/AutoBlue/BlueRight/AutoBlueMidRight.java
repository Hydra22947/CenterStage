package org.firstinspires.ftc.teamcode.auto.NewAuto.AutoBlue.BlueRight;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.auto.Actions.SubsystemActions;

public class AutoBlueMidRight {
    private final RobotHardware robot = RobotHardware.getInstance();

    SubsystemActions systemActions;

    public AutoBlueMidRight()
    {
        //systemActions = new SubsystemActions();
    }

    Action placePreloadAndIntakeTraj= robot.drive.actionBuilder(robot.drive.pose)
            .splineToLinearHeading(new Pose2d(-55, 15, Math.toRadians(45)), Math.toRadians(180))

            .build();
    Action intake5TrajLeft = robot.drive.actionBuilder(new Pose2d(-33, 28, Math.toRadians(0)))
            //  .splineToSplineHeading(new Pose2d(-40, 40, Math.toRadians(-40)), Math.toRadians(180))
            //   .splineToLinearHeading(new Pose2d(-45.5, 22, Math.toRadians(0)), Ma
            //   th.toRadians(180))

            .strafeToLinearHeading(new Vector2d(-45.5, 22), Math.toRadians(0))
           // .afterTime(0, systemActions.intake43OpenAction)
            .strafeToLinearHeading(new Vector2d(-54, 23.8), Math.toRadians(0))
            .stopAndAdd(systemActions.intake5CloseAction)
            .waitSeconds(3.5)
            .build();


        public Action generateTraj()
        {
            return new SequentialAction();
        }
}
