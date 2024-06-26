package org.firstinspires.ftc.teamcode.auto.NewAuto.AutoBlue.BlueRight;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.auto.AutoSettingsForAll.AutoConstants;
import org.firstinspires.ftc.teamcode.auto.Actions.DispMarker;
import org.firstinspires.ftc.teamcode.auto.Actions.SubsystemActions;

public class AutoBlueRightRight {
    private final RobotHardware robot = RobotHardware.getInstance();

    Action placePreloadAndIntakeTraj, placePreloadTraj, intake43Traj;
    SequentialAction placePreloadAndIntake, depositPreload, intake43;
    private SubsystemActions subsystemActions;


    public AutoBlueRightRight(SubsystemActions subsystemActions) {
        this.subsystemActions = subsystemActions;
        robot.drive.pose = AutoConstants.startPoseBlueRight;

        placePreloadAndIntakeTraj = robot.drive.actionBuilder(robot.drive.pose)
                .splineToLinearHeading(new Pose2d(-55, 30, Math.toRadians(45)), Math.toRadians(180))
                .afterDisp(0, new DispMarker(new Vector2d(-55, 30), robot.drive, subsystemActions.placePreloadAndIntakeAction()))
                .build();

        placePreloadTraj = robot.drive.actionBuilder(new Pose2d(-55, 15, Math.toRadians(45)))
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(-40, 10, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(35, 10, Math.toRadians(0)), Math.toRadians(0))
                .afterDisp(10, subsystemActions.depositAction)
                .splineToLinearHeading(new Pose2d(50, 40, Math.toRadians(0)), Math.toRadians(0))
                .build();

        intake43Traj = robot.drive.actionBuilder(new Pose2d(50, 40, Math.toRadians(0)))
                .setTangent(Math.toRadians(-180))
                .splineToLinearHeading(new Pose2d(24, 9.25, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-36, 10, Math.toRadians(0)), Math.toRadians(-180))
                .strafeToLinearHeading(new Vector2d(-44, 10), Math.toRadians(0))
                .build();
        placePreloadAndIntake = new SequentialAction(
                placePreloadAndIntakeTraj
                //new DispMarker(new Vector2d(-50, 30), robot.drive, subsystemActions.placePreloadAndIntakeAction())
                //subsystemActions.placePreloadAndIntakeAction()
        );

        depositPreload = new SequentialAction(
                placePreloadTraj
                //  new DispMarker(new Vector2d(40, 40), robot.drive, subsystemActions.depositAction)
        );

        intake43 = new SequentialAction(
                intake43Traj,
                new DispMarker(new Vector2d(-36, 10), robot.drive, subsystemActions.intake43OpenAction),
                new DispMarker(new Vector2d(-44, 10), robot.drive, subsystemActions.intake5CloseAction)
        );

    }


    public Action generateTraj() {

        return new SequentialAction(
                placePreloadAndIntake,
                depositPreload
        );
    }
}
