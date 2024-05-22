package org.firstinspires.ftc.teamcode.auto.NewAuto.AutoBlue;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.auto.NewAuto.DispMarker;
import org.firstinspires.ftc.teamcode.auto.NewAuto.SubsystemActions;

public class AutoBlueRightRight {
    private final RobotHardware robot = RobotHardware.getInstance();

    private SubsystemActions subsystemActions;

    public AutoBlueRightRight(SubsystemActions subsystemActions) {
        this.subsystemActions = subsystemActions;
    }

    Action placePreloadAndIntakeTraj = robot.drive.actionBuilder(robot.drive.pose)
            .splineToLinearHeading(new Pose2d(-55, 15, Math.toRadians(45)), Math.toRadians(180))
            .build();

    Action placePreloadTraj = robot.drive.actionBuilder(new Pose2d(-55, 15, Math.toRadians(45)))
            .setTangent(0)
            .splineToSplineHeading(new Pose2d(-40, 10, Math.toRadians(0)), Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(35, 10, Math.toRadians(0)), Math.toRadians(0))
            .splineToLinearHeading(new Pose2d(50, 40, Math.toRadians(0)), Math.toRadians(0))
            .build();

    Action intake43Traj = robot.drive.actionBuilder(new Pose2d(50, 40, Math.toRadians(0)))
            .setTangent(Math.toRadians(-180))
            .splineToLinearHeading(new Pose2d(24, 9.25, Math.toRadians(0)), Math.toRadians(180))
            .splineToLinearHeading(new Pose2d(-36, 10, Math.toRadians(0)), Math.toRadians(-180))
            .strafeToLinearHeading(new Vector2d(-44, 10), Math.toRadians(0))
            .build();
    ParallelAction placePreloadAndIntake = new ParallelAction(
            placePreloadAndIntakeTraj,
            new DispMarker(new Vector2d(-40, 20), robot.drive, subsystemActions.placePreloadAndIntakeAction)
    );

    ParallelAction depositPreload = new ParallelAction(
            placePreloadTraj,
            new DispMarker(new Vector2d(50, 40), robot.drive, subsystemActions.depositAction)
    );

    ParallelAction intake43 = new ParallelAction(
            intake43Traj,
            new DispMarker(new Vector2d(-36, 10), robot.drive, subsystemActions.intake43OpenAction),
            new DispMarker(new Vector2d(-44,10), robot.drive, subsystemActions.intake5CloseAction)
    );

    public Action generateTraj() {
        return new SequentialAction(
                placePreloadAndIntake,
                depositPreload


        );
    }
}
