package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "Auto Left Red", group = "auto")
public class auto extends LinearOpMode {
    RobotHardware robot = RobotHardware.getInstance();
    MecanumDrive drivetrain;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        drivetrain = new MecanumDrive(AutoConstants.startPoseLeft, robot);

        Action test = drivetrain.actionBuilder(AutoConstants.startPoseLeft)
                .lineToY(47)
                .waitSeconds(.5)
                //Going for backdrop
                .splineTo(new Vector2d(0, -12), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(45, -33, Math.toRadians(0)), Rotation2d.exp(Math.toRadians(0)))
                //Going for intake
                .waitSeconds(1)
                .lineToY(30)
                .splineToConstantHeading(new Vector2d(-34, -12), Math.toRadians(180))
                .build();
//
//
//        Action goPlaceYellowPixel = drivetrain.actionBuilder(AutoConstants.startPoseLeft)
//                //Place purple pixel
//                .lineToY(AutoConstants.strafeLeftToLine)
//                .waitSeconds(AutoConstants.WAIT_TIME)
//                //.lineToY(AutoConstants.strafeLeftToStageDoor)
//                .waitSeconds(AutoConstants.WAIT_TIME)
//
//                .build();
//        Action  goScorePreload = drivetrain.actionBuilder(drivetrain.pose)
//
//                .splineToConstantHeading(AutoConstants.stageDoorVector2, Math.toRadians(0))
//                .splineToLinearHeading(AutoConstants.placePixelPose, Math.toRadians(0))
//                .waitSeconds(AutoConstants.WAIT_TIME)
//                .build();
//
//        Action  goIntake = drivetrain.actionBuilder(drivetrain.pose)
//                .strafeToSplineHeading(AutoConstants.stageDoorVector1, Math.toRadians(0))
//                .splineToConstantHeading(AutoConstants.intakePixelVector, Math.toRadians(180))
//                .waitSeconds(AutoConstants.WAIT_TIME)
//                .build();
//
//        Action  goScoreAndPark = drivetrain.actionBuilder(drivetrain.pose)
//                .waitSeconds(AutoConstants.WAIT_TIME)
//                .splineToConstantHeading(AutoConstants.stageDoorVector2, Math.toRadians(0))
//                .splineToLinearHeading(AutoConstants.placePixelPose, Math.toRadians(0))
//                .build();

        waitForStart();
        if (isStopRequested()) return;
//        Actions.runBlocking(goPlaceYellowPixel);
//        Actions.runBlocking(goScorePreload);
//        Actions.runBlocking(goIntake);
//        Actions.runBlocking(goScoreAndPark);

        Actions.runBlocking(test);
        AutoConstants.currentPose = drivetrain.pose;
        while(opModeIsActive());
    }
}

