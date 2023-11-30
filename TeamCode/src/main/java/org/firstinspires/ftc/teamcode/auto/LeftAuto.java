package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
@Autonomous(name = "Auto Left Red", group = "auto")
public class LeftAuto extends LinearOpMode {

    RobotHardware robot = RobotHardware.getInstance();
    Markers markers = new Markers();


     MecanumDrive drivetrain = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0), robot);

    // TODO: add red trajectorys , shouldnt take long.
     Action placeYellowPixel = drivetrain.actionBuilder(AutoConstants.startPoseLeft)
            //Place purple pixel
            .lineToX(AutoConstants.strafeLeftToLine)
            .waitSeconds(AutoConstants.WAIT_TIME)
            .lineToX(AutoConstants.strafeLeftToStageDoor)
            .waitSeconds(AutoConstants.WAIT_TIME)

            .build();


    Action scorePreload = drivetrain.actionBuilder(drivetrain.pose)

            .splineToConstantHeading(AutoConstants.stageDoorVector2, Math.toRadians(0))
            .splineToLinearHeading(AutoConstants.placePixelPose, Math.toRadians(0))
            .waitSeconds(AutoConstants.WAIT_TIME)
            .build();

    Action intake = drivetrain.actionBuilder(drivetrain.pose)
            .strafeToSplineHeading(AutoConstants.stageDoorVector1, Math.toRadians(0))
            .splineToConstantHeading(AutoConstants.intakePixelVector, Math.toRadians(180))
            .waitSeconds(AutoConstants.WAIT_TIME)
            .build();

    Action scoreAndPark = drivetrain.actionBuilder(drivetrain.pose)
            .waitSeconds(AutoConstants.WAIT_TIME)
            .splineToConstantHeading(AutoConstants.stageDoorVector2, Math.toRadians(0))
            .splineToLinearHeading(AutoConstants.placePixelPose, Math.toRadians(0))
            .build();


    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(placeYellowPixel);
        Actions.runBlocking(scorePreload);
        Actions.runBlocking(intake);
        Actions.runBlocking(scoreAndPark);

        AutoConstants.currentPose = drivetrain.pose;
        while(opModeIsActive());
    }
}
