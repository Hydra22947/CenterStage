package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
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
        drivetrain = new MecanumDrive(hardwareMap, AutoConstants.startPoseLeft, robot);

        Action goPlaceYellowPixel = drivetrain.actionBuilder(AutoConstants.startPoseLeft)
                //Place purple pixel
                .lineToY(AutoConstants.strafeLeftToLine)
                .waitSeconds(AutoConstants.WAIT_TIME)
                //.lineToY(AutoConstants.strafeLeftToStageDoor)
                .waitSeconds(AutoConstants.WAIT_TIME)

                .build();
        Action  goScorePreload = drivetrain.actionBuilder(drivetrain.pose)

                .splineToConstantHeading(AutoConstants.stageDoorVector2, Math.toRadians(0))
                .splineToLinearHeading(AutoConstants.placePixelPose, Math.toRadians(0))
                .waitSeconds(AutoConstants.WAIT_TIME)
                .build();

        Action  goIntake = drivetrain.actionBuilder(drivetrain.pose)
                .strafeToSplineHeading(AutoConstants.stageDoorVector1, Math.toRadians(0))
                .splineToConstantHeading(AutoConstants.intakePixelVector, Math.toRadians(180))
                .waitSeconds(AutoConstants.WAIT_TIME)
                .build();

        Action  goScoreAndPark = drivetrain.actionBuilder(drivetrain.pose)
                .waitSeconds(AutoConstants.WAIT_TIME)
                .splineToConstantHeading(AutoConstants.stageDoorVector2, Math.toRadians(0))
                .splineToLinearHeading(AutoConstants.placePixelPose, Math.toRadians(0))
                .build();

        waitForStart();
        if (isStopRequested()) return;
        Actions.runBlocking(goPlaceYellowPixel);
        Actions.runBlocking(goScorePreload);
        Actions.runBlocking(goIntake);
        Actions.runBlocking(goScoreAndPark);

        AutoConstants.currentPose = drivetrain.pose;
        while(opModeIsActive());
    }
}

