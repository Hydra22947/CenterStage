package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
public class LeftAutoDriveActions {
    private RobotHardware robot;
    private Markers markers;

    private MecanumDrive drivetrain;
    public LeftAutoDriveActions(HardwareMap hardwareMap)
    {
        this.robot = RobotHardware.getInstance();
        this.markers = new Markers();

       this.robot.drivetrain = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0), robot);
    }


    // TODO: add red trajectorys , shouldnt take long.
     Action goPlaceYellowPixel = drivetrain.actionBuilder(AutoConstants.startPoseLeft)
            //Place purple pixel
            .lineToX(AutoConstants.strafeLeftToLine)
            .waitSeconds(AutoConstants.WAIT_TIME)
            .lineToX(AutoConstants.strafeLeftToStageDoor)
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


}
