package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "Auto Left Red", group = "auto")
public class auto extends LinearOpMode {
    LeftAutoDriveActions leftDriveActions = new LeftAutoDriveActions();
    RobotHardware robot = RobotHardware.getInstance();
    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        if (isStopRequested()) return;
        Actions.runBlocking(leftDriveActions.goPlaceYellowPixel);
        Actions.runBlocking(leftDriveActions.goScorePreload);
        Actions.runBlocking(leftDriveActions.goIntake);
        Actions.runBlocking(leftDriveActions.goScoreAndPark);

        AutoConstants.currentPose = this.robot.drivetrain.pose;
        while(opModeIsActive());
    }
}

