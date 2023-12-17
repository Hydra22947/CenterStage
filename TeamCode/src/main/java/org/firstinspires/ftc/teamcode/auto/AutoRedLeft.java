package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.auto.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "Auto Red Left")
public class AutoRedLeft extends LinearOpMode
{
    public static final double startX = -34, startY = -60, startHeading = 90;

    Pose2d startPose = new Pose2d(startX, startX, Math.toRadians(startHeading));

    SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);
    TrajectorySequence placePreload = drivetrain.trajectorySequenceBuilder(startPose)
            .build();

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        if (isStopRequested()) return;

        while(opModeIsActive());
    }
}
