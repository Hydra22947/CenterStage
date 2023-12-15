package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
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
@Config
public class auto extends LinearOpMode {
    RobotHardware robot = RobotHardware.getInstance();
    MecanumDrive drivetrain;

    public static double startX = -34, startY = -60, startH = Math.toRadians(90);
    public static double x1 = -34, y1 = -2;
    public static double x2 = -4, y2 = -6;
    public static double x3 = 63, y3 = -60;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        drivetrain = new MecanumDrive(new Pose2d(startX, startY, startH), robot);

        Action test = drivetrain.actionBuilder(new Pose2d(startX, startY, startH))
                .strafeTo(new Vector2d(x1,y1))
                .waitSeconds(.5)
                .strafeTo(new Vector2d(x2, y2))
                .splineToSplineHeading(new Pose2d(x3, y3, Math.toRadians(0)), Rotation2d.exp(Math.toRadians(0)))
                .waitSeconds(1)
                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(test);
        while(opModeIsActive());
    }
}

