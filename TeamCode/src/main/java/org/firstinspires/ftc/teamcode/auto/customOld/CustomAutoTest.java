package org.firstinspires.ftc.teamcode.auto.customOld;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.auto.customOld.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;

@Config
@Autonomous(name = "Custom Auto")
public class CustomAutoTest extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private SampleMecanumDrive drivetrain;
    private double loopTime = 0.0;

    public static double x = 21.5, y = -23.25, h = 0;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        robot.init(hardwareMap, telemetry);
        robot.enabled = true;
        drivetrain = new SampleMecanumDrive(hardwareMap);
        drivetrain.setPoseEstimate(new Pose2d());

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.read();
        while (!isStarted()) {
            telemetry.addLine("auto in init");
            telemetry.update();
        }

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        // go to yellow pixel scoring pos
                        new PositionCommand(new Pose(x,y,h),drivetrain),
                        new WaitCommand(350)
                )
        );
    }

    @Override
    public void run() {
        robot.read();
        super.run();
        robot.periodic();
        drivetrain.updatePoseEstimate();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        telemetry.addData("x", drivetrain.getPoseEstimate().getX());
        telemetry.addData("y", drivetrain.getPoseEstimate().getY());
        telemetry.addData("h", drivetrain.getPoseEstimate().getHeading());
        loopTime = loop;
        telemetry.update();

        robot.write();
    }
}
