package org.firstinspires.ftc.teamcode.roadrunner;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.Pose;


@Autonomous(name="Test Auto", group="tests")
@Config
public class TestAuto extends LinearOpMode {

    public static double ALLOWED_TRANSLATIONAL_ERROR = 0.25;
    public static double ALLOWED_HEADING_ERROR = Math.toRadians(1);

    private RobotHardware robot = RobotHardware.getInstance();

    MecanumDrive drive;
    public static double xP = 0.04, xD = 0.05, xF = 0;

    public static double yP = 0.04,  yD = 0.05, yF = 0;

    public static double hP = 0.6, hD = 0.3, hF = 0;

    public static double x = 0, y = 10, h = 0;

    PIDFController xController = new PIDFController(xP, 0, xD, xF);
    PIDFController yController = new PIDFController(yP, 0, yD, yF);
    PIDFController hController = new PIDFController(hP, 0, hD, hF);

    public static double maxPower = 1;
    public static double maxHeading = .5;
    private double v;

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap, telemetry);

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0), robot);

        waitForStart();

        while (opModeIsActive()) {
            robot.loopVoltage(hardwareMap);
            drive.updatePoseEstimate();

            goToPosition(new Pose(drive.pose.position.x, drive.pose.position.y, drive.pose.heading.log()), new Pose(x,y,h));
            telemetry.addData("x pos", drive.pose.position.x);
            telemetry.addData("y pos", drive.pose.position.y);
            telemetry.addData("heading pos", Math.toDegrees(drive.pose.heading.log()));
            telemetry.update();
        }
    }

//    public boolean isFinished() {
//        Pose error = targetPose.subtract(localizer.getPos());
//
//        boolean reached = ((Math.hypot(error.x, error.y) < ALLOWED_TRANSLATIONAL_ERROR) && (Math.abs(error.heading) < ALLOWED_HEADING_ERROR));
//
//        if (reached && delayTimer == null) {
//            delayTimer = new ElapsedTime();
//        }
//        if (!reached && delayTimer != null) {
//            delayTimer.reset();
//        }
//
//        boolean delayed = delayTimer != null && delayTimer.milliseconds() > delay;
//        return (deadTimer.milliseconds() > ms) || delayed;
//    }


    private static Pose relDistanceToTarget(Pose robot, Pose target) {
        return target.subtract(robot);
    }

    public Pose goToPosition(Pose robotPose, Pose targetPose) {
        Pose deltaPose = relDistanceToTarget(robotPose, targetPose);
        Pose powers = new Pose(
                xController.calculate(0, deltaPose.x),
                yController.calculate(0, deltaPose.y),
                hController.calculate(0, deltaPose.heading)
        );
        double x_rotated = powers.x * Math.cos(robotPose.heading) - powers.y * Math.sin(robotPose.heading);
        double y_rotated = powers.x * Math.sin(robotPose.heading) + powers.y * Math.cos(robotPose.heading);
        double x_power = -x_rotated < -maxPower ? -maxPower :
                Math.min(-x_rotated, maxPower);
        double y_power = -y_rotated < -maxPower ? -maxPower :
                Math.min(-y_rotated, maxPower);
        double heading_power = MathUtils.clamp(powers.heading, -maxHeading, maxHeading);

        if(Math.abs(x_power) < 0.01) x_power = 0;
        if(Math.abs(y_power) < 0.01) y_power = 0;

        return new Pose(-y_power / robot.getVoltage() * 12, x_power / robot.getVoltage() * 12, -heading_power / robot.getVoltage() * 12);
    }
}
