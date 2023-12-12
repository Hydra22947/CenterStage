package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.PIDFController;


@Autonomous(name="Test Auto", group="tests")
@Config
public class TestAuto extends LinearOpMode {

    private RobotHardware robot = RobotHardware.getInstance();

    PIDFController xController, yController, hController;

    PIDFController.PIDCoefficients xCoefficients, yCoefficients, hCoefficients;

    MecanumDrive drive;
    public static double kPx = 0, kIx = 0, kDx = 0;
    public static double kPy = 0.0001, kIy = 0, kDy = 0;
    public static double kPh = 0, kIh = 0, kDh = 0;

    public static double x = 0, y = 10, h = 0;

    public static double maxPower = 1;
    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap, telemetry);

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0), robot);

        xCoefficients = new PIDFController.PIDCoefficients();
        xCoefficients.kP = kPx;
        xCoefficients.kI = kIx;
        xCoefficients.kD = kDx;

        xController = new PIDFController(xCoefficients);
        xController.setOutputBounds(-maxPower, maxPower);

        yCoefficients = new PIDFController.PIDCoefficients();
        yCoefficients.kP = kPy;
        yCoefficients.kI = kIy;
        yCoefficients.kD = kDy;

        yController = new PIDFController(yCoefficients);
        yController.setOutputBounds(-maxPower, maxPower);

        hCoefficients = new PIDFController.PIDCoefficients();
        hCoefficients.kP = kPh;
        hCoefficients.kI = kIh;
        hCoefficients.kD = kDh;

        hController = new PIDFController(hCoefficients);
        hController.setOutputBounds(-maxPower, maxPower);

        waitForStart();

        while (opModeIsActive()) {
            drive.updatePoseEstimate();

            xController.updateError(x - drive.pose.position.x);
            yController.updateError(y - drive.pose.position.y);
            hController.updateError(AngleUnit.normalizeRadians(h - drive.pose.heading.log()));

            robot.dtFrontLeftMotor.setPower(xController.update() + hController.update() + yController.update());
            robot.dtBackLeftMotor.setPower(xController.update() + hController.update() - yController.update());
            robot.dtFrontRightMotor.setPower(xController.update()  - hController.update() - yController.update());
            robot.dtBackRightMotor.setPower(xController.update()  - hController.update() + yController.update());

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", Math.toDegrees(drive.pose.heading.log()));
            telemetry.update();
        }
    }
}
