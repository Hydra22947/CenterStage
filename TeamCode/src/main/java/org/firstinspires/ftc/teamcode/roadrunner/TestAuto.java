package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

    private RobotHardware robot;
    ThreeDeadWheelLocalizer localizer;

    PIDFController xController, yController, hController;

    PIDFController.PIDCoefficients xCoefficients, yCoefficients, hCoefficients;

    Twist2dDual<Time> pos;
    public static double kPx = 0, kIx = 0, kDx = 0;
    public static double kPy = 0, kIy = 0, kDy = 0;
    public static double kPh = 0, kIh = 0, kDh = 0;

    public static double x = 0, y = 10, h = 0;

    public static double maxPower = 1;
    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap, telemetry);

        this.robot = RobotHardware.getInstance();

        localizer = new ThreeDeadWheelLocalizer(hardwareMap, MecanumDrive.PARAMS.inPerTick);

        xCoefficients.kP = kPx;
        xCoefficients.kI = kIx;
        xCoefficients.kD = kDx;

        xController = new PIDFController(xCoefficients);
        xController.setOutputBounds(-maxPower, maxPower);

        yCoefficients.kP = kPy;
        yCoefficients.kI = kIy;
        yCoefficients.kD = kDy;

        yController = new PIDFController(yCoefficients);
        yController.setOutputBounds(-maxPower, maxPower);

        hCoefficients.kP = kPh;
        hCoefficients.kI = kIh;
        hCoefficients.kD = kDh;

        hController = new PIDFController(hCoefficients);
        hController.setOutputBounds(-maxPower, maxPower);

        waitForStart();

        while (opModeIsActive()) {
            pos = localizer.update();

            xController.updateError(x - pos.line.x.value());
            yController.updateError(y - pos.line.y.value());
            hController.updateError(AngleUnit.normalizeRadians(h - pos.angle.value()));

            robot.dtFrontLeftMotor.setPower(xController.update() + hController.update() + yController.update());
            robot.dtBackLeftMotor.setPower(xController.update() + hController.update() - yController.update());
            robot.dtFrontRightMotor.setPower(xController.update()  - hController.update() - yController.update());
            robot.dtBackRightMotor.setPower(xController.update()  - hController.update() + yController.update());

            telemetry.addData("x", pos.line.x);
            telemetry.addData("y", pos.line.y);
            telemetry.addData("heading", Math.toDegrees(pos.angle.value()));
            telemetry.update();
        }
    }
}
