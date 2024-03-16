package org.firstinspires.ftc.teamcode.testing.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;

@Config
@TeleOp(name = "Drivetrain Test", group = "A")
public class DrivetrainTest extends LinearOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    Drivetrain drivetrain;

    BetterGamepad gamepadEx;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        gamepadEx = new BetterGamepad(gamepad1);

        robot.init(hardwareMap, telemetry);

        drivetrain = new Drivetrain(gamepad1, true);

        while (opModeInInit()) {
            telemetry.addLine("Robot Initialized.");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive())
        {
            gamepadEx.update();

            telemetry.addData("x", robot.drive.pose.position.x);
                telemetry.addData("y", robot.drive.pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(robot.drive.pose.heading.toDouble()));
                telemetry.update();

            robot.drive.updatePoseEstimate();
            drivetrain.update();
            telemetry.update();
        }
    }

}
