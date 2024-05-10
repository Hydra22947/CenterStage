package org.firstinspires.ftc.teamcode.testing.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;

@Config
@TeleOp(name = "DrivetrainSubsystem Test", group = "A")
public class DrivetrainTest extends LinearOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    DrivetrainSubsystem drivetrain;

    BetterGamepad gamepadEx;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        gamepadEx = new BetterGamepad(gamepad1);

        robot.init(hardwareMap, telemetry);

        drivetrain = new DrivetrainSubsystem(gamepad1);

        while (opModeInInit()) {
            telemetry.addLine("Robot Initialized.");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive())
        {
            gamepadEx.update();


            drivetrain.update();
            telemetry.update();
        }
    }

}
