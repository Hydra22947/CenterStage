package org.firstinspires.ftc.teamcode.testing.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;

@Config
@TeleOp(name = "Drivetrain Test", group = "tests")
public class DrivetrainTest extends LinearOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    Drivetrain drivetrain;

    BetterGamepad gamepadEx;

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        gamepadEx = new BetterGamepad(gamepad1);

        robot.init(hardwareMap, telemetry, false);

        drivetrain = new Drivetrain(gamepad1, true);
        drivetrain.resetAngle();

        while (opModeInInit()) {
            telemetry.addLine("Robot Initialized.");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive())
        {
            gamepadEx.update();

            if(gamepadEx.XOnce())
            {
                drivetrain.resetAngle();
            }

            drivetrain.update();
            telemetry.update();
        }
    }

}
