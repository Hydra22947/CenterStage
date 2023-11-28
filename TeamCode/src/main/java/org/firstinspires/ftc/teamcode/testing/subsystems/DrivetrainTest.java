package org.firstinspires.ftc.teamcode.testing.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Globals;

@Config
@TeleOp(name = "Drivetrain Test", group = "tests")
public class DrivetrainTest extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    Drivetrain drivetrain;

    GamepadEx gamepadEx, gamepadEx2;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        //telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        Globals.IS_AUTO = false;
        Globals.IS_USING_IMU = true;

        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        robot.init(hardwareMap, telemetry);

        drivetrain = new Drivetrain(gamepad1, true);
        drivetrain.resetAngle();
        robot.addSubsystem(drivetrain);

        robot.read();

        gamepadEx.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> drivetrain.resetAngle()));

        while (opModeInInit()) {
            telemetry.addLine("Robot Initialized.");
            telemetry.update();
        }
    }

    @Override
    public void run() {
        robot.read();

        super.run();
        robot.periodic();

        telemetry.update();
        robot.write();
    }

}
