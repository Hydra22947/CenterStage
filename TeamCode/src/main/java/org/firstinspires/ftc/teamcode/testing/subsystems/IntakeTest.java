package org.firstinspires.ftc.teamcode.testing.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Config
@TeleOp(name = "Intake Test")
public class IntakeTest extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    Intake intake;
    GamepadEx gamepadEx;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        gamepadEx = new GamepadEx(gamepad1);
        robot.init(hardwareMap, telemetry);

        intake = new Intake();

        robot.addSubsystem(intake);

        gamepadEx.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new SequentialCommandGroup(
                        new IntakeCommand(intake, Intake.Angle.INTAKE)
                ));

        gamepadEx.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new SequentialCommandGroup(
                        new IntakeCommand(intake, Intake.Angle.MID)
                ));

        gamepadEx.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new SequentialCommandGroup(
                        new IntakeCommand(intake, Intake.Angle.TRANSFER)
                ));
    }

    @Override
    public void run() {
        if(gamepad1.right_bumper)
        {
            intake.setForward(true);
            intake.setShouldIntake(true);
        }
        else if(gamepad1.left_bumper)
        {
            intake.setForward(false);
            intake.setShouldIntake(true);
        }
        else
        {
            intake.setShouldIntake(false);
        }
        robot.read();

        super.run();
        robot.periodic();
        telemetry.update();
        robot.write();
    }

}