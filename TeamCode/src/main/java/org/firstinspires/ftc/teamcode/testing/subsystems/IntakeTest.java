package org.firstinspires.ftc.teamcode.testing.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeExtensionCommand;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;

@Config
@TeleOp(name = "Intake Test")
public class IntakeTest extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    Intake intake;
    IntakeExtension intakeExtension;
    GamepadEx gamepadEx;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        gamepadEx = new GamepadEx(gamepad1);
        robot.init(hardwareMap, telemetry);

        intake = new Intake();
        intakeExtension = new IntakeExtension();

        robot.addSubsystem(intake, intakeExtension);

        gamepadEx.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new SequentialCommandGroup(
                        new IntakeCommand(intake, Intake.Angle.INTAKE)
                ));

        gamepadEx.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new SequentialCommandGroup(
                        new IntakeCommand(intake, Intake.Angle.TRANSFER)
                ));

        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new SequentialCommandGroup(
                        new InstantCommand(() -> intake.setForward(true)),
                        new InstantCommand(() -> intake.setShouldIntake(true))
                ));

        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new SequentialCommandGroup(
                        new InstantCommand(() -> intake.setForward(false)),
                        new InstantCommand(() -> intake.setShouldIntake(true))
                ));

        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whileHeld(new SequentialCommandGroup(
                        new IntakeExtensionCommand(intakeExtension, IntakeExtension.ExtensionState.OPEN)
                ));

        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whileHeld(new SequentialCommandGroup(
                        new IntakeExtensionCommand(intakeExtension, IntakeExtension.ExtensionState.CLOSE)
                ));
    }

    @Override
    public void run() {
        if(!gamepad1.right_bumper || !gamepad1.left_bumper)
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