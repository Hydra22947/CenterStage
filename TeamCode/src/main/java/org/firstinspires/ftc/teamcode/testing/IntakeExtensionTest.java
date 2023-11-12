package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.commands.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeExtensionCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.util.values.ClawSide;

@Config
@TeleOp(name = "Extension Test")
public class IntakeExtensionTest extends CommandOpMode {
    private RobotHardware _robot;
    private IntakeExtension _intakeExtension;
    private GamepadEx _gamepadEx;


    @Override
    public void initialize() {

        this._robot = RobotHardware.getInstance();
        this._intakeExtension = new IntakeExtension();
        this._gamepadEx = new GamepadEx(gamepad1);

        this._robot.init(hardwareMap, telemetry);
        this._robot.addSubsystem(this._intakeExtension);

        this._gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new SequentialCommandGroup(
                        new IntakeExtensionCommand(this._intakeExtension, IntakeExtension.ExtensionState.OPEN)
                ));
        this._gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new SequentialCommandGroup(
                        new IntakeExtensionCommand(this._intakeExtension, IntakeExtension.ExtensionState.CLOSE)
                ));

    }

    @Override
    public void run() {
        this._robot.read();

        super.run();
        this._robot.periodic();

        telemetry.update();
        this._robot.write();
    }
}
