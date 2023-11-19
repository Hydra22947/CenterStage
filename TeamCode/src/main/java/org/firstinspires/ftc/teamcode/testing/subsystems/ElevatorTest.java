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
import org.firstinspires.ftc.teamcode.commands.ElevatorCommand;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.util.values.Globals;

@Config
@TeleOp(name = "Elevator Test")
public class ElevatorTest extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    Elevator elevator;
    GamepadEx gamepadEx;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        gamepadEx = new GamepadEx(gamepad1);
        robot.init(hardwareMap, telemetry);

        Globals.IS_AUTO = false;
        elevator = new Elevator(gamepad1);

        robot.addSubsystem(elevator);

        gamepadEx.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new SequentialCommandGroup(
                        new InstantCommand(() -> elevator.setUsePID(true)),
                        new ElevatorCommand(elevator, Elevator.BASE_LEVEL)
                ));

        gamepadEx.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> elevator.setUsePID(false)));
    }

    @Override
    public void run() {
        if(gamepad1.left_stick_y != 0)
        {
            elevator.setUsePID(false);
        }

        robot.read();

        super.run();
        robot.periodic();
        telemetry.update();
        robot.write();
    }

}