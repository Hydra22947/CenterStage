package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorIncrementCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.firstinspires.ftc.teamcode.util.Globals;

@Config
@TeleOp(name = "OpMode Red")
public class OpMode extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    Drivetrain drivetrain;
    Elevator elevator;
    Claw claw;

    GamepadEx gamepadEx, gamepadEx2;

    double elevatorTarget = Globals.START_ELEVATOR;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        Globals.IS_AUTO = false;
        Globals.IS_USING_IMU = true;

        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        drivetrain = new Drivetrain(robot, gamepad1, gamepad2, true);
        elevator = new Elevator(hardwareMap, gamepad2);
        claw = new Claw(hardwareMap);

        robot.init(hardwareMap, telemetry);
        robot.addSubsystem(drivetrain, elevator, claw);

        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                    .whenPressed(new SequentialCommandGroup(
                            new ClawCommand(claw, Claw.ClawState.CLOSED, ClawSide.BOTH),
                            new ElevatorCommand(elevator, elevatorTarget),
                            new WaitCommand(250)
                        ));

        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new SequentialCommandGroup(
                        new ClawCommand(claw, Claw.ClawState.OPEN, ClawSide.BOTH),
                        new WaitCommand(100),
                        new ElevatorCommand(elevator, 0)
                ));

        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new SequentialCommandGroup(
                        new ClawCommand(claw, Claw.ClawState.OPEN, ClawSide.LEFT)
                ));

        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new SequentialCommandGroup(
                        new ClawCommand(claw, Claw.ClawState.OPEN, ClawSide.RIGHT)
                ));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new SequentialCommandGroup(
                        new ClawCommand(claw, Claw.ClawState.OPEN, ClawSide.RIGHT)
                ));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new SequentialCommandGroup(
                        new ElevatorIncrementCommand(this, 1)
                ));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new SequentialCommandGroup(
                        new ElevatorIncrementCommand(this, -1)
                ));




        if(gamepadEx.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) || gamepadEx2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER))
        {
            elevatorTarget += Globals.ELEVATOR_INCREMENT;
        }
        else if(gamepadEx.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER) || gamepadEx2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER))
        {
            elevatorTarget -= Globals.ELEVATOR_INCREMENT;
        }

        robot.read();
        while (opModeInInit()) {
            telemetry.addLine("Robot Initialized.");
            telemetry.update();
        }
    }

    @Override
    public void run() {
        robot.read();

        if(gamepad2.left_stick_y != 0)
        {
            elevator.setUsePID(false);
        }
        else
        {
            elevator.setUsePID(true);
        }

        super.run();
        robot.periodic();

        telemetry.update();
        robot.write();
    }

    public void setElevatorTarget(double elevatorTarget)
    {
        this.elevatorTarget = elevatorTarget;
    }

    public double getElevatorTarget()
    {
        return elevatorTarget;
    }
}
