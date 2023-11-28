package org.firstinspires.ftc.teamcode.testing.subsystems;

import com.ThermalEquilibrium.homeostasis.Utils.Timer;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.commands.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorCommand;
import org.firstinspires.ftc.teamcode.commands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.util.values.ClawSide;
import org.firstinspires.ftc.teamcode.util.values.Globals;

@Config
@TeleOp(name = "Claw Test")
public class OuttakeTest extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    Claw claw;
    GamepadEx gamepadEx;
    Timer timer;
    Outtake outtake;
    Intake intake;
    Elevator elevator;
    double passed = 0;

    @Override
    public void initialize() {
        timer = new Timer();
        timer.reset();
        CommandScheduler.getInstance().reset();

        gamepadEx = new GamepadEx(gamepad1);
        robot.init(hardwareMap, telemetry);

        claw = new Claw();
        intake = new Intake();
        outtake = new Outtake(intake, claw);
        elevator = new Elevator();

        robot.addSubsystem(claw, outtake, elevator, intake);

        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new SequentialCommandGroup(
                        new ElevatorCommand(elevator, Globals.START_ELEVATOR),
                        new OuttakeCommand(outtake, Outtake.Angle.OUTTAKE)
                ));

        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new SequentialCommandGroup(
                        new ElevatorCommand(elevator, 0),
                        new ClawCommand(claw, Claw.ClawState.OPEN, ClawSide.BOTH)
                ));
    }

    @Override
    public void run() {
        if(gamepad1.right_bumper || gamepad1.left_bumper)
        {
            claw.setShouldOpen(true);
            passed = timer.currentTime();
        }

        if(timer.currentTime() - passed >= Globals.delayClaw)
        {
            claw.setShouldOpen(false);
        }

        robot.read();

        super.run();
        robot.periodic();

        telemetry.update();
        robot.write();
    }

}