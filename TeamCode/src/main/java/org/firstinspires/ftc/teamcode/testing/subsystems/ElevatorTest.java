package org.firstinspires.ftc.teamcode.testing.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpMode;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.commands.ElevatorCommand;
import org.firstinspires.ftc.teamcode.commands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.ClawSide;

@Config
@TeleOp(name = "Elevator Test")
public class ElevatorTest extends LinearOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    Elevator elevator;
    BetterGamepad gamepadEx;

    enum Lift
    {
        RETRACT, EXCTRACT
    }

    Lift liftState = Lift.RETRACT;
    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();

        gamepadEx = new BetterGamepad(gamepad1);

        robot.init(hardwareMap, telemetry);

        elevator = new Elevator(gamepad1);

        waitForStart();

        while (opModeIsActive())
        {
            if(gamepad2.left_stick_y != 0)
            {
                elevator.setUsePID(false);
            }
            else
            {
                elevator.setUsePID(true);
            }

            switch (liftState) {
                case RETRACT:
                    elevator.setTarget(0);

                    if (gamepadEx.YOnce()) {
                        liftState = Lift.EXCTRACT;
                    }
                    break;
                case EXCTRACT:
                    elevator.setTarget(Elevator.BASE_LEVEL);

                    if (gamepadEx.AOnce()) {
                        liftState = Lift.RETRACT;
                    }
                    break;
                default:
                    liftState = Lift.RETRACT;
                    break;
            }

            elevator.update();
            telemetry.update();
        }
    }

}