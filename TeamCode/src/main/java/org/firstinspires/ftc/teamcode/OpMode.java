package org.firstinspires.ftc.teamcode;

import com.ThermalEquilibrium.homeostasis.Utils.Timer;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.commands.ElevatorCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.values.Globals;

@Config
@TeleOp(name = "OpMode Red")
public class OpMode extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    Drivetrain drivetrain;
    Elevator elevator;
    Intake intake;
    Outtake outtake;
    Claw claw;
    GamepadEx gamepadEx, gamepadEx2;
    BetterGamepad betterGamepad1, betterGamepad2;
    IntakeExtension intakeExtension;

    double elevatorTarget = Globals.START_ELEVATOR;
    boolean rightBumberTrigger = false;
    Timer timer;

    @Override
    public void initialize() {
        timer = new Timer();
        timer.reset();

        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        Globals.IS_AUTO = false;
        Globals.IS_USING_IMU = true;

        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        betterGamepad1 = new BetterGamepad(gamepad1);
        betterGamepad2 = new BetterGamepad(gamepad2);

        robot.init(hardwareMap, telemetry);

        drivetrain = new Drivetrain(gamepad1, true);
        elevator = new Elevator(gamepad2);
        outtake = new Outtake(intake, claw);
        claw = new Claw();
        intake = new Intake();
        intakeExtension = new IntakeExtension(gamepad1);


        robot.addSubsystem(drivetrain, intake, intakeExtension, elevator, outtake, claw);

        gamepadEx.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new SequentialCommandGroup(
                        new InstantCommand(() -> elevator.setUsePID(true)),
                        new ElevatorCommand(elevator, Elevator.BASE_LEVEL),
                        new WaitCommand((long)Globals.WAIT_DELAY_TILL_OUTTAKE),
                        new OuttakeCommand(outtake, Outtake.Angle.OUTTAKE)
                ));

        gamepadEx.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new SequentialCommandGroup(
                        new InstantCommand(() -> elevator.setUsePID(true)),
                        new OuttakeCommand(outtake, Outtake.Angle.INTAKE),
                        new WaitCommand((long)Globals.WAIT_DELAY_TILL_OUTTAKE),
                        new ElevatorCommand(elevator, 0),
                        new InstantCommand(() -> elevatorTarget += Globals.ELEVATOR_INCREMENT)
                ));

        while (opModeInInit())
        {
            intake.setAngle(Intake.Angle.TRANSFER);
            intakeExtension.setCurrent(IntakeExtension.ExtensionState.MANUAL);

            telemetry.addLine("Initialized");
            telemetry.update();

            intake.setManual(true);
        }
    }

    @Override
    public void run() {
        betterGamepad1.update();
        betterGamepad2.update();

        if(betterGamepad1.rightBumperOnce())
        {
            intake.move(Intake.Angle.INTAKE);
            intake.intakeMove(intake.power);
            if(rightBumberTrigger)
            {
                rightBumberTrigger = false;
                intake.move(Intake.Angle.TRANSFER);
                intake.intakeMove(0);
            }
            else
            {
                rightBumberTrigger = true;
            }
        }
        else if(betterGamepad1.leftBumperOnce())
        {
            rightBumberTrigger = false;
            intake.move(Intake.Angle.TRANSFER);
            intake.intakeMove(0);
        }
        else if(gamepad1.right_trigger != 0)
        {
            intake.intakeMove(gamepad1.right_trigger);
            intake.move(Intake.Angle.INTAKE);
            rightBumberTrigger = false;

        }
        else if(gamepad1.left_trigger != 0)
        {
            intake.intakeMove(-gamepad1.left_trigger);
            rightBumberTrigger = false;
        }
        else if(!rightBumberTrigger)
        {
            intake.intakeMove(0);
            intake.setShouldIntake(false);
            intake.move(Intake.Angle.TRANSFER);
        }

        if(gamepad1.left_stick_y != 0)
        {
            elevator.setUsePID(false);
        }
        else
        {
            elevator.setUsePID(true);
        }

        super.run();
        robot.periodic();

        robot.read();

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

    public void setRightBumberTrigger(boolean rightBumberTrigger) {
        this.rightBumberTrigger = rightBumberTrigger;
    }
}
