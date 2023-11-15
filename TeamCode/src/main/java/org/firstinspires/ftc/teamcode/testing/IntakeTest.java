package org.firstinspires.ftc.teamcode.testing;

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
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePivotCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakePivot;
import org.firstinspires.ftc.teamcode.util.values.ClawSide;

@Config
@TeleOp(name = "Intake Test")
public class IntakeTest extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    Intake intake;
    IntakePivot intakePivot;
    GamepadEx gamepadEx;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        gamepadEx = new GamepadEx(gamepad1);
        robot.init(hardwareMap, telemetry);

        intake = new Intake();
        intakePivot = new IntakePivot();

        robot.addSubsystem(intake, intakePivot);

        gamepadEx.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new SequentialCommandGroup(
                        new IntakePivotCommand(intakePivot, IntakePivot.Angle.INTAKE, IntakePivot.Type.HAND),
                        new IntakePivotCommand(intakePivot, IntakePivot.Angle.INTAKE, IntakePivot.Type.AMMO)
                ));

        gamepadEx.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new SequentialCommandGroup(
                        new IntakePivotCommand(intakePivot, IntakePivot.Angle.MID, IntakePivot.Type.HAND),
                        new IntakePivotCommand(intakePivot, IntakePivot.Angle.MID, IntakePivot.Type.AMMO)
                ));

        gamepadEx.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new SequentialCommandGroup(
                        new IntakePivotCommand(intakePivot, IntakePivot.Angle.TRANSFER, IntakePivot.Type.HAND),
                        new IntakePivotCommand(intakePivot, IntakePivot.Angle.TRANSFER, IntakePivot.Type.AMMO)
                ));

        gamepadEx.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new SequentialCommandGroup(
                        new IntakePivotCommand(intakePivot, IntakePivot.Angle.TRANSFER, IntakePivot.Type.HAND),
                        new IntakePivotCommand(intakePivot, IntakePivot.Angle.TRANSFER, IntakePivot.Type.AMMO)
                ));

        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new SequentialCommandGroup(
                        new IntakeCommand(intake, 1)
                ));

        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new SequentialCommandGroup(
                        new IntakeCommand(intake, -1)
                ));

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
/*
package org.firstinspires.ftc.teamcode.testing;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.commands.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorCommand;
import org.firstinspires.ftc.teamcode.commands.HandCommand;
//import org.firstinspires.ftc.teamcode.commands.SensorCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Hand;
import org.firstinspires.ftc.teamcode.util.values.ClawSide;
import org.firstinspires.ftc.teamcode.util.values.HandPoints;

@Config
@TeleOp(name = "Claw Test")
public class ClawTest extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    public static long delayTillSensor = 1000;
    Claw claw;
    GamepadEx gamepadEx;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        gamepadEx = new GamepadEx(gamepad1);
        robot.init(hardwareMap, telemetry);

        claw = new Claw();

        robot.addSubsystem(claw);

        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new SequentialCommandGroup(
                        new ClawCommand(claw, Claw.ClawState.CLOSED, ClawSide.BOTH).withTimeout(delayTillSensor)
                        // new WaitCommand((long)delayTillSensor)
                ));

        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new SequentialCommandGroup(
                        //new SensorCommand(claw, false),
                        new ClawCommand(claw, Claw.ClawState.OPEN, ClawSide.BOTH)
                        //new WaitCommand((long)delayTillSensor)
                ));
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
*/