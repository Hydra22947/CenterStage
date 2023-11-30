package org.firstinspires.ftc.teamcode;

import com.ThermalEquilibrium.homeostasis.Utils.Timer;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.ElevatorCommand;
import org.firstinspires.ftc.teamcode.commands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.ClawSide;

@Config
@TeleOp(name = "OpMode Red")
public class OpMode extends CommandOpMode {

    // robot
    private final RobotHardware robot = RobotHardware.getInstance();

    // subsystems
    Drivetrain drivetrain;
    Elevator elevator;
    Intake intake;
    Outtake outtake;
    Claw claw;
    IntakeExtension intakeExtension;

    // gamepads
    GamepadEx gamepadEx, gamepadEx2;
    BetterGamepad betterGamepad1, betterGamepad2;

    // variables
    double clawPassed = 0;
    double elevatorReset = 0;
    double previousElevator = 0;
    int openedXTimes = 0;
    boolean retract = false;
    public enum IntakeState {
        RETRACT,
        INTAKE,
        INTAKE_EXTEND
    }

    public enum LiftState {
        RETRACT,
        EXTRACT
    }

    IntakeState intakeState = IntakeState.RETRACT;
    LiftState liftState = LiftState.RETRACT;

    @Override
    public void initialize() {
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
        claw = new Claw(this);
        intake = new Intake();
        intakeExtension = new IntakeExtension(gamepad1);

        while (opModeInInit())
        {
            intake.setAngle(Intake.Angle.TRANSFER);
            intakeExtension.setCurrent(IntakeExtension.ExtensionState.MANUAL);
            claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);
            outtake.setAngle(Outtake.Angle.INTAKE);

            telemetry.addLine("Initialized");
            telemetry.update();

            intake.setManual(true);
        }
    }

    @Override
    public void run() {
        betterGamepad1.update();
        betterGamepad2.update();
        intake.update();
        //intakeExtension.update();
        drivetrain.update();
        claw.update();
        outtake.update();

        if (getTime() - clawPassed >= Globals.delayClaw) {
            claw.setShouldOpen(false);
        }

        if (gamepad2.left_stick_y != 0) {
            elevator.setUsePID(false);
        } else {
            elevator.setUsePID(true);
        }

        switch (intakeState) {
            case RETRACT:
                intake.move(Intake.Angle.TRANSFER);

                if (betterGamepad1.rightBumperOnce() && !robot.isReadyToRetract()) {
                    intakeState = IntakeState.INTAKE;
                } else if (gamepad1.right_trigger != 0 && !robot.isReadyToRetract()) {
                    intakeState = IntakeState.INTAKE_EXTEND;
                }

                if (gamepad1.left_trigger != 0) {
                    intake.intakeMove(-gamepad1.left_trigger);
                } else if (robot.isReadyToRetract()) {
                    intake.intakeMove(Intake.powerTransfer);
                } else {
                    intake.intakeMove(0);
                }

                break;
            case INTAKE:
                intake.move(Intake.Angle.INTAKE);

                if (gamepad1.left_trigger != 0) {
                    intake.intakeMove(-gamepad1.left_trigger);
                } else {
                    intake.intakeMove(intake.power);
                }

                if (gamepad1.right_trigger != 0) {
                    intakeState = IntakeState.INTAKE_EXTEND;
                } else if (betterGamepad1.rightBumperOnce()) {
                    intakeState = IntakeState.RETRACT;
                }

                if (robot.isReadyToRetract()) {
                    intakeState = IntakeState.RETRACT;
                }
                break;
            case INTAKE_EXTEND:
                intake.intakeMove(gamepad1.right_trigger);
                intake.move(Intake.Angle.INTAKE);

                if (gamepad1.right_trigger == 0) {
                    intakeState = IntakeState.RETRACT;
                }
                break;
            default:
                intakeState = IntakeState.RETRACT;
                break;
        }

        switch (liftState) {
            case RETRACT:
                elevator.setTarget(0);
                outtake.setAngle(Outtake.Angle.INTAKE);

                if (betterGamepad1.YOnce()) {
                    previousElevator = getTime();
                    liftState = LiftState.EXTRACT;
                }
                break;
            case EXTRACT:
                elevator.setTarget(Elevator.BASE_LEVEL + (openedXTimes * Globals.ELEVATOR_INCREMENT));

                if ((getTime() - previousElevator) >= Globals.WAIT_DELAY_TILL_OUTTAKE) {
                    outtake.setAngle(Outtake.Angle.OUTTAKE);
                }

                if (betterGamepad1.AOnce()) {
                    openedXTimes++;
                    claw.setShouldOpen(true);
                    clawPassed = getTime();
                    elevatorReset = getTime();
                    retract = true;
                } else if ((getTime() - elevatorReset) >= Globals.WAIT_DELAY_TILL_CLOSE && retract) {
                    retract = false;
                    liftState = LiftState.RETRACT;
                }
                break;
            default:
                liftState = LiftState.RETRACT;
                break;
        }
        telemetry.update();

        CommandScheduler.getInstance().run();
    }



    double getTime()
    {
        return System.nanoTime() / 1000000;
    }


}
