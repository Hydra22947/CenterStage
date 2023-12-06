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
    boolean canIntake = true;
    //IntakeExtension intakeExtension;

    // gamepads
    GamepadEx gamepadEx, gamepadEx2;
    BetterGamepad betterGamepad1, betterGamepad2;
    public static double transferPower = -1;
    public static double delayTransfer = 900;
    public static double maxTransferTimer = 2750; // make sure maxTransferTimer > delayTransfer

    // variables
    double clawPassed = 0;
    double elevatorReset = 0;
    double previousElevator = 0;
    double transferTimer = 0;
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
    double loopTime = 0;
    boolean closeClaw = false;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        Globals.IS_USING_IMU = true;

        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        betterGamepad1 = new BetterGamepad(gamepad1);
        betterGamepad2 = new BetterGamepad(gamepad2);

        robot.init(hardwareMap, telemetry);

        drivetrain = new Drivetrain(gamepad1, true);
        elevator = new Elevator(gamepad2);
        outtake = new Outtake();
        claw = new Claw(this);
        intake = new Intake();
        //intakeExtension = new IntakeExtension(gamepad1);

        intake.setAngle(Intake.Angle.TRANSFER);
        //intakeExtension.setCurrent(IntakeExtension.ExtensionState.MANUAL);
        claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);
        outtake.setAngle(Outtake.Angle.INTAKE);
        elevator.setAuto(false);

        intake.setManual(true);
        intake.update();
        claw.update();
        outtake.update();

        while (opModeInInit())
        {
            telemetry.addLine("Initialized");
            telemetry.update();
            intake.update();
            claw.update();
            outtake.update();
        }
    }

    @Override
    public void run() {
        //intakeExtension.update();
        betterGamepad1.update();
        betterGamepad2.update();
        drivetrain.update();
        intake.update();
        //claw.update();
        outtake.update();

//        if (getTime() - clawPassed >= Globals.delayClaw) {
//            claw.setShouldOpen(false);
//        }

        if (gamepad2.left_stick_y != 0) {
            elevator.setUsePID(false);
        } else {
            elevator.setUsePID(true);
        }

        intakeStateMachine();
        elevatorStateMachine();

        telemetry.addData("hz ", 1000000000 / (System.nanoTime() - loopTime));
        telemetry.update();
        CommandScheduler.getInstance().run();

        loopTime = System.nanoTime();
    }

    void intakeStateMachine()
    {
        switch (intakeState) {
            case RETRACT:
                intake.move(Intake.Angle.TRANSFER);

                if (betterGamepad1.rightBumperOnce() && !robot.isReadyToRetract() && canIntake) {
                    intakeState = IntakeState.INTAKE;
                } else if (gamepad1.right_trigger != 0 && !robot.isReadyToRetract() && liftState == LiftState.RETRACT && canIntake) {
                    intakeState = IntakeState.INTAKE_EXTEND;
                }
                else if(liftState == LiftState.RETRACT)
                {
                    drivetrain.fast();
                }


                if (gamepad1.left_trigger != 0) {
                    intake.intakeMove(-gamepad1.left_trigger);
                }
                else if(((getTime() - transferTimer >= delayTransfer) && (getTime() - transferTimer <= maxTransferTimer))/*
                        || ((intake.checkIfPixelIn(robot.colorRight) || intake.checkIfPixelIn(robot.colorLeft)))*/)
                {
                    intake.intakeMove(transferPower);
                    closeClaw = true;
                    //claw.overwrite = true;
                }
                else
                {
                    intake.intakeMove(0);
                }


                if(closeClaw && (getTime() - transferTimer >= maxTransferTimer))
                {
                    claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH);
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
                    transferTimer = getTime();
                    intakeState = IntakeState.RETRACT;
                }
//                claw.overwrite = false;
//                claw.setShouldOpen(true);
                closeClaw = false;

                claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);
                break;
            case INTAKE_EXTEND:
//                claw.overwrite = false;
//                claw.setShouldOpen(true);
                intake.intakeMove(gamepad1.right_trigger);
                intake.move(Intake.Angle.INTAKE);

                if (gamepad1.right_trigger == 0) {
                    intakeState = IntakeState.RETRACT;
                }
                claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);
                break;
            default:
                intakeState = IntakeState.RETRACT;
                break;
        }
    }

    void elevatorStateMachine()
    {
        switch (liftState) {
            case RETRACT:
                elevator.setTarget(0);
                outtake.setAngle(Outtake.Angle.INTAKE);

                canIntake = true;

                if (betterGamepad1.YOnce())
                {
                    previousElevator = getTime();
                    claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH);
                    liftState = LiftState.EXTRACT;
                }
                break;
            case EXTRACT:
                canIntake = false;
                intakeState = IntakeState.RETRACT;

                elevator.setTarget(Elevator.BASE_LEVEL + (openedXTimes * Globals.ELEVATOR_INCREMENT));

                if ((getTime() - previousElevator) >= Globals.WAIT_DELAY_TILL_OUTTAKE) {
                    outtake.setAngle(Outtake.Angle.OUTTAKE);
                }

                drivetrain.slow();
                drivetrain.update();
                //claw.overwrite = true;

                if(betterGamepad1.dpadRightOnce())
                {
                    //claw.overwrite = true;
                    claw.updateState(Claw.ClawState.OPEN, ClawSide.RIGHT);
                }
                else if(betterGamepad1.dpadLeftOnce())
                {
                    //claw.overwrite = true;
                    claw.updateState(Claw.ClawState.OPEN, ClawSide.LEFT);
                }


                if (betterGamepad1.AOnce() || betterGamepad1.leftBumperOnce())  {
                    //claw.overwrite = false;
                    openedXTimes++;
                    //claw.setShouldOpen(true);
                    //clawPassed = getTime();
                    claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);

                    elevatorReset = getTime();
                    closeClaw = false;
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
    }


    double getTime()
    {
        return System.nanoTime() / 1000000;
    }


}
