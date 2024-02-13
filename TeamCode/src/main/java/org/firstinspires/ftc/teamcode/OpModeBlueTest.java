package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.ClawSide;

@Config
@TeleOp(name = "DEBUG OpMode Blue")
public class OpModeBlueTest extends LinearOpMode {

    // robot
    private final RobotHardware robot = RobotHardware.getInstance();

    // subsystems
    Drivetrain drivetrain;
    Elevator elevator;
    Intake intake;
    Outtake outtake;
    Claw claw;
    IntakeExtension intakeExtension;
    ElapsedTime codeTime;

    // gamepads
    GamepadEx gamepadEx, gamepadEx2;
    BetterGamepad betterGamepad1, betterGamepad2;

    // delays
    public static double delayTransfer = 300, delayRelease = 750, delayCloseTransfer = 350, delayGoToTransfer = 1300;
    public static double WAIT_DELAY_TILL_OUTTAKE = 0, WAIT_DELAY_TILL_CLOSE = 200, closeClawAgainDelay = 500;
    public static double INTAKE_EXTEND_PRECENTAGE = 60;
    // variables
    double elevatorReset = 0, previousElevator = 0, transferTimer = 0, releaseTimer = 0, closeTransferTimer = 0, goToTransferTimer = 0, closeClawAgainTimer = 0;
    double elevatorTargetRight = 1300;
    double elevatorTargetLeft = 1300;
    int openedXTimes = 0;
    boolean retract = false,  goToMid = false, intakeMid = true, canIntake = true, startedDelayTransfer = false, heldExtension = false;
    boolean override = false, had2Pixels = false, hang = false, resetRightTrigger = true, closeClaw = false, wasClosed = false, firstExtend = true;
    boolean overrideIntakeExtension = false, firstCloseAgain = true;

    public enum IntakeState {
        RETRACT,
        INTAKE,
        INTAKE_EXTEND
    }

    public enum LiftState {
        RETRACT,
        EXTRACT,
        HANG
    }

    enum IntakeLevel
    {
        TOP_54,
        TOP_32,
        INTAKE
    }

    IntakeLevel intakeLevel = IntakeLevel.INTAKE;
    IntakeState intakeState = IntakeState.RETRACT;
    LiftState liftState = LiftState.RETRACT;

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        betterGamepad1 = new BetterGamepad(gamepad1);
        betterGamepad2 = new BetterGamepad(gamepad2);

        robot.init(hardwareMap, telemetry); // TODO: change to false

        drivetrain = new Drivetrain(gamepad1, true);
        elevator = new Elevator(gamepad2, true);
        outtake = new Outtake();
        claw = new Claw();
        intake = new Intake();
        intakeExtension = new IntakeExtension(gamepad2, true);
        codeTime = new ElapsedTime();
        intake.setAngle(Intake.Angle.OUTTAKE);
        intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
        claw.setBothClaw(Claw.ClawState.INTAKE);
        outtake.setAngle(Outtake.Angle.INTAKE);
        elevator.setAuto(false);
        intakeExtension.setAuto(false);

        intake.update();
        claw.update();
        outtake.update();
        intakeExtension.update();

        while (opModeInInit() && !isStopRequested())
        {
            telemetry.addLine("Initialized");
            telemetry.update();
            intake.update();
            claw.update();
            outtake.update();
            intakeExtension.update();
        }

        codeTime.reset();

        waitForStart();

        while (opModeIsActive())
        {
            intakeExtension.update();
            betterGamepad1.update();
            betterGamepad2.update();
            drivetrain.update();
            intake.update();
            intake.updateClawState(intake.getClawStateLeft(), ClawSide.LEFT);
            intake.updateClawState(intake.getClawStateRight(), ClawSide.RIGHT);
            intakeExtension.update();
            outtake.update();
            elevator.update();
            claw.update();


//            if((getTime() - closeClawAgainTimer) >= closeClawAgainDelay && firstCloseAgain)
//            {
//                claw.setBothClaw(Claw.ClawState.CLOSED);
//
//                firstCloseAgain = false;
//            }
            if (liftState != LiftState.EXTRACT && gamepad2.left_stick_y != 0 && !overrideIntakeExtension) {
                intakeExtension.setUsePID(false);
            }
            else if (gamepad2.left_stick_y != 0 && !overrideIntakeExtension) {
                intakeExtension.setUsePID(false);
                //claw.setBothClaw(Claw.ClawState.OPEN);
                //closeClawAgainTimer = getTime();
                //firstCloseAgain = true;

            } else {
                intakeExtension.setUsePID(true);
            }

            if (gamepad2.right_stick_y != 0) {
                elevator.setUsePID(false);
            } else {
                elevator.setUsePID(true);
            }

            changeIntakeLevels();
            intakeStateMachine();
            elevatorStateMachine();

            telemetry.addData("INTKE OVERRIDE", overrideIntakeExtension);
            telemetry.update();
        }
    }

    void changeIntakeLevels()
    {
        if(betterGamepad2.YOnce())
        {
            intakeLevel = IntakeLevel.TOP_54;
        }
        else if(betterGamepad2.BOnce())
        {
            intakeLevel = IntakeLevel.TOP_32;
        }
        else if(betterGamepad2.AOnce())
        {
            intakeLevel = IntakeLevel.INTAKE;
        }
    }

    void intakeStateMachine()
    {
        switch (intakeState) {
            case RETRACT:
                intakeExtension.setTarget(0);

                if(gamepad2.right_trigger != 0 && resetRightTrigger)
                {
                    if(claw.leftClaw == Claw.ClawState.CLOSED && claw.rightClaw == Claw.ClawState.CLOSED)
                    {
                        wasClosed = true;
                    }

                    claw.setBothClaw(Claw.ClawState.OPEN);

                    resetRightTrigger = false;
                    closeClaw = false;
                }

                if(gamepad2.right_trigger == 0 && wasClosed)
                {
                    claw.setBothClaw(Claw.ClawState.CLOSED);
                    wasClosed = false;
                    resetRightTrigger = true;
                    closeClaw = true;
                }
                else if(gamepad2.right_trigger == 0)
                {
                    resetRightTrigger = true;
                    closeClaw = true;
                }

                if(gamepad1.right_trigger == 0)
                {
                    heldExtension = false;
                }

                if (betterGamepad1.rightBumperOnce() && !robot.has2Pixels() && canIntake)
                {
                    intakeState = IntakeState.INTAKE;
                    override = false;
                }
                else if (gamepad1.right_trigger != 0 && !robot.has2Pixels() && canIntake && !heldExtension)
                {
                    moveIntake();
                    claw.setBothClaw(Claw.ClawState.INTAKE);
                    intakeState = IntakeState.INTAKE_EXTEND;
                    override = false;
                }
                else if(liftState == LiftState.RETRACT)
                {
                    drivetrain.fast();
                }

                if(startedDelayTransfer /*&& (intakeExtension.getPos() >= -INTAKE_ZERO_MAX && intakeExtension.getPos() <= INTAKE_ZERO_MAX)*/)
                {
                    intakeMid = false;
                    intake.move(Intake.Angle.OUTTAKE);
                    startedDelayTransfer = false;

                    releaseTimer = getTime();
                }

                if((getTime() - releaseTimer) >= delayRelease/*(intakeExtension.getPos() >= 0 && intakeExtension.getPos() <= INTAKE_ZERO_MAX)*/ && had2Pixels)
                {
                    intake.updateClawState(Intake.ClawState.INDETERMINATE, ClawSide.BOTH);

                    closeTransferTimer = getTime();

                    goToMid = true;

                    had2Pixels = false;
                }


                if(getTime() - closeTransferTimer >= delayCloseTransfer && goToMid && gamepad1.left_trigger == 0 && closeClaw)
                {
                    claw.setBothClaw(Claw.ClawState.CLOSED);
                    overrideIntakeExtension = false;

                    goToMid = false;

                    intakeMid = true;
                }
                else if((liftState == LiftState.EXTRACT || liftState == LiftState.HANG) && intakeMid)
                {
                    intake.move(Intake.Angle.MID);
                    goToTransferTimer = getTime();
                }
                else if(getTime() - goToTransferTimer >= delayGoToTransfer)
                {
                    intake.move(Intake.Angle.OUTTAKE);
                }

                break;
            case INTAKE:
                moveIntake();

                intakeExtension.setTarget(0);

                if (gamepad1.right_trigger != 0)
                {
                    moveIntake();
                    claw.setBothClaw(Claw.ClawState.INTAKE);
                    intakeState = IntakeState.INTAKE_EXTEND;
                }

                claw.setBothClaw(Claw.ClawState.INTAKE);

                if ((robot.has2Pixels() && !startedDelayTransfer) || betterGamepad1.rightBumperOnce() || (intake.closedClaw() && override))
                {
                    had2Pixels = true;
                    transferTimer = getTime();

                    startedDelayTransfer = true;

                    intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
                }
                else if(robot.isCloseLeft() && !robot.has2Pixels() && !override)
                {
                    intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.LEFT);
                }
                else if(robot.isCloseRight() && !robot.has2Pixels() && !override)
                {
                    intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.RIGHT);
                }
                else if(!startedDelayTransfer && !override)
                {
                    intake.updateClawState(Intake.ClawState.OPEN, ClawSide.BOTH);
                }

                if(betterGamepad2.dpadRightOnce() && intake.getClawStateRight() == Intake.ClawState.OPEN)
                {
                    override = true;
                    intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.RIGHT);
                }
                else if(betterGamepad1.dpadRightOnce())
                {
                    override = true;
                    intake.updateClawState(Intake.ClawState.OPEN, ClawSide.RIGHT);
                }
                if(betterGamepad2.dpadLeftOnce() && intake.getClawStateLeft() == Intake.ClawState.OPEN)
                {
                    override = true;
                    intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.LEFT);
                }
                else if(betterGamepad1.dpadLeftOnce())
                {
                    override = true;
                    intake.updateClawState(Intake.ClawState.OPEN, ClawSide.LEFT);
                }

                if((intake.closedClaw() && override) && betterGamepad1.rightBumperOnce())
                {
                    overrideIntakeExtension = true;
                    intakeState = IntakeState.RETRACT;
                }

                if((getTime() - transferTimer) >= delayTransfer && startedDelayTransfer)
                {
                    overrideIntakeExtension = true;
                    intakeState = IntakeState.RETRACT;
                }


                if(betterGamepad2.dpadDownOnce())
                {
                    override = !override;
                }

                break;
            case INTAKE_EXTEND:
                moveIntake();
                heldExtension = true;
                drivetrain.slow();

                if(firstExtend)
                {
                    intakeExtension.setTarget(gamepad1.right_trigger * intakeExtension.MAX_LEVEL * (INTAKE_EXTEND_PRECENTAGE/100));
                }
                else if(gamepad2.left_stick_y != 0 && !overrideIntakeExtension)
                {
                    intakeExtension.setUsePID(false);
                    intakeExtension.setTarget(intakeExtension.getPos());
                }
                


                if ((robot.has2Pixels() && !startedDelayTransfer) || gamepad1.right_trigger == 0 || (intake.closedClaw() && override))
                {
                    overrideIntakeExtension = true;

                    had2Pixels = true;

                    transferTimer = getTime();

                    startedDelayTransfer = true;

                    intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
                }
                else if(robot.isCloseLeft() && !robot.has2Pixels() && !override)
                {
                    intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.LEFT);
                }
                else if(robot.isCloseRight() && !robot.has2Pixels() && !override)
                {
                    intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.RIGHT);
                }
                else if(!startedDelayTransfer && !override)
                {
                    intake.updateClawState(Intake.ClawState.OPEN, ClawSide.BOTH);
                }

                if(betterGamepad2.dpadRightOnce())
                {
                    override = true;
                    intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.RIGHT);
                }
                else if(betterGamepad2.dpadLeftOnce())
                {
                    override = true;
                    intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.LEFT);
                }

                if((getTime() - transferTimer) >= delayTransfer && startedDelayTransfer)
                {
                    overrideIntakeExtension = true;

                    intakeState = IntakeState.INTAKE;
                }

                if (gamepad1.right_trigger == 0) {
                    firstExtend = true;
                    intakeState = IntakeState.RETRACT;
                }

                if(betterGamepad2.dpadDownOnce())
                {
                    override = !override;
                }
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
                    intake.move(Intake.Angle.MID);
                    previousElevator = getTime();
                    claw.setBothClaw(Claw.ClawState.CLOSED);
                    liftState = LiftState.EXTRACT;
                }
                else if(betterGamepad2.dpadUpOnce())
                {
                    intake.move(Intake.Angle.MID);
                    liftState = LiftState.HANG;
                }
                break;
            case EXTRACT:
                canIntake = false;
                intakeState = IntakeState.RETRACT;

                if(retract)
                {
                    elevator.setTarget(elevatorTargetRight + (openedXTimes * (Elevator.ELEVATOR_INCREMENT * 3.5)), elevatorTargetLeft + (openedXTimes * (Elevator.ELEVATOR_INCREMENT * 3.5)));
                }
                else
                {
                    elevator.setTarget(elevatorTargetRight + (openedXTimes * (Elevator.ELEVATOR_INCREMENT)), elevatorTargetLeft + (openedXTimes * (Elevator.ELEVATOR_INCREMENT)));
                }

                if ((getTime() - previousElevator) >= WAIT_DELAY_TILL_OUTTAKE) {
                    outtake.setAngle(Outtake.Angle.OUTTAKE);
                }

                drivetrain.slow();

                if(betterGamepad1.dpadRightOnce())
                {
                    claw.setRightClaw(Claw.ClawState.OPEN);
                }
                else if(betterGamepad1.dpadLeftOnce())
                {
                    claw.setLeftClaw(Claw.ClawState.OPEN);
                }
                else if(betterGamepad1.dpadDownOnce())
                {
                    claw.setBothClaw(Claw.ClawState.INTERMEDIATE);
                }

                if(betterGamepad2.rightBumperOnce())
                {
                    elevatorTargetRight += 85;
                    elevatorTargetLeft += 85;
                }
                else if(betterGamepad2.leftBumperOnce())
                {
                    elevatorTargetRight -= 85;
                    elevatorTargetLeft -= 85;
                }

                if (betterGamepad1.AOnce())  {
                    claw.setBothClaw(Claw.ClawState.OPEN);

                    elevatorTargetRight = elevator.getTargetRight() - (openedXTimes * Elevator.ELEVATOR_INCREMENT);
                    elevatorTargetLeft = elevator.getTargetLeft() - (openedXTimes * Elevator.ELEVATOR_INCREMENT);
                    openedXTimes++;

                    elevatorReset = getTime();
                    retract = true;
                } else if ((getTime() - elevatorReset) >= WAIT_DELAY_TILL_CLOSE && retract)
                {
                    retract = false;
                    liftState = LiftState.RETRACT;
                }
                break;
            case HANG:

                if(betterGamepad2.dpadUpOnce())
                {
                    hang = !hang;
                }

                if(!hang)
                {
                    elevator.setTarget(Elevator.HANG_OPEN);
                }
                else
                {
                    elevator.setTarget(Elevator.HANG);
                }


                if (betterGamepad1.AOnce() || betterGamepad1.leftBumperOnce())
                {
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
        return codeTime.nanoseconds() / 1000000;
    }

    void moveIntake()
    {
        switch(intakeLevel)
        {
            case TOP_54:
                intake.move(Intake.Angle.TOP_54);
                intake.setSeeFarFrom(Intake.minSeeFarFrom);
                break;
            case TOP_32:
                intake.move(Intake.Angle.TOP_32);
                intake.setSeeFarFrom(Intake.minSeeFarFrom);

                break;
            case INTAKE:
                intake.move(Intake.Angle.INTAKE);
                intake.setSeeFarFrom(Intake.maxSeeFarFrom);
                break;
        }
    }


}
