package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.ClawSide;

@Config
@TeleOp(name = "OpMode Blue")
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

    // delays
    public static double delayTransfer = 300, delayRelease = 1200, delayCloseTransfer = 350, delayGoToTransfer = 800;
    public static double WAIT_DELAY_TILL_OUTTAKE = 200, WAIT_DELAY_TILL_CLOSE = 200;

    // variables
    double elevatorReset = 0, previousElevator = 0, transferTimer = 0, releaseTimer = 0, closeTransferTimer = 0, goToTransferTimer = 0;
    double elevatorTarget = Elevator.BASE_LEVEL;
    int openedXTimes = 0;
    boolean retract = false,  goToMid = false, intakeMid = true, canIntake = true, startedDelayTransfer = false, heldExtension = false;
    boolean override = false, had2Pixels = false, hang = false;
    double loopTime = 0;

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
    public void initialize() {
        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

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
        intakeExtension = new IntakeExtension(gamepad1);

        intake.setAngle(Intake.Angle.OUTTAKE);
        intakeExtension.setCurrent(IntakeExtension.ExtensionState.CLOSE);
        intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
        claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);
        outtake.setAngle(Outtake.Angle.INTAKE);
        elevator.setAuto(false);

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
    }

    @Override
    public void run() {
        intakeExtension.update();
        betterGamepad1.update();
        betterGamepad2.update();
        drivetrain.update();
        intake.update();
        outtake.update();
        elevator.update();

        if (gamepad2.left_stick_y != 0) {
            elevator.setUsePID(false);
        } else {
            elevator.setUsePID(true);
        }



        changeIntakeLevels();
        intakeStateMachine();
        elevatorStateMachine();

        telemetry.addData("hz ", 1000000000 / (System.nanoTime() - loopTime));
        telemetry.addData("closed claw", intake.closedClaw());
        telemetry.addData("manual transfer", (intake.closedClaw() && override));
        telemetry.update();
        CommandScheduler.getInstance().run();

        loopTime = System.nanoTime();
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
                intakeExtension.setCurrent(IntakeExtension.ExtensionState.CLOSE);

                if(gamepad1.right_trigger == 0)
                {
                    heldExtension = false;
                }

                if (betterGamepad1.rightBumperOnce() && !robot.has2Pixels() && canIntake) {
                    intakeState = IntakeState.INTAKE;
                    override = false;
                } else if (gamepad1.right_trigger != 0 && !robot.has2Pixels() && canIntake && !heldExtension) {
                    intakeState = IntakeState.INTAKE_EXTEND;
                    override = false;
                }
                else if(liftState == LiftState.RETRACT)
                {
                    drivetrain.fast();
                }

                if(startedDelayTransfer)
                {
                    intakeMid = false;
                    intake.move(Intake.Angle.OUTTAKE);
                    startedDelayTransfer = false;

                    releaseTimer = getTime();
                }

                if((getTime() - releaseTimer) >= delayRelease && had2Pixels)
                {
                    intake.updateClawState(Intake.ClawState.INDETERMINATE, ClawSide.BOTH);

                    closeTransferTimer = getTime();

                    goToMid = true;

                    had2Pixels = false;
                }


                if(getTime() - closeTransferTimer >= delayCloseTransfer && goToMid)
                {
                    claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH);

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

                intakeExtension.setCurrent(IntakeExtension.ExtensionState.CLOSE);

                if (gamepad1.right_trigger != 0) {
                    intakeState = IntakeState.INTAKE_EXTEND;
                }

                claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);

                if ((robot.has2Pixels() && !startedDelayTransfer) || betterGamepad1.rightBumperOnce() || (intake.closedClaw() && override)) {
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
                else if(betterGamepad2.dpadRightOnce())
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
                    intakeState = IntakeState.RETRACT;
                }


                if(betterGamepad2.dpadDownOnce())
                {
                    override = !override;
                }

                break;
            case INTAKE_EXTEND:
                heldExtension = true;
                drivetrain.slow();

                intakeExtension.setCurrent(IntakeExtension.ExtensionState.MANUAL);
                moveIntake();
                claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);

                if ((robot.has2Pixels() && !startedDelayTransfer) || gamepad1.right_trigger == 0 || (intake.closedClaw() && override))
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
                else if(betterGamepad2.dpadRightOnce())
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
                    intakeState = IntakeState.INTAKE;
                }

                if (gamepad1.right_trigger == 0) {
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
                    claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH);
                    liftState = LiftState.EXTRACT;
                }
                else if(betterGamepad1.dpadUpOnce())
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
                    elevator.setTarget(elevatorTarget + (openedXTimes * (Elevator.ELEVATOR_INCREMENT * 3.5)));
                }
                else
                {
                    elevator.setTarget(elevatorTarget + (openedXTimes * Elevator.ELEVATOR_INCREMENT));
                }

                if(gamepad2.left_stick_y != 0)
                {
                    elevatorTarget += elevator.getPos() - (elevatorTarget + (openedXTimes * Elevator.ELEVATOR_INCREMENT));
                }

                if ((getTime() - previousElevator) >= WAIT_DELAY_TILL_OUTTAKE) {
                    outtake.setAngle(Outtake.Angle.OUTTAKE);
                }

                drivetrain.slow();

                if(betterGamepad1.dpadRightOnce())
                {
                    claw.updateState(Claw.ClawState.OPEN, ClawSide.RIGHT);
                }
                else if(betterGamepad1.dpadLeftOnce())
                {
                    claw.updateState(Claw.ClawState.OPEN, ClawSide.LEFT);
                }
                else if(betterGamepad1.dpadDownOnce())
                {
                    claw.updateState(Claw.ClawState.INTERMEDIATE, ClawSide.BOTH);
                }

                if(betterGamepad2.rightBumperOnce())
                {
                    outtake.setOuttakeHandPivot(outtake.outtakeHandPivot += 0.015);
                }
                else if(betterGamepad2.leftBumperOnce())
                {
                    outtake.setOuttakeHandPivot(outtake.outtakeHandPivot -= 0.015);
                }

                if (betterGamepad1.AOnce() || betterGamepad1.leftBumperOnce())  {
                    elevatorTarget = elevator.getTarget() - (openedXTimes * Elevator.ELEVATOR_INCREMENT);
                    openedXTimes++;
                    claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);

                    elevatorReset = getTime();
                    retract = true;
                } else if ((getTime() - elevatorReset) >= WAIT_DELAY_TILL_CLOSE && retract) {
                    retract = false;
                    liftState = LiftState.RETRACT;
                }
                break;
            case HANG:

                if(betterGamepad1.dpadUpOnce()) {
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
        return System.nanoTime() / 1000000;
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
