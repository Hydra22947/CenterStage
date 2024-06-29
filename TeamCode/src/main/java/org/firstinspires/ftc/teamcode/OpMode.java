package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Plane;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.ClawSide;

@Config
@TeleOp(name = "OpMode Teleop")
public class OpMode extends LinearOpMode {

    // robot
    private final RobotHardware robot = RobotHardware.getInstance();

    // subsystems
    Drivetrain drivetrain;
    Elevator elevator;
    Intake intake;
    Outtake outtake;
    Claw claw;
    IntakeExtension intakeExtension;
    Plane plane;
    ElapsedTime codeTime;
    Gamepad.RumbleEffect customRumbleEffect;    // Use to build a custom rumble sequence.


    // gamepads
    GamepadEx gamepadEx, gamepadEx2;
    BetterGamepad betterGamepad1, betterGamepad2;

    // delays
    public static double delayTransfer = 300, delayRelease = 1100, delayCloseTransfer = 350, XDelay = 500, goToIntakeDelay = 50, goToAlmostIntakeDelay = 250;
    public static double WAIT_DELAY_TILL_OUTTAKE = 150, WAIT_DELAY_TILL_CLOSE = 250, ELEVATOR_ZERO = 10, COOL_DOWN = 350;
    public static double DEFAULT_INTAKE_EXTEND_PRECENTAGE = 42.5, SHORT_INTAKE_EXTEND_PRECENTAGE = 25, delayReleaseFromIntake = 200;
    // variables
    double elevatorReset = 0, previousElevator = 0, transferTimer = 0, releaseTimer = 0, closeTransferTimer = 0, goToTransferTimer = 0, goToIntakeTimer = 0, goToAlmostIntakeTimer = 0;
    double elevatorTargetRight = 800, intakePrecentage = DEFAULT_INTAKE_EXTEND_PRECENTAGE, releaseFromIntake = 0, startXDelay = 0, cooldown = 0;
    double elevatorTargetLeft = 800;
    int openedXTimes = 0, ACount = 0;
    boolean firstOuttakeAngle = true, retract = false,  goToMid = false, intakeMid = true, canIntake = true, startedDelayTransfer = false, heldExtension = false, firstReleaseThreeTimer = true;
    boolean override = false, had2Pixels = false, hang = false, resetRightTrigger = true, closeClaw = false, wasClosed = false, firstExtend = true, XPressed = false;
    boolean overrideIntakeExtension = false, movedStack = false, outtakeToOuttake = true, firstReleaseThree = true, firstOuttake = true, goToIntake = false, goToAlmostIntake = false;
    boolean rightClaw = true, leftClaw = true;
    public enum IntakeState {
        RETRACT,
        RELEASE,
        INTAKE,
        INTAKE_EXTEND
    }

    public enum OuttakeState {
        OUTTAKE,
        OUTTAKE_LONG
    }

    public enum LiftState {
        RETRACT,
        EXTRACT,
        STUCK_3,
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
    OuttakeState outtakeState = OuttakeState.OUTTAKE;

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        betterGamepad1 = new BetterGamepad(gamepad1);
        betterGamepad2 = new BetterGamepad(gamepad2);

        robot.init(hardwareMap, telemetry);

        drivetrain = new Drivetrain(gamepad1, true, false);
        elevator = new Elevator(gamepad2, false, false);
        outtake = new Outtake();
        claw = new Claw();
        intake = new Intake();
        plane = new Plane();
        intakeExtension = new IntakeExtension(gamepad2, false);
        codeTime = new ElapsedTime();
        intake.setAngle(Intake.Angle.TELEOP_MID);
        intake.updateClawState(Intake.ClawState.INDETERMINATE, ClawSide.BOTH);
        claw.setBothClaw(Claw.ClawState.INTAKE);
        outtake.setAngle(Outtake.Angle.INTAKE);
        elevator.setAuto(false);
        intakeExtension.setAuto(false);

        intake.update();
        claw.update();
        outtake.update();
        intakeExtension.update();

        customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
                .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 0.0, 250)  //  Pause for 250 mSec
                .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
                .build();


        while (opModeInInit() && !isStopRequested())
        {
            telemetry.addLine("Initialized");
            telemetry.update();
            intake.update();
            plane.update();
            claw.update();
            outtake.update();
            intakeExtension.update();
        }

        waitForStart();

        codeTime.reset();


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
            plane.update();

            if(betterGamepad1.dpadUpOnce())
            {
                plane.toggle();
            }

            if(gamepad2.left_trigger != 0 || gamepad1.left_trigger != 0)
            {
                drivetrain.superSlow();
            }


            if (liftState != LiftState.EXTRACT && gamepad2.left_stick_y != 0 && !overrideIntakeExtension) {
                intakeExtension.setUsePID(false);
            }
            else if (gamepad2.left_stick_y != 0 && !overrideIntakeExtension) {
                intakeExtension.setUsePID(false);
            } else {
                intakeExtension.setUsePID(true);
            }

            if (gamepad2.right_stick_y != 0) {
                elevator.setUsePID(false);
            } else {
                elevator.setUsePID(true);
            }

            if(betterGamepad2.dpadDownOnce())
            {
                if(movedStack)
                {
                    intake.returnStack();
                    movedStack = false;
                }
                else
                {
                    intake.moveStack();
                    movedStack = true;
                }
            }

            if(betterGamepad1.shareOnce())
            {
                drivetrain.maxPower = 0.9;
            }

            changeIntakeLevels();
            intakeStateMachine();
            elevatorStateMachine();


            telemetry.addData("left" , leftClaw);
            telemetry.addData("right" , rightClaw);
            telemetry.update();
        }
    }

    void changeIntakeLevels()
    {
        if(betterGamepad2.YOnce() && liftState != LiftState.EXTRACT)
        {
            intakePrecentage = SHORT_INTAKE_EXTEND_PRECENTAGE;
            intakeLevel = IntakeLevel.TOP_54;
        }
        else if(betterGamepad2.BOnce() && liftState != LiftState.EXTRACT)
        {
            intakePrecentage = SHORT_INTAKE_EXTEND_PRECENTAGE;
            intakeLevel = IntakeLevel.TOP_32;
        }
        else if(betterGamepad2.AOnce() && liftState != LiftState.EXTRACT)
        {
            intakePrecentage = DEFAULT_INTAKE_EXTEND_PRECENTAGE;
            intakeLevel = IntakeLevel.INTAKE;
        }
    }

    void intakeStateMachine()
    {
        switch (intakeState) {
            case RETRACT:
                intakeExtension.setTarget(-35);

                if(gamepad2.right_trigger != 0 && resetRightTrigger)
                {
                    if(claw.leftClaw == Claw.ClawState.CLOSED && claw.rightClaw == Claw.ClawState.CLOSED)
                    {
                        wasClosed = true;
                    }

                    openCloseClaws();

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

                if (betterGamepad1.rightBumperOnce() && (!robot.has2Pixels() && canIntake))
                {
                    firstReleaseThree = true;
                    intakeState = IntakeState.INTAKE;
                    override = false;
                }
                else if (gamepad1.right_trigger != 0 && (!robot.has2Pixels() && canIntake && !heldExtension))
                {
                    firstReleaseThree = true;
                    moveIntake();

                    openCloseClaws();

                    intakeState = IntakeState.INTAKE_EXTEND;
                    override = false;
                }
                else if(liftState == LiftState.RETRACT && (gamepad2.left_trigger == 0 && gamepad1.left_trigger == 0))
                {
                    drivetrain.fast();
                }

                if(startedDelayTransfer)
                {
                    outtake.setAngle(Outtake.Angle.ALMOST_INTAKE);
                    intakeMid = false;
                    intake.move(Intake.Angle.OUTTAKE);
                    startedDelayTransfer = false;

                    releaseTimer = getTime();
                }

                if((getTime() - releaseTimer) >= delayRelease && had2Pixels)
                {

                    outtake.setAngle(Outtake.Angle.INTAKE);

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

                    goToIntakeTimer = getTime();
                    goToIntake = true;
                }
                else if((liftState == LiftState.EXTRACT || liftState == LiftState.HANG || liftState == LiftState.STUCK_3) && intakeMid)
                {
                    intake.move(Intake.Angle.TELEOP_MID);
                    goToTransferTimer = getTime();
                }
                else if((elevator.getPos() <= ELEVATOR_ZERO) && !XPressed)
                {
                    //intake.move(Intake.Angle.TELEOP_MID);
                }
                else if(XPressed && (getTime() - startXDelay) >= XDelay)
                {
                    XPressed = false;
                    intake.move(Intake.Angle.OUTTAKE);
                }

                if(goToIntake && (getTime() - goToIntakeTimer) >= goToIntakeDelay)
                {
                    intake.updateClawState(Intake.ClawState.INDETERMINATE, ClawSide.BOTH);

                    goToAlmostIntakeTimer = getTime();
                    goToAlmostIntake = true;
                    goToIntake = false;
                }
                if(goToAlmostIntake && (getTime() - goToAlmostIntakeTimer) >= goToAlmostIntakeDelay)
                {

                    outtake.setAngle(Outtake.Angle.INTAKE);
                    intake.move(Intake.Angle.TELEOP_MID);
                    goToAlmostIntake = false;
                }

                break;
            case INTAKE:
                claw.setBothClaw(Claw.ClawState.INTAKE);
                leftClaw = true;
                rightClaw = true;
                moveIntake();

                if(firstReleaseThreeTimer)
                {
                    releaseFromIntake = getTime();
                    firstReleaseThreeTimer = false;
                }

                if(firstReleaseThree && ((getTime() - releaseFromIntake) >= delayReleaseFromIntake))
                {
                    intake.updateClawState(Intake.ClawState.OPEN, ClawSide.BOTH);
                    firstReleaseThree = false;
                }

                if(gamepad2.left_stick_y != 0 && !overrideIntakeExtension)
                {
                    intakeExtension.setUsePID(false);
                    intakeExtension.setTarget(intakeExtension.getPos());
                }
                else
                {
                    intakeExtension.setTarget(-35);
                }

                if (gamepad1.right_trigger != 0)
                {
                    moveIntake();

                    openCloseClaws();

                    intakeState = IntakeState.INTAKE_EXTEND;
                }

                //openCloseClaws();

                if ((((robot.has2Pixels() && !startedDelayTransfer) || betterGamepad1.rightBumperOnce() || (intake.closedClaw() && override))))
                {
                    had2Pixels = true;
                    transferTimer = getTime();

                    startedDelayTransfer = true;

                    intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
                }
                else if(robot.isCloseLeft() && !robot.has2Pixels() && !override)
                {
                    leftClaw = false;
                    intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.LEFT);
                }
                else if(robot.isCloseRight() && !robot.has2Pixels() && !override)
                {
                    rightClaw = false;
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
                else if(betterGamepad2.dpadRightOnce())
                {
                    override = true;
                    intake.updateClawState(Intake.ClawState.OPEN, ClawSide.RIGHT);
                }
                if(betterGamepad2.dpadLeftOnce() && intake.getClawStateLeft() == Intake.ClawState.OPEN)
                {
                    override = true;
                    intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.LEFT);
                }
                else if(betterGamepad2.dpadLeftOnce())
                {
                    override = true;
                    intake.updateClawState(Intake.ClawState.OPEN, ClawSide.LEFT);
                }

                if((intake.closedClaw() && override) && betterGamepad1.rightBumperOnce())
                {
                    overrideIntakeExtension = true;
                    if(outtakeToOuttake)
                    {
                        intakeState = IntakeState.RETRACT;
                    }
                }

                if((getTime() - transferTimer) >= delayTransfer && startedDelayTransfer)
                {
                    overrideIntakeExtension = true;
                    if(outtakeToOuttake)
                    {
                        intakeState = IntakeState.RETRACT;
                    }
                }


                if(betterGamepad2.dpadDownOnce())
                {
                    override = !override;
                }

                break;
            case INTAKE_EXTEND:
                claw.setBothClaw(Claw.ClawState.INTAKE);
                leftClaw = true;
                rightClaw = true;
                moveIntake();
                heldExtension = true;

                if(firstReleaseThreeTimer)
                {
                    releaseFromIntake = getTime();
                    firstReleaseThreeTimer = false;
                }

                if(firstReleaseThree && ((getTime() - releaseFromIntake) >= delayReleaseFromIntake))
                {
                    intake.updateClawState(Intake.ClawState.OPEN, ClawSide.BOTH);
                    firstReleaseThree = false;
                }

                if(firstExtend && !startedDelayTransfer)
                {
                    intakeExtension.setTarget(gamepad1.right_trigger * intakeExtension.MAX_LEVEL * (intakePrecentage/100));
                }
                else if(gamepad2.left_stick_y != 0 && !overrideIntakeExtension && !startedDelayTransfer)
                {
                    intakeExtension.setUsePID(false);
                    intakeExtension.setTarget(intakeExtension.getPos());
                }

                if ((((robot.has2Pixels() && !startedDelayTransfer) || gamepad1.right_trigger == 0 || (intake.closedClaw() && override))))
                {
                    overrideIntakeExtension = true;

                    had2Pixels = true;

                    transferTimer = getTime();

                    startedDelayTransfer = true;

                    intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
                }
                else if(robot.isCloseLeft() && !robot.has2Pixels() && !override)
                {
                    leftClaw = false;
                    intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.LEFT);
                }
                else if(robot.isCloseRight() && !robot.has2Pixels() && !override)
                {
                    rightClaw = false;
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
                else if(betterGamepad2.dpadRightOnce())
                {
                    override = true;
                    intake.updateClawState(Intake.ClawState.OPEN, ClawSide.RIGHT);
                }
                if(betterGamepad2.dpadLeftOnce() && intake.getClawStateLeft() == Intake.ClawState.OPEN)
                {
                    override = true;
                    intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.LEFT);
                }
                else if(betterGamepad2.dpadLeftOnce())
                {
                    override = true;
                    intake.updateClawState(Intake.ClawState.OPEN, ClawSide.LEFT);
                }

                if((getTime() - transferTimer) >= delayTransfer && startedDelayTransfer)
                {
                    intakeExtension.setTarget(0);

                    overrideIntakeExtension = true;

                    intakeState = IntakeState.INTAKE;
                }

                if (gamepad1.right_trigger == 0) {
                    firstExtend = true;
                    if(outtakeToOuttake)
                    {
                        intakeState = IntakeState.RETRACT;
                    }
                }

                if(betterGamepad2.dpadDownOnce())
                {
                    override = !override;
                }
                break;
            case RELEASE:

                break;
            default:
                if(outtakeToOuttake)
                {
                    intakeState = IntakeState.RETRACT;
                }
                break;
        }
    }

    void elevatorStateMachine()
    {
        switch (liftState) {
            case RETRACT:
                firstOuttake = true;
                elevator.setTarget(0);

                canIntake = true;

                if (betterGamepad1.YOnce())
                {
                    intake.move(Intake.Angle.TELEOP_MID);
                    previousElevator = getTime();
                    claw.setBothClaw(Claw.ClawState.CLOSED);
                    liftState = LiftState.EXTRACT;
                }
                else if(betterGamepad2.dpadUpOnce())
                {
                    intake.move(Intake.Angle.TELEOP_MID);
                    liftState = LiftState.HANG;
                }
                else if(betterGamepad2.XOnce())
                {
                    intake.move(Intake.Angle.TELEOP_MID);
                    liftState = LiftState.STUCK_3;
                }
                break;
            case EXTRACT:

                leftClaw = true;
                rightClaw = true;

                if(firstOuttake)
                {
                    claw.setBothClaw(Claw.ClawState.CLOSED);
                    firstOuttake = false;
                }
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

                if(gamepad2.right_stick_y != 0)
                {
                    elevatorTargetRight = elevator.getPosRight() - (openedXTimes * (Elevator.ELEVATOR_INCREMENT));
                    elevatorTargetLeft = elevator.getPosLeft() - (openedXTimes * (Elevator.ELEVATOR_INCREMENT));
                }

                if (firstOuttakeAngle && (getTime() - previousElevator) >= WAIT_DELAY_TILL_OUTTAKE) {
                    outtake.setAngle(Outtake.Angle.OUTTAKE);
                    firstOuttakeAngle = false;
                }

                if(betterGamepad1.dpadRightOnce() && cooldowned())
                {
                    claw.setRightClaw(Claw.ClawState.OPEN);
                }
                else if(betterGamepad1.dpadLeftOnce() && cooldowned())
                {
                    claw.setLeftClaw(Claw.ClawState.OPEN);
                }

                if(betterGamepad2.BOnce())
                {
                    outtake.spinOuttake(1);
                }
                else if(betterGamepad2.XOnce())
                {
                    outtake.spinOuttake(-1);
                }

                if(betterGamepad2.rightBumperOnce())
                {
                    elevatorTargetRight += 150;
                    elevatorTargetLeft += 150;
                    coolDownReset();
                }
                else if(betterGamepad2.leftBumperOnce())
                {
                    elevatorTargetRight -= 150;
                    elevatorTargetLeft -= 150;
                    coolDownReset();
                }
                else if(betterGamepad2.dpadLeftOnce())
                {
                    elevatorTargetRight -= 415;
                    elevatorTargetLeft -= 415;
                    coolDownReset();
                }
                else if(betterGamepad2.dpadRightOnce())
                {
                    elevatorTargetRight += 415;
                    elevatorTargetLeft += 415;
                    coolDownReset();
                }

                if(betterGamepad2.touchpadOnce())
                {
                    switchOuttake();
                }

                if(betterGamepad1.YOnce() || betterGamepad1.AOnce())
                {
                    claw.setBothClaw(Claw.ClawState.OPEN);

                    ACount++;
                    //outtake.releasePixel();
                }

                if ((ACount > 1 && cooldowned()) || (betterGamepad2.shareOnce() && cooldowned()))  {
                    claw.setBothClaw(Claw.ClawState.OPEN);

//                    elevatorTargetRight = elevator.getTargetRight() - (openedXTimes * Elevator.ELEVATOR_INCREMENT);
//                    elevatorTargetLeft = elevator.getTargetLeft() - (openedXTimes * Elevator.ELEVATOR_INCREMENT);
                    openedXTimes++;


                    elevatorReset = getTime();
                    retract = true;
                    ACount = 0;
                } else if ((getTime() - elevatorReset) >= WAIT_DELAY_TILL_CLOSE && retract)
                {
                    retract = false;
                    firstOuttakeAngle = true;
                    outtake.resetOuttake();
                    outtake.setAngle(Outtake.Angle.INTAKE);
                    liftState = LiftState.RETRACT;
                }
                break;
            case HANG:

                outtake.setAngle(Outtake.Angle.HANG);

                if(betterGamepad2.rightBumperOnce())
                {
                    elevatorTargetRight += 100;
                    elevatorTargetLeft += 100;
                }
                else if(betterGamepad2.leftBumperOnce())
                {
                    elevatorTargetRight -= 100;
                    elevatorTargetLeft -= 100;
                }

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


                if (betterGamepad1.AOnce() || betterGamepad1.leftBumperOnce() || betterGamepad2.shareOnce())
                {
                    liftState = LiftState.RETRACT;
                    outtake.setAngle(Outtake.Angle.INTAKE);
                }
                break;
            case STUCK_3:
                outtake.setAngle(Outtake.Angle.OUTTAKE);


                if(betterGamepad2.XOnce())
                {
                    claw.setBothClaw(Claw.ClawState.CLOSED);
                    startXDelay = getTime();
                    XPressed = true;
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
    void switchOuttake()
    {
        switch (outtakeState)
        {
            case OUTTAKE:
                outtakeState = OuttakeState.OUTTAKE_LONG;
                break;
            case OUTTAKE_LONG:
                outtakeState = OuttakeState.OUTTAKE;
                break;
        }
    }

    boolean cooldowned()
    {
        return (getTime() - cooldown) >= COOL_DOWN;
    }

    void coolDownReset()
    {
        cooldown = getTime();
    }

    void openCloseClaws()
    {
        if(leftClaw)
        {
            claw.setRightClaw(Claw.ClawState.INTAKE);
        }
        else
        {
            claw.setRightClaw(Claw.ClawState.CLOSED);
        }
        if(rightClaw)
        {
            claw.setLeftClaw(Claw.ClawState.INTAKE);
        }
        else
        {
            claw.setLeftClaw(Claw.ClawState.CLOSED);
        }
    }


}
