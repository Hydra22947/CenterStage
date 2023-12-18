package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.testing.harman.PoseStorage;
import org.firstinspires.ftc.teamcode.util.ClawSide;

@Config
@Autonomous(name = "Auto Red Left")
public class AutoRedLeft extends CommandOpMode
{
    AutoConstants autoConstants;
    // robot
    private final RobotHardware robot = RobotHardware.getInstance();

    // subsystems
    Elevator elevator;
    Intake intake;
    Outtake outtake;
    Claw claw;
    IntakeExtension intakeExtension;
    SampleMecanumDrive drivetrain;

    // gamepads
    public static double delayTransfer = 300;
    public static double delayRelease = 1200;
    public static double delayGoToMid = 500;
    public static double delayGoToTransfer = 500;
    public static double WAIT_DELAY_TILL_OUTTAKE = 150;
    public static double WAIT_DELAY_TILL_CLOSE = 250;

    // variables
    double elevatorReset = 0, previousElevator = 0, transferTimer = 0, releaseTimer = 0, goToMidTimer = 0, goToTransferTimer = 0;
    int openedXTimes = 0;
    boolean retract = false, goToMid = false, intakeMid = true, canIntake = true, startedDelayTransfer = false, heldExtension = false, had2Pixels = false;
    boolean resetElevator = false, openElevator = false, releaseIntake = false, extendIntake = false;
    double waitTillPlacePurple = 1, waitTillIntakePixel = 2, waitTillOuttakePixel = 2;
    ElapsedTime waitTimer1 = new ElapsedTime(), waitTimer2 = new ElapsedTime();
    ElapsedTime waitTimer3 = new ElapsedTime(), waitTimer4 = new ElapsedTime(), waitTimer5 = new ElapsedTime();
    Pose2d poseEstimate;
    public enum IntakeState {
        RETRACT,
        INTAKE,
        INTAKE_EXTEND
    }

    public enum LiftState {
        RETRACT,
        EXTRACT
    }

    enum robotMovingState {
        PLACE_PURPLE_PIXEL,
        WAIT_PLACE_PURPLE_PIXEL,
        INTAKE_1_PIXEL,
        WAIT_INTAKE_1_PIXEL,
        WAIT_INTAKE_CYCLE_1,
        PLACE_PRELOAD,
        WAIT_INTAKE,
        WAIT_PLACE_PRELOAD_CYCLE,
        CYCLE1,
        PARK,
        IDLE
    }

    IntakeState intakeState = IntakeState.RETRACT;
    LiftState liftState = LiftState.RETRACT;
    double loopTime = 0;
    robotMovingState robotState = robotMovingState.IDLE;

    TrajectorySequence placePurplePixel, placePreload, goIntake, park;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, telemetry);

        elevator = new Elevator();
        outtake = new Outtake();
        claw = new Claw();
        intake = new Intake();
        intakeExtension = new IntakeExtension();

        intake.setAngle(Intake.Angle.TRANSFER);
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
            intakeExtension.update();
            outtake.update();
        }
    }

    @Override
    public void run() {
        autoConstants = new AutoConstants();
        drivetrain = new SampleMecanumDrive(hardwareMap);
        drivetrain.setPoseEstimate(autoConstants.startPose);

        placePurplePixel = drivetrain.trajectorySequenceBuilder(autoConstants.startPose)
                .forward(AutoConstants.strafeForPurplePixel)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> releaseIntake = true)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> releaseIntake = true)
                //.waitSeconds(AutoConstants.WAIT_EXTENSION)
                .build();

        // TODO: add intake 1 pixel

        placePreload = drivetrain.trajectorySequenceBuilder(/** CHANGE THE END*/placePurplePixel.end())
                //Going for backdrop
                .lineToLinearHeading(autoConstants.stageDoorMidPose)
                .splineToSplineHeading(autoConstants.stageDoorEndPose, Math.toRadians(0))
                .splineToLinearHeading(autoConstants.placePixelPose, Math.toRadians(0))
                .waitSeconds(1)
                .build();

        goIntake = drivetrain.trajectorySequenceBuilder(placePreload.end())
                .waitSeconds(AutoConstants.WAIT_EXTENSION)
                .lineToSplineHeading(autoConstants.stageDoorStartPose)
                .splineToConstantHeading(autoConstants.intakePixelVector, Math.toRadians(180))
                .build();

        // TODO: add cycles

        park = drivetrain.trajectorySequenceBuilder(placePreload.end())
                //Going for backdrop
                .lineToLinearHeading(autoConstants.park)
                .build();


        waitForStart();
        if (isStopRequested()) return;

        robotState = robotMovingState.PLACE_PURPLE_PIXEL;
        drivetrain.followTrajectorySequenceAsync(placePurplePixel);

        while(opModeIsActive())
        {
            intakeExtension.update();
            drivetrain.update();
            intake.update();
            outtake.update();

            autoFSM();
            intakeStateMachine();
            elevatorStateMachine();

            // Read pose
            poseEstimate = drivetrain.getPoseEstimate();

            // Continually write pose to `PoseStorage`
            PoseStorage.currentPose = poseEstimate;

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }

    void autoFSM()
    {
        switch (robotState)
        {
            case PLACE_PURPLE_PIXEL:
                if (!drivetrain.isBusy()) {
                    robotState = robotMovingState.WAIT_PLACE_PURPLE_PIXEL;
                    waitTimer1.reset();
                }
                break;
            case WAIT_PLACE_PURPLE_PIXEL:
                if (waitTimer1.seconds() >= waitTillPlacePurple) {
                    robotState = robotMovingState.INTAKE_1_PIXEL;
                    //TODO: drivetrain.followTrajectorySequenceAsync(goToIntake1Pixel);
                }
                break;
            case INTAKE_1_PIXEL:
                if (!drivetrain.isBusy()) {
                    robotState = robotMovingState.WAIT_INTAKE_1_PIXEL;
                    waitTimer2.reset();
                }
                break;
            case WAIT_INTAKE_1_PIXEL:
                if (waitTimer1.seconds() >= waitTillIntakePixel) {
                    robotState = robotMovingState.PLACE_PRELOAD;
                    drivetrain.followTrajectorySequenceAsync(placePreload);
                }
                break;
            case PLACE_PRELOAD:
                if (!drivetrain.isBusy()) {
                    robotState = robotMovingState.WAIT_PLACE_PRELOAD_CYCLE;
                    waitTimer3.reset();
                }
                break;
            case WAIT_PLACE_PRELOAD_CYCLE:
                if (waitTimer3.seconds() >= waitTillOuttakePixel) {
                    robotState = robotMovingState.WAIT_INTAKE;
                    drivetrain.followTrajectorySequenceAsync(goIntake);
                }
                break;
            case CYCLE1:
                if (!drivetrain.isBusy()) {
                    robotState = robotMovingState.WAIT_INTAKE_CYCLE_1;
                    waitTimer4.reset();
                }
                break;
            case WAIT_INTAKE_CYCLE_1:
                if (waitTimer4.seconds() >= waitTillOuttakePixel) {
                    robotState = robotMovingState.PARK;//CYCLE2;
                    //drivetrain.followTrajectorySequenceAsync(goToPlace);
                }
                break;
//            case CYCLE2: //continue from here
//                if (waitTimer4.seconds() >= waitTillIntakePixel) {
//                    robotState = robotMovingState.WAIT_PLACE_CYCLE_2;
//                    drivetrain.followTrajectorySequenceAsync(goIntake);
//                }
//                break;
            case PARK:
                if (!drivetrain.isBusy()) {
                    robotState = robotMovingState.IDLE;
                }
                break;
            case IDLE:
                break;
        }

        telemetry.addData("hz ", 1000000000 / (System.nanoTime() - loopTime));
        telemetry.update();
        CommandScheduler.getInstance().run();

        loopTime = System.nanoTime();
    }

    void intakeStateMachine()
    {
        switch (intakeState) {
            case RETRACT:
                intakeExtension.setCurrent(IntakeExtension.ExtensionState.CLOSE);

                if (releaseIntake) {
                    intakeState = IntakeState.INTAKE;
                } else if (extendIntake) {
                    intakeState = IntakeState.INTAKE_EXTEND;
                }


                if(startedDelayTransfer)
                {
                    intakeMid = false;
                    intake.move(Intake.Angle.TRANSFER);
                    startedDelayTransfer = false;

                    releaseTimer = getTime();
                }

                if((getTime() - releaseTimer) >= delayRelease && had2Pixels)
                {
                    intake.updateClawState(Intake.ClawState.OPEN, ClawSide.BOTH);

                    goToMidTimer = getTime();

                    goToMid = true;

                    had2Pixels = false;
                }


                if(getTime() - goToMidTimer >= delayGoToMid && goToMid)
                {
                    intake.move(Intake.Angle.MID);
                    claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH);

                    goToMid = false;

                    intakeMid = true;
                }
                else if(liftState == LiftState.EXTRACT && intakeMid)
                {
                    intake.move(Intake.Angle.MID);
                    goToTransferTimer = getTime();
                }
                else if(getTime() - goToTransferTimer >= delayGoToTransfer)
                {
                    intake.move(Intake.Angle.TRANSFER);
                }


                break;
            case INTAKE:
                intake.move(Intake.Angle.INTAKE);
                intakeExtension.setCurrent(IntakeExtension.ExtensionState.CLOSE);

                if (extendIntake) {
                    intakeState = IntakeState.INTAKE_EXTEND;
                } else if (!releaseIntake) {
                    intakeState = IntakeState.RETRACT;
                }
                claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);


                if (robot.has2Pixels() && !startedDelayTransfer) {
                    had2Pixels = true;
                    transferTimer = getTime();

                    startedDelayTransfer = true;

                    intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
                }
                else if(robot.isCloseLeft() && !robot.has2Pixels())
                {
                    intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.LEFT);
                }
                else if(robot.isCloseRight() && !robot.has2Pixels())
                {
                    intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.RIGHT);
                }
                else if(!startedDelayTransfer)
                {
                    intake.updateClawState(Intake.ClawState.OPEN, ClawSide.BOTH);
                }

                if((getTime() - transferTimer) >= delayTransfer && startedDelayTransfer)
                {
                    intakeState = IntakeState.RETRACT;
                }

                break;
            case INTAKE_EXTEND:
                heldExtension = true;

                intakeExtension.setCurrent(IntakeExtension.ExtensionState.MANUAL);
                intake.move(Intake.Angle.INTAKE);
                claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);


                if (robot.has2Pixels() && !startedDelayTransfer) {
                    had2Pixels = true;

                    transferTimer = getTime();

                    startedDelayTransfer = true;

                    intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
                }
                else if(robot.isCloseLeft() && !robot.has2Pixels())
                {
                    intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.LEFT);
                }
                else if(robot.isCloseRight() && !robot.has2Pixels())
                {
                    intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.RIGHT);
                }
                else if(!startedDelayTransfer)
                {
                    intake.updateClawState(Intake.ClawState.OPEN, ClawSide.BOTH);
                }

                if((getTime() - transferTimer) >= delayTransfer && startedDelayTransfer)
                {
                    intakeState = IntakeState.INTAKE;
                }

//                if (!extendIntake) { we want to close intake only if we see pixels for now(inifnite loop if no seeing)
//                    intakeState = IntakeState.RETRACT;
//                }
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

                if (openElevator)
                {
                    intake.move(Intake.Angle.MID);
                    previousElevator = getTime();
                    claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH);
                    liftState = LiftState.EXTRACT;
                }
                break;
            case EXTRACT:
                canIntake = false;
                intakeState = IntakeState.RETRACT;

                elevator.setTarget(Elevator.BASE_LEVEL + (openedXTimes * Elevator.ELEVATOR_INCREMENT));

                if ((getTime() - previousElevator) >= WAIT_DELAY_TILL_OUTTAKE) {
                    outtake.setAngle(Outtake.Angle.OUTTAKE);
                }

                if (resetElevator)  {
                    openedXTimes++;
                    claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);

                    elevatorReset = getTime();
                    retract = true;
                } else if ((getTime() - elevatorReset) >= WAIT_DELAY_TILL_CLOSE && retract) {
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