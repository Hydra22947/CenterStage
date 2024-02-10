package org.firstinspires.ftc.teamcode.auto.machines;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.Main;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.auto.PoseStorage;
import org.firstinspires.ftc.teamcode.auto.old_with_cycles.AutoConstants;
import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.firstinspires.ftc.teamcode.util.Stopwatch;

import java.util.function.Consumer;

@Config
@Autonomous(name = "autoFSM")
public class FSMauto extends LinearOpMode {
    VelocityConstraint smallVel;
    private final RobotHardware robot = RobotHardware.getInstance();
    Stopwatch timer;
    ElapsedTime time;

    // subsystems
    SampleMecanumDrive drivetrain;
    static Elevator elevator;
    Intake intake;
    Outtake outtake;
    Claw claw;
    IntakeExtension intakeExtension;
    AutoConstants autoConstants;
    TrajectorySequence placePurplePixel, intakeAnotherPreload, placePreloadsOnBoard, intakeCycle43, intakeCycle21, place43, place21, park;


    enum AutoState {
        PLACE_PURPLE,
        RETRACT,
        PLACE_PRELOAD,
        PLACE_PIXEL,
        INTAKE,
        PARK

    }

    ;

    enum IntakeLevel {
        TOP_54,
        TOP_32
    }

    ;


    enum ElevatorLevel {
        TOP_54,
        TOP_32
    }

    ;

    public enum DepositeState {
        OPEN_ELEVATOR,
        RESET_INTAKE_OUTTAKE,
        PLACE_PIXEL,
        RETRACT,
        IDLE
    }

    ;

    enum IntakeState {
        PLACE_PURPLE,
        READY,
        INTAKE,
        RETRACT,
        IDLE
    }

    ;

    enum Cycles {
        PRELOAD,
        FIRST_CYCLE,
        SECOND_CYCLE
    }

    ;

    IntakeLevel intakeLevel = IntakeLevel.TOP_54;
    Cycles currentCycle = Cycles.PRELOAD;
    DepositeState depositeState = DepositeState.IDLE;
    IntakeState intakeState = IntakeState.IDLE;
    AutoState autoState = AutoState.PLACE_PURPLE;

    void moveElevatorByTraj() {
        switch (currentCycle) {
            case PRELOAD:
                elevator.move(1050);
                break;
            case FIRST_CYCLE:
                elevator.move(1500);
                break;
            case SECOND_CYCLE:
                elevator.move(1550);
                break;
        }
    }

    void moveIntakeByTraj() {
        switch (intakeLevel) {
            case TOP_54:
                intake.move(Intake.Angle.TOP_54_AUTO);
                break;
            case TOP_32:
                intake.move(Intake.Angle.TOP_32_AUTO);
                break;
        }
    }

    //This function will prepare the intake and outtake  for deposit
    private void resetIntakeOuttake() {
        intake.move(Intake.Angle.MID);
        outtake.setAngle(Outtake.Angle.OUTTAKE);
    }


    public void retractElevator() {
        outtake.setAngle(Outtake.Angle.INTAKE);
        elevator.move(0);
    }

    //This function will get the function, its parameters and the delay and execute
    //this function with the delay.
    private boolean activateSystem(Runnable systemFunction, long delay, Object... parameters) {
        if (timer.hasTimePassed(delay)) {
            systemFunction.run();
            timer.reset();
            return true; // Activation successful
        } else {
            return false; // Activation failed
        }
    }

    private void depositFSM() {
        timer = new Stopwatch();

        switch (depositeState) {
            case RESET_INTAKE_OUTTAKE:
                timer.reset();
                if (!activateSystem(() -> claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH), 0))
                    return;
                if (!activateSystem(() -> resetIntakeOuttake(), 200)) return;

                depositeState = DepositeState.RESET_INTAKE_OUTTAKE;
                break;
            case OPEN_ELEVATOR:
                if (!activateSystem(() -> moveElevatorByTraj(), 100)) return;
                depositeState = DepositeState.PLACE_PIXEL;
                break;
            case PLACE_PIXEL:
                if (!activateSystem(() -> claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH), 1000))
                    return;
                depositeState = DepositeState.RETRACT;
                break;
            case RETRACT:
                if (!activateSystem(() -> retractElevator(), 800)) return;
                depositeState = DepositeState.IDLE;
                break;
            case IDLE:
                break;
        }

    }

    //FSM for every time u need to intake.
    private void intakeFSM() {

        switch (intakeState) {
            case PLACE_PURPLE:
                timer.reset();
                if (!activateSystem(() -> intake.move(Intake.Angle.INTAKE), 0)) return;
                if (!activateSystem(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.LEFT), 200))
                    return;
                if (!activateSystem(() -> intake.move(Intake.Angle.OUTTAKE), 0)) return;
                break;
            case READY:
                if (!activateSystem(() -> moveElevatorByTraj(), 0)) return;
                if (!activateSystem(() -> intake.move(Intake.Angle.INTAKE), 0)) return;
                if (!activateSystem(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.LEFT), 0))
                    return;

                intakeState = IntakeState.INTAKE;
                break;
            case INTAKE:
                if (!activateSystem(() -> intakeExtension.setTarget(300), 600)) return;
                if (!activateSystem(() -> intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.LEFT), 500))
                    return;

                intakeState = IntakeState.RETRACT;
                break;
            case RETRACT:
                if (!activateSystem(() -> intake.moveStack(), 0)) return;
                if (!activateSystem(() -> intake.move(Intake.Angle.OUTTAKE), 300)) return;
                if (!activateSystem(() -> intake.updateClawState(Intake.ClawState.INDETERMINATE, ClawSide.BOTH), 1000))
                    return;
                intakeState = IntakeState.IDLE;
                break;
            case IDLE:
                break;
        }
    }

    private void autoFSM() {
        switch (autoState) {
            case PLACE_PURPLE:
                if (drivetrain.isNotBusy()) {
                    autoState = AutoState.PLACE_PRELOAD;
                    drivetrain.followTrajectorySequenceAsync(placePurplePixel);
                    intakeState = IntakeState.PLACE_PURPLE;
                }
                break;
            case PLACE_PRELOAD:
                if (drivetrain.isNotBusy()) {
                    depositeState = DepositeState.RESET_INTAKE_OUTTAKE;
                    autoState = AutoState.PARK;
                    drivetrain.followTrajectorySequenceAsync(placePreloadsOnBoard);
                    currentCycle = Cycles.PRELOAD;

                }
                break;
            case PARK:
                if (drivetrain.isNotBusy()) {
                    drivetrain.followTrajectorySequenceAsync(park);
                }
                break;

        }
    }

    @Override
    public void runOpMode() {
        time = new ElapsedTime();
        CommandScheduler.getInstance().reset();
        drivetrain = new SampleMecanumDrive(hardwareMap);

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, telemetry, true);

        autoConstants = new AutoConstants();
        drivetrain.setPoseEstimate(autoConstants.startPoseRedLeft);

        elevator = new Elevator(true);
        outtake = new Outtake();
        claw = new Claw();
        intake = new Intake();
        intakeExtension = new IntakeExtension(true);

        smallVel = new VelocityConstraint() {
            @Override
            public double get(double v) {
                return 50;
            }
        };


        //Trajectories

        placePurplePixel = drivetrain.trajectorySequenceBuilder(autoConstants.startPoseRedLeft)


                // place purple pixel distance

                .lineToLinearHeading(new Pose2d(-45.5, -15, Math.toRadians(80)))
                .lineToLinearHeading(new Pose2d(-40, -8.2, Math.toRadians(0)))

                .build();


        placePreloadsOnBoard = drivetrain.trajectorySequenceBuilder(intakeAnotherPreload.end())

                // truss pose next to board
                .lineToSplineHeading(new Pose2d(12, -9, Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(53, -27, Math.toRadians(0)), Math.toRadians(0))
                .build();


        park = drivetrain.trajectorySequenceBuilder(placePreloadsOnBoard.end())
                .lineTo(new Vector2d(50, -9))
                .lineToLinearHeading(new Pose2d(60, -10, Math.toRadians(-90)))
                .build();


        elevator.setAuto(true);
        intakeExtension.setAuto(true);
        while (opModeInInit() && !isStopRequested()) {
            intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
            intake.setAngle(Intake.Angle.OUTTAKE);
            claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);
            outtake.setAngle(Outtake.Angle.INTAKE);
            telemetry.addLine("Initialized");
        }

        waitForStart();

        if (isStopRequested()) {
            return;
        }


        time.reset();


        double autoSeconds = time.seconds();
        while (opModeIsActive()) {
            autoFSM();
            intakeFSM();
            depositFSM();
            drivetrain.update();

            telemetry.addData("Auto seconds: ", autoSeconds);
            telemetry.update();
        }
    }


}