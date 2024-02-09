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
        RETACT,
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
        RETRACT
    }

    ;

    enum IntakeState {
        READY,
        INTAKE,
        RETRACT,
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
    DepositeState depositeState = DepositeState.OPEN_ELEVATOR;
    IntakeState intakeState = IntakeState.READY;

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
    private void activateSystem(Runnable systemFunction, long delay, Object... parameters) {
        timer.reset();
        if (timer.hasTimePassed(delay)) {
            systemFunction.run();
            timer.reset();
        }
    }

    //FSM for every time u need to place a pixel.
    private void depositFSM() {
        timer = new Stopwatch();

        switch (depositeState) {
            case OPEN_ELEVATOR:
                activateSystem(() -> claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH), 0);
                activateSystem(() -> moveElevatorByTraj(), 500);
                depositeState = DepositeState.RESET_INTAKE_OUTTAKE;
                break;
            case RESET_INTAKE_OUTTAKE:
                activateSystem(() -> resetIntakeOuttake(), 500);
                depositeState = DepositeState.PLACE_PIXEL;
                break;
            case PLACE_PIXEL:
                activateSystem(() -> claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH), 1000);
                depositeState = DepositeState.RETRACT;
                break;
            case RETRACT:
                activateSystem(() -> retractElevator(), 800);
                break;
        }
    }

    //FSM for every time u need to intake.
    private void intakeFSM() {
        timer = new Stopwatch();

        switch (intakeState) {
            case READY:
                activateSystem(() -> moveElevatorByTraj(), 0);
                activateSystem(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.LEFT), 0);
                intakeState = IntakeState.INTAKE;
                break;
            case INTAKE:
                activateSystem(() -> intakeExtension.setTarget(300), 600);
                activateSystem(() -> intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.LEFT), 500);
                intakeState = IntakeState.RETRACT;
                break;
            case RETRACT:
                activateSystem(() -> intake.moveStack(), 0);
                activateSystem(() -> intake.move(Intake.Angle.OUTTAKE), 300);
                activateSystem(() -> intake.updateClawState(Intake.ClawState.INDETERMINATE, ClawSide.BOTH), 1000);
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
                .addTemporalMarker(() -> intake.move(Intake.Angle.INTAKE))

                .build();


        intakeAnotherPreload = drivetrain.trajectorySequenceBuilder(placePurplePixel.end())
                .lineToLinearHeading(new Pose2d(-40, -8.2, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> intake.updateClawState(Intake.ClawState.INDETERMINATE, ClawSide.BOTH))
                .build();

        placePreloadsOnBoard = drivetrain.trajectorySequenceBuilder(intakeAnotherPreload.end())

                // truss pose next to board
                .lineToSplineHeading(new Pose2d(12, -9, Math.toRadians(0)))

                .addSpatialMarker(new Vector2d(8, -6), () -> claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH))
                .addSpatialMarker(new Vector2d(9, -6), () -> elevator.setTarget(1050))
                .addSpatialMarker(new Vector2d(9, -6), () -> elevator.update())
                .addSpatialMarker(new Vector2d(10, -6), () -> intake.move(Intake.Angle.MID))
                .addSpatialMarker(new Vector2d(10, -6), () -> outtake.setAngle(Outtake.Angle.OUTTAKE))

                // backdrop pose
                .splineToLinearHeading(new Pose2d(53, -27, Math.toRadians(0)), Math.toRadians(0))
                .build();

        intakeCycle43 = drivetrain.trajectorySequenceBuilder(placePreloadsOnBoard.end())
                .addTemporalMarker(() -> outtake.setAngle(Outtake.Angle.INTAKE))
                .addTemporalMarker(() -> elevator.setTarget(0))
                .addTemporalMarker(() -> elevator.update())

                // truss pose next to wing
                .lineToSplineHeading(new Pose2d(30, -15, Math.toRadians(0)))

                .UNSTABLE_addDisplacementMarkerOffset(7, () -> moveIntakeByTraj())

                // intake pose
                .splineToLinearHeading(new Pose2d(-37.75, -10), Math.toRadians(180))

                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.LEFT))
                .addTemporalMarker(() -> intakeExtension.setTarget(100))
                .waitSeconds(0.8)
                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.LEFT))
                //.waitSeconds(AutoConstants.WAIT + 0.5)
                .waitSeconds(0.5)
                .build();

        place43 = drivetrain.trajectorySequenceBuilder(intakeCycle43.end())
                .addTemporalMarker(() -> intake.moveStack())
                .waitSeconds(0.3)
                .addTemporalMarker(() -> intake.move(Intake.Angle.OUTTAKE))
                .addTemporalMarker(() -> intakeExtension.setTarget(0))

                .addTemporalMarker(1, () -> intake.updateClawState(Intake.ClawState.INDETERMINATE, ClawSide.BOTH))

                .waitSeconds(0.3)

                // truss pose next to board
                .lineToSplineHeading(new Pose2d(12.5, -9, Math.toRadians(0)))

                .addSpatialMarker(new Vector2d(7, -6), () -> claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH))
                .addSpatialMarker(new Vector2d(5, -6), () -> elevator.setTarget(Elevator.BASE_LEVEL + 500))
                .addSpatialMarker(new Vector2d(5, -6), () -> elevator.update())
                .addSpatialMarker(new Vector2d(7, -10), () -> intake.move(Intake.Angle.MID))
                .addSpatialMarker(new Vector2d(15, -6), () -> outtake.setAngle(Outtake.Angle.OUTTAKE))

                // backdrop pose
                .splineToLinearHeading(new Pose2d(53, -28.8, Math.toRadians(0)), Math.toRadians(0))

                .waitSeconds(.2)
                .addTemporalMarker(() -> claw.updateState(Claw.ClawState.INTERMEDIATE, ClawSide.BOTH))
                .waitSeconds(0.1)
                //.forward(.5)
                .addTemporalMarker(() -> claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH))
                .waitSeconds(.2)
                .addTemporalMarker(() -> elevator.setTarget(0))
                .addTemporalMarker(() -> elevator.update())
                .addTemporalMarker(() -> outtake.setAngle(Outtake.Angle.INTAKE))
                .build();

        intakeCycle21 = drivetrain.trajectorySequenceBuilder(place43.end())
                .addTemporalMarker(() -> outtake.setAngle(Outtake.Angle.INTAKE))
                .addTemporalMarker(() -> elevator.setTarget(0))
                .addTemporalMarker(() -> elevator.update())

                // truss pose next to wing
                .lineToSplineHeading(new Pose2d(30, -15, Math.toRadians(0)))
                .UNSTABLE_addDisplacementMarkerOffset(7, () -> intake.move(Intake.Angle.OUTTAKE))

                // intake pose
                .splineToLinearHeading(new Pose2d(-37.7, -9), Math.toRadians(180))

                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.LEFT))
                .addTemporalMarker(() -> intakeExtension.setTarget(100))
                .addTemporalMarker(() -> intake.move(Intake.Angle.TOP_32_AUTO))
                .waitSeconds(.8)
                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.LEFT))
                //.waitSeconds(AutoConstants.WAIT + 0.4)
                .waitSeconds(0.5)
                .build();

        place21 = drivetrain.trajectorySequenceBuilder(intakeCycle21.end())
                .addTemporalMarker(() -> intake.moveStack())
                .waitSeconds(0.3)
                .addTemporalMarker(() -> intake.move(Intake.Angle.OUTTAKE))
                .addTemporalMarker(() -> intakeExtension.setTarget(0))

                .addTemporalMarker(1, () -> intake.updateClawState(Intake.ClawState.INDETERMINATE, ClawSide.BOTH))

                .waitSeconds(0.3)

                // truss pose next to board
                .lineToSplineHeading(new Pose2d(12, -9, Math.toRadians(0)))

                .addSpatialMarker(new Vector2d(9, -6), () -> claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH))
                .addSpatialMarker(new Vector2d(5, -6), () -> elevator.setTarget(Elevator.BASE_LEVEL + 550))
                .addSpatialMarker(new Vector2d(5, -6), () -> elevator.update())
                .addSpatialMarker(new Vector2d(7, -10), () -> intake.move(Intake.Angle.MID))
                .addSpatialMarker(new Vector2d(15, -6), () -> outtake.setAngle(Outtake.Angle.OUTTAKE))

                // backdrop pose
                .splineToLinearHeading(new Pose2d(53.75, -29.3, Math.toRadians(0)), Math.toRadians(0))

                .waitSeconds(.2)
                .addTemporalMarker(() -> claw.updateState(Claw.ClawState.INTERMEDIATE, ClawSide.BOTH))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH))
                .waitSeconds(.2)
                .addTemporalMarker(() -> elevator.setTarget(0))
                .addTemporalMarker(() -> elevator.update())
                .addTemporalMarker(() -> outtake.setAngle(Outtake.Angle.INTAKE))
                .build();

        park = drivetrain.trajectorySequenceBuilder(placePreloadsOnBoard.end())
                .lineTo(new Vector2d(50, -9))
                .lineToLinearHeading(new Pose2d(60, -10, Math.toRadians(-90)))
                .addTemporalMarker(() -> elevator.setTarget(0))
                .addTemporalMarker(() -> elevator.update())
                .addTemporalMarker(() -> outtake.setAngle(Outtake.Angle.INTAKE))
                .build();


        while (opModeInInit() && !isStopRequested()) {
            intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
            intake.setAngle(Intake.Angle.OUTTAKE);
            claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);
            outtake.setAngle(Outtake.Angle.INTAKE);
            telemetry.addLine("Initialized");
        }

        waitForStart();
        if (isStopRequested()) return;

        time.reset();


        double autoSeconds = time.seconds();
        while (opModeIsActive()) {
            drivetrain.update();
            elevator.update();
            intakeExtension.update();

            telemetry.addData("Auto seconds: ", autoSeconds);
            telemetry.update();
        }
    }


}