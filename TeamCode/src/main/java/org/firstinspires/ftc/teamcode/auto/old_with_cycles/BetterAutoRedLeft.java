package org.firstinspires.ftc.teamcode.auto.old_with_cycles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.roadrunner.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.util.ClawSide;

@Config
@Autonomous(name = "Better 2+5 Auto Red Left")
public class BetterAutoRedLeft extends LinearOpMode {
    VelocityConstraint smallVel;
    private final RobotHardware robot = RobotHardware.getInstance();

    public static int base = 700;
    ElapsedTime time;

    // subsystems
    SampleMecanumDrive drivetrain;
    Elevator elevator;
    Intake intake;
    Outtake outtake;
    Claw claw;
    IntakeExtension intakeExtension;
    AutoConstants autoConstants;
    TrajectorySequence placePurplePixel, intakeAnotherPreload, placePreloadsOnBoard, intakeCycle43, intakeCycle21, place43, place21;

    enum IntakeLevel {
        TOP_5,
        TOP_43,
        TOP_21
    }

    IntakeLevel intakeLevel = IntakeLevel.TOP_5;

    @Override
    public void runOpMode() {
        time = new ElapsedTime();
        CommandScheduler.getInstance().reset();
        drivetrain = new SampleMecanumDrive(hardwareMap);

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, telemetry, true);

        autoConstants = new AutoConstants();
        drivetrain.setPoseEstimate(autoConstants.startPoseRedLeft);

        elevator = new Elevator();
        outtake = new Outtake();
        claw = new Claw();
        intake = new Intake();
        intakeExtension = new IntakeExtension();

        smallVel = new VelocityConstraint() {
            @Override
            public double get(double v) {
                return 50;
            }
        };


        placePurplePixel = drivetrain.trajectorySequenceBuilder(autoConstants.startPoseRedLeft)

                // place purple pixel distance
                .forward(52)

                .addSpatialMarker(new Vector2d(-36, -39), () -> intake.move(Intake.Angle.INTAKE))
                .waitSeconds(AutoConstants.WAIT)
                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.LEFT))
                .waitSeconds(.1)
                .addTemporalMarker(() -> intake.move(Intake.Angle.TOP_5))
                .waitSeconds(.2)
                .build();

        intakeAnotherPreload = drivetrain.trajectorySequenceBuilder(placePurplePixel.end())
                .lineTo(new Vector2d(-39.8, -8.1))
                .turn(Math.toRadians(-90), Math.toRadians(180), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> intakeExtension.openExtension())
                .UNSTABLE_addTemporalMarkerOffset(1.7, () -> intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.LEFT))
                .UNSTABLE_addTemporalMarkerOffset(1.8, () -> intake.moveStack())
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(1.9, () -> intake.move(Intake.Angle.OUTTAKE))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(2, () -> intakeExtension.closeExtension())
                .waitSeconds(2.2)
                .addTemporalMarker(() -> DriveConstants.MAX_ANG_ACCEL = Math.toRadians(360))
                .waitSeconds(0.7)
                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.INDETERMINATE, ClawSide.BOTH))
                .build();

        placePreloadsOnBoard = drivetrain.trajectorySequenceBuilder(intakeAnotherPreload.end())

                // truss pose next to board
                .lineToSplineHeading(new Pose2d(12, -9, Math.toRadians(0)))

                .addSpatialMarker(new Vector2d(8, -6), () -> claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH))
                .addSpatialMarker(new Vector2d(10, -6), () -> elevator.setTarget(base))
                .addSpatialMarker(new Vector2d(10, -6), () -> elevator.update())
                .addSpatialMarker(new Vector2d(10, -6), () -> intake.move(Intake.Angle.MID))
                .addSpatialMarker(new Vector2d(10, -6), () -> outtake.setAngle(Outtake.Angle.OUTTAKE))

                // backdrop pose
                .splineToLinearHeading(new Pose2d(51.5, -30.8, Math.toRadians(0)), Math.toRadians(0))

                .waitSeconds(0.25)
                .addTemporalMarker(() -> claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH))
                .waitSeconds(0.1)
                .build();

        intakeCycle43 = drivetrain.trajectorySequenceBuilder(placePreloadsOnBoard.end())
                .addTemporalMarker(() -> outtake.setAngle(Outtake.Angle.INTAKE))
                .addTemporalMarker(() -> elevator.setTarget(0))
                .addTemporalMarker(() -> elevator.update())

                // truss pose next to wing
                .lineToSplineHeading(new Pose2d(30, -15, Math.toRadians(0)))

                .UNSTABLE_addDisplacementMarkerOffset(7, () -> moveIntakeByTraj())
                .UNSTABLE_addDisplacementMarkerOffset(20, () -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.LEFT))
                .UNSTABLE_addDisplacementMarkerOffset(35, () -> intakeExtension.openExtension())
                // intake pose
                .splineToLinearHeading(new Pose2d(-40.35, -9.335), Math.toRadians(180))
                .waitSeconds(.85)
                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.LEFT))
                .waitSeconds(0.4)
                .build();

        place43 = drivetrain.trajectorySequenceBuilder(intakeCycle43.end())
                .addTemporalMarker(() -> intake.moveStack())
                .waitSeconds(0.1)
                .addTemporalMarker(() -> intake.move(Intake.Angle.OUTTAKE))
                .addTemporalMarker(() -> intakeExtension.closeExtension())

                .addTemporalMarker(1, () -> intake.updateClawState(Intake.ClawState.INDETERMINATE, ClawSide.BOTH))


                // truss pose next to board
                .lineToSplineHeading(new Pose2d(12, -9, Math.toRadians(0)))

                .addSpatialMarker(new Vector2d(9, -6), () -> claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH))
                .addSpatialMarker(new Vector2d(5, -6), () -> elevator.setTarget(Elevator.BASE_LEVEL + 300))
                .addSpatialMarker(new Vector2d(5, -6), () -> elevator.update())
                .addSpatialMarker(new Vector2d(8, -10), () -> intake.move(Intake.Angle.MID))
                .addSpatialMarker(new Vector2d(15, -6), () -> outtake.setAngle(Outtake.Angle.OUTTAKE))

                // backdrop pose
                .splineToLinearHeading(new Pose2d(52.6, -28.8, Math.toRadians(0)), Math.toRadians(0))

                .waitSeconds(.2)
                .addTemporalMarker(() -> claw.updateState(Claw.ClawState.INTERMEDIATE, ClawSide.BOTH))
                .waitSeconds(0.1)
                //.forward(.5)
                .addTemporalMarker(() -> claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH))
                .waitSeconds(.1)
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

                .UNSTABLE_addDisplacementMarkerOffset(20, () -> intake.move(Intake.Angle.TOP_5))
                .UNSTABLE_addDisplacementMarkerOffset(40, () -> moveIntakeByTraj())
                .UNSTABLE_addDisplacementMarkerOffset(20, () -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.LEFT))
                .UNSTABLE_addDisplacementMarkerOffset(35, () -> intakeExtension.openExtension())
                // intake pose
                .splineToLinearHeading(new Pose2d(-41, -10.15), Math.toRadians(180))
                .waitSeconds(.85)
                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.LEFT))
                //.waitSeconds(AutoConstants.WAIT + 0.4)
                .waitSeconds(0.4)
                .build();

        place21 = drivetrain.trajectorySequenceBuilder(intakeCycle21.end())
                .addTemporalMarker(() -> intake.moveStack())
                .waitSeconds(0.1)
                .addTemporalMarker(() -> intake.move(Intake.Angle.OUTTAKE))
                .addTemporalMarker(() -> intakeExtension.closeExtension())

                .addTemporalMarker(1, () -> intake.updateClawState(Intake.ClawState.INDETERMINATE, ClawSide.BOTH))

                // truss pose next to board
                .lineToSplineHeading(new Pose2d(12, -9, Math.toRadians(0)))

                .addSpatialMarker(new Vector2d(7.5, -6), () -> claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH))
                .addSpatialMarker(new Vector2d(5, -6), () -> elevator.setTarget(Elevator.BASE_LEVEL + 350))
                .addSpatialMarker(new Vector2d(5, -6), () -> elevator.update())
                .addSpatialMarker(new Vector2d(8, -10), () -> intake.move(Intake.Angle.MID))
                .addSpatialMarker(new Vector2d(15, -6), () -> outtake.setAngle(Outtake.Angle.OUTTAKE))

                // backdrop pose
                .splineToLinearHeading(new Pose2d(53.5, -29.3, Math.toRadians(0)), Math.toRadians(0))

                .waitSeconds(.2)
                .addTemporalMarker(() -> claw.updateState(Claw.ClawState.INTERMEDIATE, ClawSide.BOTH))
                .waitSeconds(0.1)
                //.forward(.5)
                .addTemporalMarker(() -> claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH))
                .waitSeconds(.1)
                .addTemporalMarker(() -> elevator.setTarget(0))
                .addTemporalMarker(() -> elevator.update())
                .addTemporalMarker(() -> outtake.setAngle(Outtake.Angle.INTAKE))
                .build();


        while (opModeInInit() && !isStopRequested()) {
            intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
            intake.setAngle(Intake.Angle.OUTTAKE);
            intakeExtension.closeExtension();
            claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);
            outtake.setAngle(Outtake.Angle.INTAKE);
            telemetry.addLine("Initialized");
        }

        waitForStart();
        if (isStopRequested()) return;

        time.reset();

        drivetrain.followTrajectorySequence(placePurplePixel);
        drivetrain.followTrajectorySequence(intakeAnotherPreload);
        drivetrain.followTrajectorySequence(placePreloadsOnBoard);
        intakeLevel = IntakeLevel.TOP_43;
        drivetrain.followTrajectorySequence(intakeCycle43);
        drivetrain.followTrajectorySequence(place43);
        intakeLevel = IntakeLevel.TOP_21;
        drivetrain.followTrajectorySequence(intakeCycle21);
        drivetrain.followTrajectorySequence(place21);

        double autoSeconds = time.seconds();
        while (opModeIsActive())
        {
            drivetrain.update();

            telemetry.addData("Auto seconds: ", autoSeconds);
            telemetry.update();
        }
    }


    void moveIntakeByTraj() {
        switch (intakeLevel) {
            case TOP_5:
                intake.move(Intake.Angle.TOP_5);
                break;
            case TOP_43:
                intake.move(Intake.Angle.TOP_43);
                break;
            case TOP_21:
                intake.move(Intake.Angle.TOP_21);
                break;
        }
    }


}