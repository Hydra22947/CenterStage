package org.firstinspires.ftc.teamcode.auto.RedLeft;

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
import org.firstinspires.ftc.teamcode.auto.old_with_cycles.AutoConstants;
import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.util.ClawSide;

@Config
@Autonomous(name = "2+5 Auto Red Left")
public class AutoRedLeft extends LinearOpMode {
    VelocityConstraint smallVel;
    private final RobotHardware robot = RobotHardware.getInstance();

    ElapsedTime time;

    // subsystems
    SampleMecanumDrive drivetrain;
    Elevator elevator;
    Intake intake;
    Outtake outtake;
    Claw claw;
    IntakeExtension intakeExtension;
    AutoConstants autoConstants;
    TrajectorySequence placePurplePixel, intakeAnotherPreload, placePreloadsOnBoard, intakeCycle43, intakeCycle21, place43, place21, park;

    public enum IntakeLevel {
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

        robot.init(hardwareMap, telemetry);

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
                .lineTo(new Vector2d(-40, -8.2))
                .turn(Math.toRadians(-90), Math.toRadians(90), Math.toRadians(75))
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> intakeExtension.openExtension())
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.LEFT))
                .UNSTABLE_addTemporalMarkerOffset(1.35, () -> intake.moveStack())
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(1.4, () -> intake.move(Intake.Angle.OUTTAKE))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(1.7, () -> intakeExtension.closeExtension())
                .waitSeconds(2.4)
                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.INDETERMINATE, ClawSide.BOTH))
                .build();

        placePreloadsOnBoard = drivetrain.trajectorySequenceBuilder(intakeAnotherPreload.end())

                // truss pose next to board
                .lineToSplineHeading(new Pose2d(12, -9, Math.toRadians(0)))

                .addSpatialMarker(new Vector2d(8, -6), () -> claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH))
                .addSpatialMarker(new Vector2d(9, -6), () -> elevator.setTarget(700))
                .addSpatialMarker(new Vector2d(9, -6), () -> elevator.update())
                .addSpatialMarker(new Vector2d(10, -6), () -> intake.move(Intake.Angle.MID))
                .addSpatialMarker(new Vector2d(10, -6), () -> outtake.setAngle(Outtake.Angle.OUTTAKE))

                // backdrop pose
                .splineToLinearHeading(new Pose2d(52, -32.75, Math.toRadians(0)), Math.toRadians(0))

                .waitSeconds(0.25)
                .addTemporalMarker(() -> claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH))
                .waitSeconds(0.15)
                .build();

        intakeCycle43 = drivetrain.trajectorySequenceBuilder(placePreloadsOnBoard.end())
                .addTemporalMarker(() -> outtake.setAngle(Outtake.Angle.INTAKE))
                .addTemporalMarker(() -> elevator.setTarget(0))
                .addTemporalMarker(() -> elevator.update())

                // truss pose next to wing
                .lineToSplineHeading(new Pose2d(30, -15, Math.toRadians(0)))

                .UNSTABLE_addDisplacementMarkerOffset(7, () -> moveIntakeByTraj())

                // intake pose
                .splineToLinearHeading(new Pose2d(-40.55, -9.8), Math.toRadians(180))

                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.LEFT))
                .addTemporalMarker(() -> intakeExtension.openExtension())
                .waitSeconds(1)
                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.LEFT))
                //.waitSeconds(AutoConstants.WAIT + 0.5)
                .waitSeconds(0.5)
                .build();

        place43 = drivetrain.trajectorySequenceBuilder(intakeCycle43.end())
                .addTemporalMarker(() -> intake.moveStack())
                .waitSeconds(0.3)
                .addTemporalMarker(() -> intake.move(Intake.Angle.OUTTAKE))
                .addTemporalMarker(() -> intakeExtension.closeExtension())

                .addTemporalMarker(1, () -> intake.updateClawState(Intake.ClawState.INDETERMINATE, ClawSide.BOTH))

                .waitSeconds(0.3)

                // truss pose next to board
                .lineToSplineHeading(new Pose2d(12.5, -9, Math.toRadians(0)))

                .addSpatialMarker(new Vector2d(9, -6), () -> claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH))
                .addSpatialMarker(new Vector2d(5, -6), () -> elevator.setTarget(Elevator.BASE_LEVEL + 200))
                .addSpatialMarker(new Vector2d(5, -6), () -> elevator.update())
                .addSpatialMarker(new Vector2d(8, -10), () -> intake.move(Intake.Angle.MID))
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

                .UNSTABLE_addDisplacementMarkerOffset(20, () -> intake.move(Intake.Angle.TOP_5))

                // intake pose
                .splineToLinearHeading(new Pose2d(-41, -10.35), Math.toRadians(180))

                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.LEFT))
                .addTemporalMarker(() -> intakeExtension.openExtension())
                .waitSeconds(.25)
                .addTemporalMarker(() -> intake.move(Intake.Angle.TOP_21))
                .waitSeconds(1)
                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.LEFT))
                //.waitSeconds(AutoConstants.WAIT + 0.4)
                .waitSeconds(0.5)
                .build();

        place21 = drivetrain.trajectorySequenceBuilder(intakeCycle21.end())
                .addTemporalMarker(() -> intake.moveStack())
                .waitSeconds(0.3)
                .addTemporalMarker(() -> intake.move(Intake.Angle.OUTTAKE))
                .addTemporalMarker(() -> intakeExtension.closeExtension())

                .addTemporalMarker(1, () -> intake.updateClawState(Intake.ClawState.INDETERMINATE, ClawSide.BOTH))

                .waitSeconds(0.3)

                // truss pose next to board
                .lineToSplineHeading(new Pose2d(12, -9, Math.toRadians(0)))

                .addSpatialMarker(new Vector2d(7.5, -6), () -> claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH))
//                .addSpatialMarker(new Vector2d(5, -6), () -> elevator.setTarget(Elevator.BASE_LEVEL + 250))
//                .addSpatialMarker(new Vector2d(5, -6), () -> elevator.update())
//                .addSpatialMarker(new Vector2d(8, -10), () -> intake.move(Intake.Angle.MID))
//                .addSpatialMarker(new Vector2d(15, -6), () -> outtake.setAngle(Outtake.Angle.OUTTAKE))

                .splineToLinearHeading(new Pose2d(51.5, -5, Math.toRadians(0)), Math.toRadians(0))
                // backdrop pose
//                .splineToLinearHeading(new Pose2d(53.75, -29.3, Math.toRadians(0)), Math.toRadians(0))
//
//                .waitSeconds(.2)
//                .addTemporalMarker(() -> claw.updateState(Claw.ClawState.INTERMEDIATE, ClawSide.BOTH))
//                .waitSeconds(0.1)
//                .addTemporalMarker(() -> claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH))
//                .waitSeconds(.2)
//                .addTemporalMarker(() -> elevator.setTarget(0))
//                .addTemporalMarker(() -> elevator.update())
//                .addTemporalMarker(() -> outtake.setAngle(Outtake.Angle.INTAKE))
                .build();

//        park = drivetrain.trajectorySequenceBuilder(place21.end())
//                //go park
//                .lineToLinearHeading(new Pose2d(51.5, -5, Math.toRadians(90)))
//
//                .addTemporalMarker(() -> intake.move(Intake.Angle.OUTTAKE))
//                .build();


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