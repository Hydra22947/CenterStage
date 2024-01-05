package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.auto.AutoConstants.ROBOT_ERROR_INTAKE_X;
import static org.firstinspires.ftc.teamcode.auto.AutoConstants.ROBOT_ERROR_INTAKE_Y;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.AccelerationConstraint;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
@Autonomous(name = "Better Auto Red Left")
public class BetterAutoRedLeft extends CommandOpMode {
    VelocityConstraint smallVel;
    private final RobotHardware robot = RobotHardware.getInstance();

    // subsystems
    SampleMecanumDrive drivetrain;
    Elevator elevator;
    Intake intake;
    Outtake outtake;
    Claw claw;
    IntakeExtension intakeExtension;
    AutoConstants autoConstants;
    TrajectorySequence placePurplePixel, intakeAnotherPreload, placePreloadsOnBoard, intakeCycle43, intakeCycle21, place43, place21, park;

    enum IntakeLevel {
        TOP_5,
        TOP_43,
        TOP_21
    }

    IntakeLevel intakeLevel = IntakeLevel.TOP_5;

    public void initialize() {
        CommandScheduler.getInstance().reset();
        drivetrain = new SampleMecanumDrive(hardwareMap);

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, telemetry);

        autoConstants = new AutoConstants();
        drivetrain.setPoseEstimate(autoConstants.startPoseRedLeft);

        elevator = new Elevator();
        outtake = new Outtake();
        claw = new Claw(this);
        intake = new Intake();
        intakeExtension = new IntakeExtension();

        smallVel = new VelocityConstraint() {
            @Override
            public double get(double v) {
                return 50;
            }
        };


        placePurplePixel = drivetrain.trajectorySequenceBuilder(autoConstants.startPoseRedLeft)

                .forward(52)

                .addSpatialMarker(new Vector2d(-36, -39), () -> intake.move(Intake.Angle.INTAKE))
                .waitSeconds(AutoConstants.WAIT)
                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.LEFT))
                .waitSeconds(.1)
                .addTemporalMarker(() -> intake.move(Intake.Angle.TOP_5))
                .waitSeconds(.2)
                .build();

        intakeAnotherPreload = drivetrain.trajectorySequenceBuilder(placePurplePixel.end())
                .addTemporalMarker(() -> DriveConstants.MAX_ANG_ACCEL = Math.toRadians(40))

                .lineToLinearHeading(new Pose2d(-38.8, -8.5, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(40, Math.toRadians(40), DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(40))

                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> intakeExtension.openExtension())
                .UNSTABLE_addTemporalMarkerOffset(1.7, () -> intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.LEFT))
                .UNSTABLE_addTemporalMarkerOffset(1.9, () -> intake.move(Intake.Angle.TRANSFER))
                .UNSTABLE_addTemporalMarkerOffset(2, () -> intakeExtension.closeExtension())
                .waitSeconds(2.3)
                .addTemporalMarker(() -> DriveConstants.MAX_ANG_ACCEL = Math.toRadians(360))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.BOTH))
                .waitSeconds(0.3)
                .addTemporalMarker(() -> claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH))
                .build();

        placePreloadsOnBoard = drivetrain.trajectorySequenceBuilder(placePurplePixel.end())

                .lineToSplineHeading(new Pose2d(12, -9, Math.toRadians(0)))

                .addSpatialMarker(new Vector2d(10, -6), () -> elevator.setTarget(Elevator.BASE_LEVEL))
                .addSpatialMarker(new Vector2d(10, -6), () -> elevator.update())
                .addSpatialMarker(new Vector2d(10, -6), () -> intake.move(Intake.Angle.MID))
                .addSpatialMarker(new Vector2d(10, -6), () -> outtake.setAngle(Outtake.Angle.OUTTAKE))

                .splineToLinearHeading(new Pose2d(51.4, -28.8, Math.toRadians(0)), Math.toRadians(0))

                .waitSeconds(0.6)
                .addTemporalMarker(() -> claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH))
                .waitSeconds(0.3)
                .build();

        intakeCycle43 = drivetrain.trajectorySequenceBuilder(placePreloadsOnBoard.end())
                .addTemporalMarker(() -> outtake.setAngle(Outtake.Angle.INTAKE))
                .addTemporalMarker(() -> elevator.setTarget(0))
                .addTemporalMarker(() -> elevator.update())

                .lineToSplineHeading(new Pose2d(30, -15, Math.toRadians(0)))

                .UNSTABLE_addDisplacementMarkerOffset(autoConstants.TEMP, () -> moveIntakeByTraj())

                .splineToLinearHeading(new Pose2d(-39, -9.9), Math.toRadians(180))

                .addTemporalMarker(() -> intakeExtension.openExtension())
                .waitSeconds(1)
                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.LEFT))
                .waitSeconds(AutoConstants.WAIT + 0.5)
                .build();

        place43 = drivetrain.trajectorySequenceBuilder(intakeCycle43.end())
                .addTemporalMarker(() -> intakeExtension.closeExtension())
                .waitSeconds(0.1)
                .addTemporalMarker(() -> intake.move(Intake.Angle.TRANSFER))
                .addTemporalMarker(1, () -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.BOTH))

                .lineToSplineHeading(new Pose2d(12, -9, Math.toRadians(0)))

                .addSpatialMarker(new Vector2d(6, -6), () -> claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH))
                .addSpatialMarker(new Vector2d(5, -6), () -> elevator.setTarget(Elevator.BASE_LEVEL + 250))
                .addSpatialMarker(new Vector2d(5, -6), () -> elevator.update())
                .addSpatialMarker(new Vector2d(8, -10), () -> intake.move(Intake.Angle.MID))
                .addSpatialMarker(new Vector2d(15, -6), () -> outtake.setAngle(Outtake.Angle.OUTTAKE))

                .splineToLinearHeading(new Pose2d(51.4, -28.8, Math.toRadians(0)), Math.toRadians(0))

                .waitSeconds(.4)
                .addTemporalMarker(() -> claw.updateState(Claw.ClawState.INTERMEDIATE, ClawSide.BOTH))
                .waitSeconds(0.2)
                .forward(.5)
                .addTemporalMarker(() -> claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH))
                .waitSeconds(.3)
                .addTemporalMarker(() -> elevator.setTarget(0))
                .addTemporalMarker(() -> elevator.update())
                .addTemporalMarker(() -> outtake.setAngle(Outtake.Angle.INTAKE))
                .build();

        intakeCycle21 = drivetrain.trajectorySequenceBuilder(place43.end())
                .addTemporalMarker(() -> outtake.setAngle(Outtake.Angle.INTAKE))
                .addTemporalMarker(() -> elevator.setTarget(0))
                .addTemporalMarker(() -> elevator.update())

                .lineToSplineHeading(new Pose2d(30, -15, Math.toRadians(0)))

                .UNSTABLE_addDisplacementMarkerOffset(autoConstants.TEMP, () -> moveIntakeByTraj())

                .splineToLinearHeading(new Pose2d(-40, -9.9), Math.toRadians(180))

                .addTemporalMarker(() -> intakeExtension.openExtension())
                .waitSeconds(1)
                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.LEFT))
                .waitSeconds(AutoConstants.WAIT + 0.5)
                .build();

        place21 = drivetrain.trajectorySequenceBuilder(intakeCycle21.end())
                .addTemporalMarker(() -> intakeExtension.closeExtension())
                .waitSeconds(0.1)
                .addTemporalMarker(() -> intake.move(Intake.Angle.TRANSFER))
                .addTemporalMarker(1, () -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.BOTH))

                .lineToSplineHeading(new Pose2d(12, -9, Math.toRadians(0)))

                .addSpatialMarker(new Vector2d(6, -6), () -> claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH))
                .addSpatialMarker(new Vector2d(5, -6), () -> elevator.setTarget(Elevator.BASE_LEVEL + 250))
                .addSpatialMarker(new Vector2d(5, -6), () -> elevator.update())
                .addSpatialMarker(new Vector2d(8, -10), () -> intake.move(Intake.Angle.MID))
                .addSpatialMarker(new Vector2d(15, -6), () -> outtake.setAngle(Outtake.Angle.OUTTAKE))

                .splineToLinearHeading(new Pose2d(51.9, -26.8, Math.toRadians(0)), Math.toRadians(0))

                .waitSeconds(.4)
                .addTemporalMarker(() -> claw.updateState(Claw.ClawState.INTERMEDIATE, ClawSide.BOTH))
                .waitSeconds(0.2)
                .forward(.5)
                .addTemporalMarker(() -> claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH))
                .waitSeconds(.3)
                .addTemporalMarker(() -> elevator.setTarget(0))
                .addTemporalMarker(() -> elevator.update())
                .addTemporalMarker(() -> outtake.setAngle(Outtake.Angle.INTAKE))
                .build();

        park = drivetrain.trajectorySequenceBuilder(place21.end())
                //Going for backdrop
                .lineToLinearHeading(new Pose2d(51.5, -5, Math.toRadians(90)))
                .addTemporalMarker(() -> intake.move(Intake.Angle.TRANSFER))
                .build();


        while (opModeInInit() && !isStopRequested()) {
            intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
            intake.setAngle(Intake.Angle.TRANSFER);
            intakeExtension.closeExtension();
            claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);
            outtake.setAngle(Outtake.Angle.INTAKE);
            telemetry.addLine("Initialized");
        }
    }

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();
        if (isStopRequested()) return;
        drivetrain.followTrajectorySequence(placePurplePixel);
        drivetrain.followTrajectorySequence(intakeAnotherPreload);
        drivetrain.followTrajectorySequence(placePreloadsOnBoard);
        intakeLevel = IntakeLevel.TOP_43;
        drivetrain.followTrajectorySequence(intakeCycle43);
        drivetrain.followTrajectorySequence(place43);
        intakeLevel = IntakeLevel.TOP_21;
        drivetrain.followTrajectorySequence(intakeCycle21);
        drivetrain.followTrajectorySequence(place21);
        //drivetrain.followTrajectorySequence(park);
        while (opModeIsActive()) ;
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