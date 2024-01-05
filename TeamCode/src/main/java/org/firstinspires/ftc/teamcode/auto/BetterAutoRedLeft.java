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
    AccelerationConstraint smallAccel;
    private final RobotHardware robot = RobotHardware.getInstance();

    // subsystems
    SampleMecanumDrive drivetrain;
    Elevator elevator;
    Intake intake;
    Outtake outtake;
    Claw claw;
    IntakeExtension intakeExtension;
    AutoConstants autoConstants;
    TrajectorySequence placePurplePixel, intakeAndPlacePreload, intakeTraj, placeSecond, park;

    enum IntakeLevel {
        TOP_5,
        TOP_32,
        TOP_21
    }

    IntakeLevel intakeLevel = IntakeLevel.TOP_5;

    public void initialize() {
        CommandScheduler.getInstance().reset();
        drivetrain = new SampleMecanumDrive(hardwareMap);

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, telemetry);

        autoConstants = new AutoConstants();
        drivetrain.setPoseEstimate(autoConstants.startPose);

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


        placePurplePixel = drivetrain.trajectorySequenceBuilder(autoConstants.startPose)
                .forward(AutoConstants.strafeForPurplePixel)
                .addSpatialMarker(new Vector2d(-36, -39), () -> intake.move(Intake.Angle.INTAKE))
                .waitSeconds(AutoConstants.WAIT)
                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.LEFT))
                .waitSeconds(.1)
                .addTemporalMarker(() -> intake.move(Intake.Angle.TOP_5))
                .waitSeconds(.2)
                .build();

        intakeAndPlacePreload = drivetrain.trajectorySequenceBuilder(placePurplePixel.end())
                //Going for backdrop
                .addTemporalMarker(() -> DriveConstants.MAX_ANG_ACCEL = Math.toRadians(40))
                .lineToLinearHeading(new Pose2d(autoConstants.intakePixelVector.getX(), autoConstants.intakePixelVector.getY() - 0.15, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(40, Math.toRadians(40), DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> intakeExtension.openExtension())
                .UNSTABLE_addTemporalMarkerOffset(1.7, () -> intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.LEFT))
                .UNSTABLE_addTemporalMarkerOffset(1.9, () -> intake.move(Intake.Angle.TRANSFER))
                .UNSTABLE_addTemporalMarkerOffset(2, () -> intakeExtension.closeExtension())
                .waitSeconds(2.5)
                .addTemporalMarker(() -> DriveConstants.MAX_ANG_ACCEL = Math.toRadians(360))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.BOTH))
                .waitSeconds(0.3)
                .addTemporalMarker(() -> claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH))
                .lineToSplineHeading(autoConstants.stageDoorEndPose)
                .addSpatialMarker(new Vector2d(10, -6), () -> elevator.setTarget(Elevator.BASE_LEVEL))
                .addSpatialMarker(new Vector2d(10, -6), () -> elevator.update())
                .addSpatialMarker(new Vector2d(10, -6), () -> intake.move(Intake.Angle.MID))
                .addSpatialMarker(new Vector2d(10, -6), () -> outtake.setAngle(Outtake.Angle.OUTTAKE))
                .splineToLinearHeading(autoConstants.placePixelPose, Math.toRadians(0))
                .waitSeconds(0.8)
                .addTemporalMarker(() -> claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH))
                .waitSeconds(0.5)
                .build();

        intakeTraj = drivetrain.trajectorySequenceBuilder(intakeAndPlacePreload.end())
                .addTemporalMarker(() -> outtake.setAngle(Outtake.Angle.INTAKE))
                .addTemporalMarker(() -> elevator.setTarget(0))
                .addTemporalMarker(() -> elevator.update())
                .lineToSplineHeading(autoConstants.stageDoorStartPose)
                .UNSTABLE_addDisplacementMarkerOffset(autoConstants.TEMP, () -> moveIntakeByTraj())
                .splineToLinearHeading(new Pose2d(autoConstants.intakePixelVector.getX() + ROBOT_ERROR_INTAKE_X, autoConstants.intakePixelVector.getY() + ROBOT_ERROR_INTAKE_Y), Math.toRadians(180))
                .addTemporalMarker(() -> intakeExtension.openExtension())
                .waitSeconds(1)
                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.LEFT))
                .waitSeconds(AutoConstants.WAIT + 0.5)
                .build();

        placeSecond = drivetrain.trajectorySequenceBuilder(intakeTraj.end())
                .addTemporalMarker(() -> intakeExtension.closeExtension())
                .waitSeconds(0.1)
                .addTemporalMarker(() -> intake.move(Intake.Angle.TRANSFER))
                .addTemporalMarker(1, () -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.BOTH))
                .lineToSplineHeading(autoConstants.stageDoorEndPose)
                .addSpatialMarker(new Vector2d(6, -6), () -> claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH))
                .addSpatialMarker(new Vector2d(5, -6), () -> elevator.setTarget(Elevator.BASE_LEVEL + 250))
                .addSpatialMarker(new Vector2d(5, -6), () -> elevator.update())
                .addSpatialMarker(new Vector2d(8, -10), () -> intake.move(Intake.Angle.MID))
                .addSpatialMarker(new Vector2d(15, -6), () -> outtake.setAngle(Outtake.Angle.OUTTAKE))
                .splineToLinearHeading(autoConstants.placePixelPose, Math.toRadians(0))
                .waitSeconds(.5)
                .addTemporalMarker(() -> claw.updateState(Claw.ClawState.INTERMEDIATE, ClawSide.BOTH))
                .waitSeconds(0.2)
                .forward(.5)
                .addTemporalMarker(() -> claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH))
                .waitSeconds(.5)
                .addTemporalMarker(() -> elevator.setTarget(0))
                .addTemporalMarker(() -> elevator.update())
                .addTemporalMarker(() -> outtake.setAngle(Outtake.Angle.INTAKE))

                .build();
        park = drivetrain.trajectorySequenceBuilder(placeSecond.end())
                //Going for backdrop
                .lineToLinearHeading(autoConstants.park)
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
        drivetrain.followTrajectorySequence(intakeAndPlacePreload);
        intakeLevel = IntakeLevel.TOP_32;
        drivetrain.followTrajectorySequence(intakeTraj);
        autoConstants.ROBOT_ERROR_INTAKE_X -= 1;
        drivetrain.followTrajectorySequence(placeSecond);
        intakeLevel = IntakeLevel.TOP_21;
        autoConstants.placePixelPoseY += 2;
        autoConstants.placePixelPoseX += .5;
        drivetrain.followTrajectorySequence(intakeTraj);
        drivetrain.followTrajectorySequence(placeSecond);
        //drivetrain.followTrajectorySequence(park);
        while (opModeIsActive()) ;
    }

    void moveIntakeByTraj() {
        switch (intakeLevel) {
            case TOP_5:
                intake.move(Intake.Angle.TOP_5);
                break;
            case TOP_32:
                intake.move(Intake.Angle.TOP_32);
                break;
            case TOP_21:
                intake.move(Intake.Angle.TOP_21);
                break;
        }
    }


}