package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotHardware;
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
public class BetterAutoRedLeft extends CommandOpMode
{
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
    enum IntakeLevel
    {
        TOP_54,
        TOP_32,
        TOP_21
    }

    IntakeLevel intakeLevel = IntakeLevel.TOP_54;

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

        placePurplePixel = drivetrain.trajectorySequenceBuilder(autoConstants.startPose)
                .forward(AutoConstants.strafeForPurplePixel)
                .addTemporalMarker(0.5, () -> intake.move(Intake.Angle.INTAKE))
                .waitSeconds(AutoConstants.WAIT)
                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.LEFT))
                .waitSeconds(.2)
                .addTemporalMarker(()-> intake.move(Intake.Angle.TOP_54))
                .build();
        intakeAndPlacePreload = drivetrain.trajectorySequenceBuilder(placePurplePixel.end())
                //Going for backdrop
                .lineToLinearHeading(new Pose2d(autoConstants.intakePixelVector.getX(), autoConstants.intakePixelVector.getY(),Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> intakeExtension.openExtension())
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.LEFT))
                .UNSTABLE_addTemporalMarkerOffset(1.4, () -> intakeExtension.closeExtension())
                .UNSTABLE_addTemporalMarkerOffset(1.9, () -> intake.move(Intake.Angle.TRANSFER))
                .waitSeconds(3.5)
                .lineToSplineHeading(autoConstants.stageDoorEndPose)
                .splineToLinearHeading(autoConstants.placePixelPose, Math.toRadians(0))
                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.BOTH))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH))
                .addTemporalMarker(() -> elevator.setTarget(Elevator.BASE_LEVEL))
                .addTemporalMarker(() -> elevator.update())
                .addTemporalMarker(() -> intake.move(Intake.Angle.MID))
                .addTemporalMarker(() -> outtake.setAngle(Outtake.Angle.OUTTAKE))
                .waitSeconds(1.5)
                .addTemporalMarker(() -> claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH))
                .waitSeconds(0.5)
                .build();

        intakeTraj = drivetrain.trajectorySequenceBuilder(intakeAndPlacePreload.end())
                .addTemporalMarker(() -> outtake.setAngle(Outtake.Angle.INTAKE))
                .addTemporalMarker(() -> elevator.setTarget(0))
                .addTemporalMarker(() -> elevator.update())
                .addTemporalMarker(() -> moveIntakeByTraj())
                .addTemporalMarker(2, () -> intakeExtension.openExtension())
                .lineToSplineHeading(autoConstants.stageDoorStartPose)
                .splineToLinearHeading(autoConstants.intakePixelVector, Math.toRadians(180))
                .waitSeconds(.2)
                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.LEFT))
                .waitSeconds(AutoConstants.WAIT)
                .build();

         placeSecond = drivetrain.trajectorySequenceBuilder(intakeTraj.end())
                 .addTemporalMarker(() -> intakeExtension.closeExtension())
                 .addTemporalMarker(0.5, () -> intake.move(Intake.Angle.TRANSFER))
                 .lineToSplineHeading(autoConstants.stageDoorEndPose)
                 .splineToLinearHeading(autoConstants.placePixelPose, Math.toRadians(0))
                 .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.BOTH))
                 .waitSeconds(0.5)
                 .addTemporalMarker(() -> claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH))
                 .addTemporalMarker(() -> elevator.setTarget(Elevator.BASE_LEVEL))
                 .addTemporalMarker(() -> elevator.update())
                 .addTemporalMarker(() -> intake.move(Intake.Angle.MID))
                 .addTemporalMarker(() -> outtake.setAngle(Outtake.Angle.OUTTAKE))
                 .waitSeconds(1.5)
                 .addTemporalMarker(() -> claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH))
                 .waitSeconds(0.5)
                 .addTemporalMarker(() -> elevator.setTarget(0))
                 .addTemporalMarker(() -> elevator.update())
                 .addTemporalMarker(() -> outtake.setAngle(Outtake.Angle.INTAKE))

                .build();
         park = drivetrain.trajectorySequenceBuilder(placeSecond.end())
                //Going for backdrop
                .lineToLinearHeading(autoConstants.park)
                 .addTemporalMarker(() -> intake.move(Intake.Angle.TRANSFER))
                .build();



        while (opModeInInit() && !isStopRequested())
        {
            intake.setAngle(Intake.Angle.TRANSFER);
            intakeExtension.closeExtension();
            intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
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
        drivetrain.followTrajectorySequence(placeSecond);
        intakeLevel = IntakeLevel.TOP_21;
        drivetrain.followTrajectorySequence(intakeTraj);
        drivetrain.followTrajectorySequence(placeSecond);
        drivetrain.followTrajectorySequence(park);
        while(opModeIsActive());
    }

    void moveIntakeByTraj()
    {
        switch (intakeLevel)
        {
            case TOP_54:
                intake.move(Intake.Angle.TOP_54);
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