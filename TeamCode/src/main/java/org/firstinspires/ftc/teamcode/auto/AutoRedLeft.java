package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpMode;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.testing.harman.PoseStorage;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.ClawSide;

@Config
@Autonomous(name = "Auto Red Left")
public class AutoRedLeft extends CommandOpMode
{
    private final RobotHardware robot = RobotHardware.getInstance();

    // subsystems
    SampleMecanumDrive drivetrain;
    Elevator elevator;
    Intake intake;
    Outtake outtake;
    Claw claw;
    IntakeExtension intakeExtension;

    MarkersAuto markersAuto;
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

        markersAuto = new MarkersAuto(outtake, elevator, claw, intakeExtension , intake);

        placePurplePixel = drivetrain.trajectorySequenceBuilder(autoConstants.startPose)
                .forward(AutoConstants.strafeForPurplePixel)
                .addTemporalMarker(0.5 ,markersAuto.placePurplePixelMarker)
                .build();
        intakeAndPlacePreload = drivetrain.trajectorySequenceBuilder(placePurplePixel.end())
                //Going for backdrop
                .lineToLinearHeading(new Pose2d(autoConstants.intakePixelVector.getX(), autoConstants.intakePixelVector.getY(),Math.toRadians(0)))
                .addTemporalMarker(markersAuto.openIntake)
                .waitSeconds(0.1)
                .addTemporalMarker(markersAuto.closeIntake)
                .lineToSplineHeading(autoConstants.stageDoorEndPose)
                .splineToLinearHeading(autoConstants.placePixelPose, Math.toRadians(0))
                .addTemporalMarker(markersAuto.openElevatorLOW)
                .UNSTABLE_addTemporalMarkerOffset(.2, () -> outtake.setAngle(Outtake.Angle.OUTTAKE))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH))
                .waitSeconds(0.1)
                .build();

        intakeTraj = drivetrain.trajectorySequenceBuilder(intakeAndPlacePreload.end())
                .addTemporalMarker(markersAuto.closeElevator)
                .addTemporalMarker(() -> moveIntakeByTraj())
                .addTemporalMarker(2, () -> intakeExtension.openExtension())
                .lineToSplineHeading(autoConstants.stageDoorStartPose)
                .splineToLinearHeading(autoConstants.intakePixelVector, Math.toRadians(180))
                .waitSeconds(.2)
                .build();

         placeSecond = drivetrain.trajectorySequenceBuilder(intakeTraj.end())
                 .waitSeconds(AutoConstants.WAIT)
                 .addTemporalMarker(markersAuto.closeIntake)
                 .lineToSplineHeading(autoConstants.stageDoorEndPose)
                 .splineToLinearHeading(autoConstants.placePixelPose, Math.toRadians(0))
                 .addTemporalMarker(() -> claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH))
                 .addTemporalMarker(markersAuto.openElevatorLOW)
                 .UNSTABLE_addTemporalMarkerOffset(.2, () -> outtake.setAngle(Outtake.Angle.OUTTAKE))
                 .waitSeconds(.5)
                 .addTemporalMarker(() -> claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH))
                 .addTemporalMarker(markersAuto.closeElevator)

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