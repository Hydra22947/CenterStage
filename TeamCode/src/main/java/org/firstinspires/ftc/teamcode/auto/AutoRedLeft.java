package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
    AutoConstants autoConstants;
    TrajectorySequence placePurplePixel, placePreloadAndIntake, intakeTraj, placeSecond, park;

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
                .forward(AutoConstants.strafeForPurplePixel) // TODO: chnage to line to, and make it go a bit close to white pixel 5 ARMA
                .addTemporalMarker(0.5, () -> intake.move(Intake.Angle.INTAKE))
                .waitSeconds(AutoConstants.WAIT)
                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.RIGHT))
                .waitSeconds(.5)
                .addTemporalMarker(()-> intake.move(Intake.Angle.MID))
                .build();
        placePreloadAndIntake = drivetrain.trajectorySequenceBuilder(placePurplePixel.end())
                //Going for backdrop
                .turn(Math.toRadians(-90)) // TODO: instead of turn, line to pos eccause turn can be nigga
                .addTemporalMarker(0.5, () -> intakeExtension.openExtension())
                .waitSeconds(0.5)
                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.RIGHT))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> intake.move(Intake.Angle.TRANSFER))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> intakeExtension.closeExtension())
                .waitSeconds(1)
                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.BOTH))
                .lineToSplineHeading(autoConstants.stageDoorEndPose)
                // TODO: move markers to end of spline to placement or with distance markers
                .addTemporalMarker(() -> elevator.setTarget(Elevator.BASE_LEVEL))
                .addTemporalMarker(() -> elevator.update())
                .addTemporalMarker(() -> claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH))
                .addTemporalMarker(() -> intake.move(Intake.Angle.MID))
                .UNSTABLE_addTemporalMarkerOffset(.2, () -> outtake.setAngle(Outtake.Angle.OUTTAKE))
                .splineToLinearHeading(autoConstants.placePixelPose, Math.toRadians(0))
                .addTemporalMarker(() -> claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH))
                .waitSeconds(0.5)
                .build();

        intakeTraj = drivetrain.trajectorySequenceBuilder(placePreloadAndIntake.end())
                .addTemporalMarker(() -> elevator.setTarget(0))
                .addTemporalMarker(() -> elevator.update())
                .addTemporalMarker(() -> intake.move(Intake.Angle.INTAKE))
                .lineToSplineHeading(autoConstants.stageDoorStartPose)
                .addTemporalMarker(2, () -> intakeExtension.openExtension())
                .splineToLinearHeading(autoConstants.intakePixelVector, Math.toRadians(180))
                .waitSeconds(1)
                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.RIGHT))
                .waitSeconds(AutoConstants.WAIT)
                .build();

         placeSecond = drivetrain.trajectorySequenceBuilder(intakeTraj.end())
                 .addTemporalMarker(() -> intakeExtension.closeExtension())
                 .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.BOTH))
                .lineToSplineHeading(autoConstants.stageDoorEndPose)
                 // TODO: move markers to end of spline to placement or with distance markers
                 .addTemporalMarker(() -> elevator.setTarget(Elevator.BASE_LEVEL))
                 .addTemporalMarker(() -> elevator.update())
                 .addTemporalMarker(() -> claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH))
                 .addTemporalMarker(() -> intake.move(Intake.Angle.MID))
                 .UNSTABLE_addTemporalMarkerOffset(.2, () -> outtake.setAngle(Outtake.Angle.OUTTAKE))
                 .waitSeconds(.5)
                 .splineToLinearHeading(autoConstants.placePixelPose, Math.toRadians(0))
                 .addTemporalMarker(() -> claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH))
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
        drivetrain.followTrajectorySequence(placePreloadAndIntake);
        drivetrain.followTrajectorySequence(intakeTraj);
        drivetrain.followTrajectorySequence(placeSecond);
        drivetrain.followTrajectorySequence(intakeTraj);
        drivetrain.followTrajectorySequence(placeSecond);
        drivetrain.followTrajectorySequence(park);
        while(opModeIsActive());
    }


}