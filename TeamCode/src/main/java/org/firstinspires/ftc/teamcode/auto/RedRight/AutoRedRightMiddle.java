package org.firstinspires.ftc.teamcode.auto.RedRight;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
@Autonomous(name = "Middle 2+0 Auto Red Right")
public class AutoRedRightMiddle extends CommandOpMode {
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

    enum IntakeLevel {
        TOP_54,
        TOP_32
    }

    IntakeLevel intakeLevel = IntakeLevel.TOP_54;

    public void initialize() {
        time = new ElapsedTime();
        CommandScheduler.getInstance().reset();
        drivetrain = new SampleMecanumDrive(hardwareMap);

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, telemetry, true);

        autoConstants = new AutoConstants();
        drivetrain.setPoseEstimate(autoConstants.startPoseRedRight);

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


        placePurplePixel = drivetrain.trajectorySequenceBuilder(autoConstants.startPoseRedRight)
                .lineToLinearHeading(new Pose2d(36, -22, Math.toRadians(0)))


                .addTemporalMarker(() -> intake.move(Intake.Angle.INTAKE))
                .addTemporalMarker(() -> intakeExtension.openExtension())
                .waitSeconds(.5)
                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.LEFT))
                .waitSeconds(.5)
                .addTemporalMarker(() -> intakeExtension.closeExtension())
                .addTemporalMarker(() -> intake.move(Intake.Angle.MID))
                .waitSeconds(.2)
                .build();


        placePreloadsOnBoard = drivetrain.trajectorySequenceBuilder(placePurplePixel.end())
                .addTemporalMarker(() -> elevator.setTarget(1050))
                .addTemporalMarker(() -> elevator.update())
                .addTemporalMarker(() -> outtake.setAngle(Outtake.Angle.OUTTAKE))

                // backdrop pose
                .splineToLinearHeading(new Pose2d(52.5, -28.8, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.25)
                .addTemporalMarker(() -> claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH))
                .waitSeconds(0.15)
                .build();


        park = drivetrain.trajectorySequenceBuilder(placePreloadsOnBoard.end())
                .addTemporalMarker(() -> elevator.setTarget(0))
                .addTemporalMarker(() -> elevator.update())
                .addTemporalMarker(() -> outtake.setAngle(Outtake.Angle.INTAKE))
                .lineToLinearHeading(new Pose2d(55, -60, Math.toRadians(autoConstants.startPoseRedRight.getHeading())))
                .addTemporalMarker(() -> intake.move(Intake.Angle.OUTTAKE))
                .build();

        while (opModeInInit() && !isStopRequested()) {
            intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
            intake.setAngle(Intake.Angle.OUTTAKE);
            intakeExtension.closeExtension();
            claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH);
            outtake.setAngle(Outtake.Angle.INTAKE);
            telemetry.addLine("Initialized");
        }
    }

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();
        if (isStopRequested()) return;

        time.reset();

        drivetrain.followTrajectorySequence(placePurplePixel);
        drivetrain.followTrajectorySequence(placePreloadsOnBoard);
        drivetrain.followTrajectorySequence(park);
       /* intakeLevel = IntakeLevel.TOP_54;
        drivetrain.followTrajectorySequence(intakeCycle43);
        drivetrain.followTrajectorySequence(place43);
        intakeLevel = IntakeLevel.TOP_32;
        drivetrain.followTrajectorySequence(intakeCycle21);
        drivetrain.followTrajectorySequence(place21);
        *///drivetrain.followTrajectorySequence(park);

        double autoSeconds = time.seconds();
        while (opModeIsActive()) {
            drivetrain.update();

            telemetry.addData("Auto seconds: ", autoSeconds);
            telemetry.update();
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


}