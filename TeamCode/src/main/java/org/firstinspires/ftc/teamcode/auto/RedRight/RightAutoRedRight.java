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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.auto.old_with_cycles.AutoConstants;
import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.util.ClawSide;

@Config
@Autonomous(name = "Right 2+0 Auto Red Right")
public class RightAutoRedRight extends LinearOpMode {
    VelocityConstraint smallVel;
    private final RobotHardware robot = RobotHardware.getInstance();

    public static double temp = 50, temp2 = -30;
    ElapsedTime time;

    // subsystems
    SampleMecanumDrive drivetrain;
    Elevator elevator;
    Outtake outtake;
    Intake intake;
    Claw claw;
    IntakeExtension intakeExtension;
    AutoConstants autoConstants;
    TrajectorySequence placePurplePixel, intakeAnotherPreload, placePreloadsOnBoard, intakeCycle43, intakeCycle21, place43, place21, park;

    enum IntakeLevel {
        TOP_54,
        TOP_32
    }

    RightAutoRedRight.IntakeLevel intakeLevel = RightAutoRedRight.IntakeLevel.TOP_54;

    @Override
    public void runOpMode() {
        time = new ElapsedTime();
        CommandScheduler.getInstance().reset();
        drivetrain = new SampleMecanumDrive(hardwareMap);

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, telemetry);

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
                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.RIGHT))

                .lineToLinearHeading(new Pose2d(62, -32.75, Math.toRadians(0)))
                .waitSeconds(.1)
                .addTemporalMarker(() -> intake.move(Intake.Angle.INTAKE))
                .addTemporalMarker(() -> intakeExtension.openExtension())
                .waitSeconds(1)

                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.LEFT))
                .waitSeconds(.5)

                .addTemporalMarker(() -> intakeExtension.closeExtension())
                .addTemporalMarker(() -> intake.move(Intake.Angle.OUTTAKE))
                .waitSeconds(.5)
                .build();


        //  intakeAnotherPreload = drivetrain.trajectorySequenceBuilder(placePurplePixel.end())
        //    .lineToSplineHeading(new Pose2d(-40, -8.2, Math.toRadians(0)))
        //.UNSTABLE_addTemporalMarkerOffset(0.5, () -> intake.updateClawState(Intake.ClawState.INDETERMINATE, ClawSide.BOTH))
        //   .build();

        placePreloadsOnBoard = drivetrain.trajectorySequenceBuilder(placePurplePixel.end())

                // truss pose next to board
                .addTemporalMarker(() -> claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH))
                .addTemporalMarker(() -> elevator.setTarget(1050))
                .addTemporalMarker(() -> elevator.update())
                .addTemporalMarker(() -> intake.move(Intake.Angle.MID))
                .addTemporalMarker(() -> outtake.setAngle(Outtake.Angle.OUTTAKE))
                .waitSeconds(0.25)
                .addTemporalMarker(() -> claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH))
                .waitSeconds(0.15)
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

        double autoSeconds = time.seconds();
        while (opModeIsActive()) {
            drivetrain.update();

            telemetry.addData("Auto seconds: ", autoSeconds);
            telemetry.update();
        }
    }


}

