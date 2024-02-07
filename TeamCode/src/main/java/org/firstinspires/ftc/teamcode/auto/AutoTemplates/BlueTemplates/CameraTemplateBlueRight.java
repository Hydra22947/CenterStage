package org.firstinspires.ftc.teamcode.auto.AutoTemplates.BlueTemplates;

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
@Autonomous(name = "Blue Right 2 + 0 Camera")
public class CameraTemplateBlueRight extends LinearOpMode {

    //TODO: Add Camera
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

    TrajectorySequence trajMiddle, trajRight, trajLeft;


    enum Location {
        LEFT,
        RIGHT,
        MIDDLE,
    }

    Location locationTraj;


    public void initialize() {


    }


    void locationScore ()
    {

        switch (locationTraj)
        {
            case LEFT:
                drivetrain.followTrajectorySequence(trajLeft);
                break;

            case RIGHT:
                drivetrain.followTrajectorySequence(trajRight);
                break;

            case MIDDLE:
                drivetrain.followTrajectorySequence(trajMiddle);
                break;

            default:
                drivetrain.followTrajectorySequence(trajMiddle);

        }


        while (opModeInInit() && !isStopRequested()) {
            intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
            intake.setAngle(Intake.Angle.OUTTAKE);
            claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);
            outtake.setAngle(Outtake.Angle.INTAKE);
            telemetry.addLine("Initialized");

        }
    }



    @Override
    public void runOpMode() throws InterruptedException {

        locationTraj = locationTraj.MIDDLE;

        time = new ElapsedTime();
        CommandScheduler.getInstance().reset();
        drivetrain = new SampleMecanumDrive(hardwareMap);

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, telemetry, true);

        autoConstants = new AutoConstants();
        drivetrain.setPoseEstimate(autoConstants.startPoseBlueLeft);

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

        trajLeft = drivetrain.trajectorySequenceBuilder(autoConstants.startPoseBlueRight)

                .lineToLinearHeading(new Pose2d(-45.8, 30, Math.toRadians(180)))
                .addTemporalMarker(() -> intake.move(Intake.Angle.INTAKE))
                .back(9.2)
                .waitSeconds(AutoConstants.WAIT)
                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.LEFT))
                .waitSeconds(.4)
                .addTemporalMarker(() -> intake.move(Intake.Angle.OUTTAKE))

                .lineToSplineHeading(new Pose2d(12, 9, Math.toRadians(0)))

                .addSpatialMarker(new Vector2d(8, 6), () -> claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH))
                .addSpatialMarker(new Vector2d(9, 6), () -> elevator.setTarget(1050))
                .addSpatialMarker(new Vector2d(9, 6), () -> elevator.update())
                .addSpatialMarker(new Vector2d(10, 6), () -> intake.move(Intake.Angle.MID))
                .addSpatialMarker(new Vector2d(10, 6), () -> outtake.setAngle(Outtake.Angle.OUTTAKE))

                .lineToLinearHeading(new Pose2d(-40, 8.2, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> intake.update())

                // backdrop pose
                .splineToLinearHeading(new Pose2d(51, 40, Math.toRadians(0)), Math.toRadians(90))
                .forward(2)

                .waitSeconds(0.25)
                .addTemporalMarker(() -> claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH))
                .addTemporalMarker(() -> outtake.setAngle(Outtake.Angle.INTAKE))
                .addTemporalMarker(() -> elevator.setTarget(0))
                .addTemporalMarker(() -> elevator.update())
                .waitSeconds(0.15)

                .lineTo(new Vector2d(45,28))
                .lineToLinearHeading(new Pose2d(55, 7, Math.toRadians(-90)))
                .lineTo(new Vector2d(65,12))

                .build();

        trajMiddle = drivetrain.trajectorySequenceBuilder(autoConstants.startPoseBlueLeft)

                // place purple pixel distance
                .lineToConstantHeading(new Vector2d(-36,10))
                //.splineToLinearHeading(new Pose2d(-36,10, Math.toRadians(90)), Math.toRadians(180))
                .addSpatialMarker(new Vector2d(-36, 39), () -> intake.move(Intake.Angle.INTAKE))
                .addTemporalMarker( () -> intake.move(Intake.Angle.INTAKE))
                .waitSeconds(AutoConstants.WAIT)
                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.LEFT))
                .waitSeconds(.1)
                .addTemporalMarker(() -> intake.move(Intake.Angle.OUTTAKE))
                .lineToLinearHeading(new Pose2d(-40, 8.2, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> intake.updateClawState(Intake.ClawState.INDETERMINATE, ClawSide.BOTH))


                .lineToSplineHeading(new Pose2d(12, 9, Math.toRadians(0)))

                .addSpatialMarker(new Vector2d(8, 6), () -> claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH))
                .addSpatialMarker(new Vector2d(9, 6), () -> elevator.setTarget(1050))
                .addSpatialMarker(new Vector2d(9, 6), () -> elevator.update())
                .addSpatialMarker(new Vector2d(10, 6), () -> intake.move(Intake.Angle.MID))
                .addSpatialMarker(new Vector2d(10, 6), () -> outtake.setAngle(Outtake.Angle.OUTTAKE))

                // backdrop pose
                .splineToLinearHeading(new Pose2d(53.8, 31.5, Math.toRadians(0)), Math.toRadians(0))

                .waitSeconds(0.25)
                .addTemporalMarker(() -> claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH))
                .waitSeconds(0.15)
                .addTemporalMarker(()->outtake.setAngle(Outtake.Angle.INTAKE))
                .addTemporalMarker(()->elevator.setTarget(0))
                .addTemporalMarker(()->elevator.update())

                .lineTo(new Vector2d(45,28))
                .lineToLinearHeading(new Pose2d(57, 7, Math.toRadians(-90)))
                .lineTo(new Vector2d(65,12))

                .build();

        trajRight = drivetrain.trajectorySequenceBuilder(autoConstants.startPoseBlueLeft)

                // place purple pixel distance
                .lineToLinearHeading(new Pose2d(-37, 10, Math.toRadians(-70)))
                .addTemporalMarker( () -> intake.move(Intake.Angle.INTAKE))
                .waitSeconds(AutoConstants.WAIT)
                .lineTo(new Vector2d(-40,20))
                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.LEFT))
                .waitSeconds(.1)
                .addTemporalMarker(()->intakeExtension.setTarget(0))
                .addTemporalMarker(() -> intake.move(Intake.Angle.OUTTAKE))

                .lineToLinearHeading(new Pose2d(-40, 8.2, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> intake.updateClawState(Intake.ClawState.INDETERMINATE, ClawSide.BOTH))
                .lineToSplineHeading(new Pose2d(12, 9, Math.toRadians(0)))

                .addSpatialMarker(new Vector2d(8, 6), () -> claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH))
                .addSpatialMarker(new Vector2d(9, 6), () -> elevator.setTarget(1050))
                .addSpatialMarker(new Vector2d(9, 6), () -> elevator.update())

                .addSpatialMarker(new Vector2d(10, 6), () -> intake.move(Intake.Angle.MID))
                .addSpatialMarker(new Vector2d(10, 6), () -> outtake.setAngle(Outtake.Angle.OUTTAKE))

                // backdrop pose
                .splineToLinearHeading(new Pose2d(53.8, 29, Math.toRadians(0)), Math.toRadians(0))

                .waitSeconds(0.25)
                .addTemporalMarker(() -> claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH))
                .waitSeconds(0.15)
                .addTemporalMarker(()->outtake.setAngle(Outtake.Angle.INTAKE))
                .addTemporalMarker(()->elevator.setTarget(0))
                .addTemporalMarker(()->elevator.update())

                .lineTo(new Vector2d(45,28))
                .lineToLinearHeading(new Pose2d(57, 7, Math.toRadians(-90)))
                .lineTo(new Vector2d(65,12))

                .build();


        intakeExtension.setAuto(true);
        elevator.setAuto(true);

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

            telemetry.addData("Auto seconds: ", autoSeconds);
            telemetry.update();

            locationScore();


        }


    }
}