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
@Autonomous(name = "Blue Left 2 + 0 Camera")
public class CameraTemplateBlueLeft extends LinearOpMode {

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

    }


    @Override
    public void runOpMode() throws InterruptedException {


        locationTraj = locationTraj.MIDDLE;

        trajLeft = drivetrain.trajectorySequenceBuilder(autoConstants.startPoseBlueLeft)

                .addTemporalMarker(() -> intake.move(Intake.Angle.MID))
                .addTemporalMarker(() -> elevator.setTarget(1050))
                .addTemporalMarker(() -> elevator.update())
                .addTemporalMarker(() -> outtake.setAngle(Outtake.Angle.OUTTAKE))
                .lineToLinearHeading(new Pose2d(51.5, 38, Math.toRadians(0)))

                .addTemporalMarker(() -> claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH))
                .addTemporalMarker(() -> intake.move(Intake.Angle.INTAKE))
//                .addTemporalMarker(() -> intakeExtension.openExtensionAuto()) // TODO: now we have motor new pos please
                .waitSeconds(.5)
                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.LEFT))
                .waitSeconds(.5)
//                .addTemporalMarker(() -> intakeExtension.closeExtension()) // TODO: now we have motor new pos please
                .addTemporalMarker(() -> intake.move(Intake.Angle.MID))
                .waitSeconds(.2)

                .waitSeconds(0.25)
                .waitSeconds(0.5)
                .addTemporalMarker(() -> outtake.setAngle(Outtake.Angle.INTAKE))
                .addTemporalMarker(() -> elevator.setTarget(0))
                .addTemporalMarker(() -> elevator.update())
                .addTemporalMarker(() -> outtake.setAngle(Outtake.Angle.INTAKE))
                .waitSeconds(0.15)

                .lineTo(new Vector2d(48, 39))
                .lineToLinearHeading(new Pose2d(53, 60, Math.toRadians((-90))))
                .addTemporalMarker(() -> intake.move(Intake.Angle.OUTTAKE))
                .build();

        trajMiddle = drivetrain.trajectorySequenceBuilder(autoConstants.startPoseBlueLeft)


                .addTemporalMarker(() -> intake.move(Intake.Angle.MID))
                .addTemporalMarker(() -> elevator.setTarget(1050))
                .addTemporalMarker(() -> elevator.update())
                .addTemporalMarker(() -> outtake.setAngle(Outtake.Angle.OUTTAKE))
                .lineToLinearHeading(new Pose2d(36, 25, Math.toRadians(0)))

                .addTemporalMarker(() -> intake.move(Intake.Angle.INTAKE))
//                .addTemporalMarker(() -> intakeExtension.openExtension())// TODO: now we have motor new pos please
                .waitSeconds(.5)
                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.LEFT))
                .waitSeconds(.5)
//                .addTemporalMarker(() -> intakeExtension.closeExtension()) // TODO: now we have motor new pos please
                .addTemporalMarker(() -> intake.move(Intake.Angle.MID))
                .waitSeconds(.2)

                .splineToLinearHeading(new Pose2d(52.5, 33, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.25)
                .addTemporalMarker(() -> claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> outtake.setAngle(Outtake.Angle.INTAKE))
                .addTemporalMarker(() -> elevator.setTarget(0))
                .addTemporalMarker(() -> elevator.update())
                .addTemporalMarker(() -> outtake.setAngle(Outtake.Angle.INTAKE))
                .waitSeconds(0.15)

                .lineTo(new Vector2d(48, 36))
                .lineToLinearHeading(new Pose2d(51, 58, Math.toRadians((-90))))
                .build();

        trajRight = drivetrain.trajectorySequenceBuilder(autoConstants.startPoseBlueLeft)

                .addTemporalMarker(() -> intake.move(Intake.Angle.MID))
                .addTemporalMarker(() -> elevator.setTarget(1050))
                .addTemporalMarker(() -> elevator.update())
                .addTemporalMarker(() -> outtake.setAngle(Outtake.Angle.OUTTAKE))
                .lineToLinearHeading(new Pose2d(29, 32, Math.toRadians(0)))

                .addTemporalMarker(() -> intake.move(Intake.Angle.INTAKE))
//                .addTemporalMarker(() -> intakeExtension.openExtension()) // TODO: now we have motor new pos please
                .waitSeconds(.5)
                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.LEFT))
                .waitSeconds(.5)
//                .addTemporalMarker(() -> intakeExtension.closeExtension()) // TODO: now we have motor new pos please
                .addTemporalMarker(() -> intake.move(Intake.Angle.MID))
                .waitSeconds(.2)

                .splineToLinearHeading(new Pose2d(53, 29, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.25)
                .addTemporalMarker(() -> claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> outtake.setAngle(Outtake.Angle.INTAKE))
                .addTemporalMarker(() -> elevator.setTarget(0))
                .addTemporalMarker(() -> elevator.update())
                .addTemporalMarker(() -> outtake.setAngle(Outtake.Angle.INTAKE))
                .waitSeconds(0.15)

                .lineTo(new Vector2d(48, 28))
                .lineToLinearHeading(new Pose2d(51, 58, Math.toRadians(-90)))
                .build();

        time = new ElapsedTime();
        CommandScheduler.getInstance().reset();
        drivetrain = new SampleMecanumDrive(hardwareMap);

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, telemetry, true);

        autoConstants = new AutoConstants();
        drivetrain.setPoseEstimate(autoConstants.startPoseBlueLeft);

        elevator = new Elevator(true);
        outtake = new Outtake();
        claw = new Claw();
        intake = new Intake();
        intakeExtension = new IntakeExtension(true);

        smallVel = new VelocityConstraint() {
            @Override
            public double get(double v) {
                return 50;
            }
        };


        elevator.setAuto(true);
        intakeExtension.setAuto(true);
        while (opModeInInit() && !isStopRequested()) {
            intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
            intake.setAngle(Intake.Angle.OUTTAKE);
            claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);
            outtake.setAngle(Outtake.Angle.INTAKE);
            telemetry.addLine("Initialized");
        }

        waitForStart();

        if(isStopRequested()) { return; }


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