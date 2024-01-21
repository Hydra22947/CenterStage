package org.firstinspires.ftc.teamcode.auto.right;

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
import org.firstinspires.ftc.teamcode.auto.AutoConstants;
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
@Autonomous(name = "2+4 Auto Red Right")
public class AutoRedRight extends CommandOpMode {
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
    TrajectorySequence placePurpleAndYellowPixel,intakeCycle54, intakeCycle32, place54, place32, park;

    public enum IntakeLevel {
        TOP_5,
        TOP_43,
        TOP_21
    }

    IntakeLevel intakeLevel = IntakeLevel.TOP_5;

    public void initialize() {
        time = new ElapsedTime();
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
        claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH);
        smallVel = new VelocityConstraint() {
            @Override
            public double get(double v) {
                return 50;
            }
        };


        placePurpleAndYellowPixel = drivetrain.trajectorySequenceBuilder(autoConstants.startPoseRedLeft)

                // place yellow and purple pixel distance
                .lineToLinearHeading((new Pose2d(51.6, -40, Math.toRadians(0))))
                .waitSeconds(AutoConstants.WAIT)

                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.LEFT))
                .addTemporalMarker(()-> elevator.setTarget(700))
                .addTemporalMarker(()-> elevator.update())
                .addTemporalMarker(()-> outtake.setAngle(Outtake.Angle.OUTTAKE))
                .addTemporalMarker(()-> claw.updateState(Claw.ClawState.OPEN, ClawSide.LEFT))

                .waitSeconds(.3)
                .addTemporalMarker(() -> intake.move(Intake.Angle.TOP_54))
                .waitSeconds(.2)
                .build();


        intakeCycle54 = drivetrain.trajectorySequenceBuilder(placePurpleAndYellowPixel.end())
                .addTemporalMarker(() -> outtake.setAngle(Outtake.Angle.INTAKE))
                .addTemporalMarker(() -> elevator.setTarget(0))
                .addTemporalMarker(() -> elevator.update())

                // truss pose next to wing
                .lineToSplineHeading(new Pose2d(25,-37))
                .UNSTABLE_addDisplacementMarkerOffset(autoConstants.TEMP, () -> moveIntakeByTraj())
                .splineToLinearHeading( new Pose2d(-34,-35), Math.toRadians(180))

                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.LEFT))
                .addTemporalMarker(() -> intakeExtension.openExtension())
                .waitSeconds(1)
                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.LEFT))
                .waitSeconds(0.5)
                .build();

        place54 = drivetrain.trajectorySequenceBuilder(intakeCycle54.end())
                .addTemporalMarker(() -> intake.moveStack())
                .waitSeconds(0.3)
                .addTemporalMarker(() -> intake.move(Intake.Angle.OUTTAKE))
                .addTemporalMarker(() -> intakeExtension.closeExtension())

                .addTemporalMarker(1, () -> intake.updateClawState(Intake.ClawState.INDETERMINATE, ClawSide.BOTH))

                .waitSeconds(0.3)
                .forward(80)

                .addSpatialMarker(new Vector2d(drivetrain.getPoseEstimate().getX() + 60, drivetrain.getPoseEstimate().getY()), () -> claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH))
                .addSpatialMarker(new Vector2d(drivetrain.getPoseEstimate().getX() + 60, drivetrain.getPoseEstimate().getY()), () -> elevator.setTarget(Elevator.BASE_LEVEL + 200))
                .addSpatialMarker(new Vector2d(drivetrain.getPoseEstimate().getX() + 60, drivetrain.getPoseEstimate().getY()), () -> elevator.update())
                .addSpatialMarker(new Vector2d(drivetrain.getPoseEstimate().getX() + 65, drivetrain.getPoseEstimate().getY()), () -> intake.move(Intake.Angle.MID))
                .addSpatialMarker(new Vector2d(drivetrain.getPoseEstimate().getX() + 70, drivetrain.getPoseEstimate().getY()), () -> outtake.setAngle(Outtake.Angle.OUTTAKE))

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

        intakeCycle32 = drivetrain.trajectorySequenceBuilder(place54.end())
                .addTemporalMarker(() -> outtake.setAngle(Outtake.Angle.INTAKE))
                .addTemporalMarker(() -> elevator.setTarget(0))
                .addTemporalMarker(() -> elevator.update())

                // truss pose next to wing
                .lineToSplineHeading(new Pose2d(25,-37))
                .UNSTABLE_addDisplacementMarkerOffset(autoConstants.TEMP, () -> moveIntakeByTraj())
                .splineToLinearHeading( new Pose2d(-34,-35), Math.toRadians(180))

                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.LEFT))
                .addTemporalMarker(() -> intakeExtension.openExtension())
                .waitSeconds(1)
                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.LEFT))
                .waitSeconds(0.5)
                .build();

        place32 = drivetrain.trajectorySequenceBuilder(intakeCycle32.end())
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
    }

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();
        if (isStopRequested()) return;

        time.reset();

        drivetrain.followTrajectorySequence(placePurpleAndYellowPixel);
        drivetrain.followTrajectorySequence(intakeCycle54);
        drivetrain.followTrajectorySequence(place54);
        intakeLevel = IntakeLevel.TOP_43;//TODO:ADD TOP 45
        drivetrain.followTrajectorySequence(intakeCycle32);
        drivetrain.followTrajectorySequence(place32);
        intakeLevel = IntakeLevel.TOP_21;//TODO:ADD TOP 32
        drivetrain.followTrajectorySequence(place32);
        //drivetrain.followTrajectorySequence(park);

        double autoSeconds = time.seconds();
        while (opModeIsActive())
        {
            drivetrain.update();

            telemetry.addData("Auto seconds: ", autoSeconds);
            telemetry.update();
        }
    }

    void moveIntakeByTraj() {
        //TODO:: ADD INTAKE LEVELS FOR RIGHT SIDE
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