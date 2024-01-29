package org.firstinspires.ftc.teamcode.auto.RedLeft;

import static org.firstinspires.ftc.teamcode.subsystems.Intake.Angle.MID;

import android.util.Size;

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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PropPipeline;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.auto.PoseStorage;
import org.firstinspires.ftc.teamcode.auto.old_with_cycles.AutoConstants;
import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name = " Camera 2+0 Auto Red Left")
public class AutoRedLeftCamera extends LinearOpMode {
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
    PropPipeline vision;
    VisionPortal portal;
    PropPipeline.Location teamPropLocation;
    Pose2d boardPose;

    enum IntakeLevel {
        TOP_54, TOP_32
    }

    IntakeLevel intakeLevel = IntakeLevel.TOP_54;

    public void cameraSetup() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        vision = new PropPipeline(telemetry, PropPipeline.Alliance.RED);

        portal = new VisionPortal.Builder().setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")).setCameraResolution(new Size(1600, 1200)).addProcessors(vision).setStreamFormat(VisionPortal.StreamFormat.MJPEG).enableLiveView(true).setAutoStopLiveView(true).build();
    }

    //Getting the position on the board for detection
    public Pose2d getBoardPose() {
        teamPropLocation = vision.getLocation();
        switch (teamPropLocation) {
            case RIGHT:
                return PoseStorage.redRightBoard;

            case CENTER:
                return PoseStorage.redMidBoard;

            case LEFT:
                return PoseStorage.redLeftBoard;
        }
        return PoseStorage.redMidBoard;
    }

    @Override
    public void runOpMode() {
        time = new ElapsedTime();
        CommandScheduler.getInstance().reset();
        drivetrain = new SampleMecanumDrive(hardwareMap);

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, telemetry);

        autoConstants = new AutoConstants();
        drivetrain.setPoseEstimate(autoConstants.startPoseRedLeft);

        elevator = new Elevator();
        outtake = new Outtake();
        claw = new Claw();
        intake = new Intake();
        intakeExtension = new IntakeExtension();

        boardPose = getBoardPose();
        smallVel = new VelocityConstraint() {
            @Override
            public double get(double v) {
                return 50;
            }
        };


        placePurplePixel = drivetrain.trajectorySequenceBuilder(autoConstants.startPoseRedLeft)

                // place purple pixel distance
                .lineToLinearHeading(new Pose2d(-47, -30, Math.toRadians(90))).turn(Math.toRadians(90)).addTemporalMarker(() -> intake.move(Intake.Angle.INTAKE)).back(8).waitSeconds(AutoConstants.WAIT).addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.LEFT)).waitSeconds(.1).addTemporalMarker(() -> intake.move(Intake.Angle.OUTTAKE)).build();

        intakeAnotherPreload = drivetrain.trajectorySequenceBuilder(placePurplePixel.end()).lineToLinearHeading(new Pose2d(-40, -8.2, Math.toRadians(0))).UNSTABLE_addTemporalMarkerOffset(0.5, () -> intake.updateClawState(Intake.ClawState.INDETERMINATE, ClawSide.BOTH)).build();

        placePreloadsOnBoard = drivetrain.trajectorySequenceBuilder(intakeAnotherPreload.end())

                // truss pose next to board
                .lineToSplineHeading(new Pose2d(12, -9, Math.toRadians(0)))

                .addSpatialMarker(new Vector2d(8, -6), () -> claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH)).addSpatialMarker(new Vector2d(9, -6), () -> elevator.setTarget(1050)).addSpatialMarker(new Vector2d(9, -6), () -> elevator.update()).addSpatialMarker(new Vector2d(10, -6), () -> intake.move(MID)).addSpatialMarker(new Vector2d(10, -6), () -> outtake.setAngle(Outtake.Angle.OUTTAKE))

                // backdrop pose
                .splineToLinearHeading(boardPose, Math.toRadians(0))

                .waitSeconds(0.25).addTemporalMarker(() -> claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH)).waitSeconds(0.15).build();

        park = drivetrain.trajectorySequenceBuilder(placePreloadsOnBoard.end())
                .lineToLinearHeading(new Pose2d(55, -10, Math.toRadians(90)))
                .addTemporalMarker(() -> elevator.setTarget(0))
                .addTemporalMarker(() -> elevator.update())
                .addTemporalMarker(() -> outtake.setAngle(Outtake.Angle.INTAKE))
                .build();


        while (opModeInInit() && !isStopRequested()) {
            cameraSetup();
            intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
            intake.setAngle(Intake.Angle.OUTTAKE);
            intakeExtension.closeExtension();
            claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);
            outtake.setAngle(Outtake.Angle.INTAKE);
            telemetry.addLine("Initialized");
            telemetry.addData("Current prop location:", teamPropLocation.toString());
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        time.reset();

        drivetrain.followTrajectorySequence(placePurplePixel);
        drivetrain.followTrajectorySequence(intakeAnotherPreload);
        drivetrain.followTrajectorySequence(placePreloadsOnBoard);
        drivetrain.followTrajectorySequence(park);


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