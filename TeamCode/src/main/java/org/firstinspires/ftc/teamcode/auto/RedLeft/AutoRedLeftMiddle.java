//package org.firstinspires.ftc.teamcode.auto.RedLeft;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.profile.VelocityConstraint;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.RobotHardware;
//import org.firstinspires.ftc.teamcode.auto.old_with_cycles.AutoConstants;
//import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.subsystems.Claw;
//import org.firstinspires.ftc.teamcode.subsystems.Elevator;
//import org.firstinspires.ftc.teamcode.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
//import org.firstinspires.ftc.teamcode.subsystems.Outtake;
//import org.firstinspires.ftc.teamcode.util.ClawSide;
//
//@Config
//@Autonomous(name = "Middle 2+0 Auto Red Left")
//public class AutoRedLeftMiddle extends LinearOpMode {
//    private final RobotHardware robot = RobotHardware.getInstance();
//
//    ElapsedTime time;
//
//    // subsystems
//    SampleMecanumDrive drivetrain;
//    Elevator elevator;
//    Intake intake;
//    Outtake outtake;
//    Claw claw;
//    IntakeExtension intakeExtension;
//    AutoConstants autoConstants;
//    TrajectorySequence placePurplePixel, intakeAnotherPreload, placePreloadsOnBoard, park;
//
//    enum IntakeLevel {
//        TOP_54,
//        TOP_32
//    }
//
//    IntakeLevel intakeLevel = IntakeLevel.TOP_54;
//
//    @Override
//    public void runOpMode() {
//        time = new ElapsedTime();
//        CommandScheduler.getInstance().reset();
//        drivetrain = new SampleMecanumDrive(hardwareMap);
//
//        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
//
//        robot.init(hardwareMap, telemetry, true);
//
//        autoConstants = new AutoConstants();
//        drivetrain.setPoseEstimate(autoConstants.startPoseRedLeft);
//
//        elevator = new Elevator(true);
//        outtake = new Outtake();
//        claw = new Claw();
//        intake = new Intake();
//        intakeExtension = new IntakeExtension(true);
//
//        placePurplePixel = drivetrain.trajectorySequenceBuilder(autoConstants.startPoseRedLeft)
//
//                // place purple pixel distance
//                .lineToSplineHeading(new Pose2d(-55,-30, Math.toRadians(90)))
//                .splineToLinearHeading(new Pose2d(-36,-15, Math.toRadians(90)), Math.toRadians(180))
//                .addSpatialMarker(new Vector2d(-36, -39), () -> intake.move(Intake.Angle.INTAKE))
//                .forward(5.25)
//                .waitSeconds(AutoConstants.WAIT)
//                .addTemporalMarker(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.LEFT))
//                .waitSeconds(.1)
//                .addTemporalMarker(() -> intake.move(Intake.Angle.OUTTAKE))
//                .build();
//
//        intakeAnotherPreload = drivetrain.trajectorySequenceBuilder(placePurplePixel.end())
//                .lineToLinearHeading(new Pose2d(-40, -8.2, Math.toRadians(0)))
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> intake.updateClawState(Intake.ClawState.INDETERMINATE, ClawSide.BOTH))
//                .build();
//
//        placePreloadsOnBoard = drivetrain.trajectorySequenceBuilder(intakeAnotherPreload.end())
//
//                // truss pose next to board
//                .lineToSplineHeading(new Pose2d(12, -9, Math.toRadians(0)))
//
//                .addSpatialMarker(new Vector2d(8, -6), () -> claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH))
//                .addSpatialMarker(new Vector2d(9, -6), () -> elevator.setTarget(1050))
//                .addSpatialMarker(new Vector2d(9, -6), () -> elevator.update())
//                .addSpatialMarker(new Vector2d(10, -6), () -> intake.move(Intake.Angle.MID))
//                .addSpatialMarker(new Vector2d(10, -6), () -> outtake.setAngle(Outtake.Angle.OUTTAKE))
//
//                // backdrop pose
//                .splineToLinearHeading(new Pose2d(52.5, -32.5, Math.toRadians(0)), Math.toRadians(0))
//
//                .waitSeconds(0.25)
//                .addTemporalMarker(() -> claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH))
//                .waitSeconds(0.15)
//                .build();
//
//        park = drivetrain.trajectorySequenceBuilder(placePreloadsOnBoard.end())
//                .lineToLinearHeading(new Pose2d(55, -10, Math.toRadians(-90)))
//                .addTemporalMarker(()->elevator.setTarget(0))
//                .addTemporalMarker(()->elevator.update())
//                .addTemporalMarker(()->outtake.setAngle(Outtake.Angle.INTAKE))
//                .build();
//
//        intakeExtension.setAuto(true);
//        elevator.setAuto(true);
//
//        while (opModeInInit() && !isStopRequested()) {
//            intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
//            intake.setAngle(Intake.Angle.OUTTAKE);
//            claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);
//            outtake.setAngle(Outtake.Angle.INTAKE);
//            telemetry.addLine("Initialized");
//        }
//
//        waitForStart();
//        if (isStopRequested()) return;
//
//        time.reset();
//
//        drivetrain.followTrajectorySequence(placePurplePixel);
//        drivetrain.followTrajectorySequence(intakeAnotherPreload);
//        drivetrain.followTrajectorySequence(placePreloadsOnBoard);
//        drivetrain.followTrajectorySequence(park);
//
//        double autoSeconds = time.seconds();
//        while (opModeIsActive())
//        {
//            drivetrain.update();
//
//            telemetry.addData("Auto seconds: ", autoSeconds);
//            telemetry.update();
//        }
//    }
//
//    void moveIntakeByTraj() {
//        switch (intakeLevel) {
//            case TOP_54:
//                intake.move(Intake.Angle.TOP_54_AUTO);
//                break;
//            case TOP_32:
//                intake.move(Intake.Angle.TOP_32_AUTO);
//                break;
//        }
//    }
//
//
//}