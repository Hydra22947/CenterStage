package org.firstinspires.ftc.teamcode.auto.rr1;

import android.drm.DrmStore;
// RR-specific imports
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.auto.old_with_cycles.AutoConstants;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.util.ClawSide;

@Config
@Autonomous(name = "Middle 2+0 Auto Blue Right")
public class RR1AutoRightBlue extends LinearOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();
    ElapsedTime time;

    // subsystems
    MecanumDrive drivetrain;
    Elevator elevator;
    Intake intake;
    Outtake outtake;
    Claw claw;
    IntakeExtension intakeExtension;
    AutoConstants autoConstants;

    enum IntakeLevel {
        TOP_54,
        TOP_32
    }


    Action placePurplePixel, intakeAnotherPreload, placePreloadsOnBoard, intakeCycle43, intakeCycle21, place43, place21, park;


    IntakeLevel intakeLevel = IntakeLevel.TOP_54;


    @Override
    public void runOpMode() {
        time = new ElapsedTime();
        CommandScheduler.getInstance().reset();
        drivetrain = new MecanumDrive(hardwareMap, autoConstants.startPoseBlueRight);

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, telemetry, true);

        autoConstants = new AutoConstants();

        elevator = new Elevator(true);
        outtake = new Outtake();
        claw = new Claw();
        intake = new Intake();
        intakeExtension = new IntakeExtension(true);


        placePurplePixel = drivetrain.actionBuilder(AutoConstants.startPoseBlueRight)
                .lineToY(12)
                .waitSeconds(1)
                .build();
        placePreloadsOnBoard = drivetrain.actionBuilder(drivetrain.pose)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(51, 40, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(1)
                .build();
        park = drivetrain.actionBuilder(drivetrain.pose)
                .setTangent(270)
                .lineToY(60)
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
        Actions.runBlocking(
                new SequentialAction(
                        placePurplePixel,
                        placePreloadsOnBoard,
                        park
                )
        );


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