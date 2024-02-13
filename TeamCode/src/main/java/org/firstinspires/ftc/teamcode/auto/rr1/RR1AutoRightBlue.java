package org.firstinspires.ftc.teamcode.auto.rr1;

import android.drm.DrmStore;
// RR-specific imports
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.auto.old_with_cycles.AutoConstants;
import org.firstinspires.ftc.teamcode.auto.rr1.Actions.DepositActions;
import org.firstinspires.ftc.teamcode.auto.rr1.Actions.PlacePurpleActions;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.util.ClawSide;

@Config
@Autonomous(name = "2+0 Auto Blue Left")
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


    Action placePurplePixel, placePreloadsOnBoard, park, preloadMidAuto;


    IntakeLevel intakeLevel = IntakeLevel.TOP_54;

    DepositActions depositActions;
    PlacePurpleActions placePurpleActions;

    @Override
    public void runOpMode() {
        time = new ElapsedTime();

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, telemetry, autoConstants.startPoseBlueLeft);

        autoConstants = new AutoConstants();

        elevator = new Elevator(true);
        outtake = new Outtake();
        claw = new Claw();
        intake = new Intake();
        intakeExtension = new IntakeExtension(true);

        depositActions = new DepositActions(elevator, intake, claw, outtake);
        placePurpleActions = new PlacePurpleActions(intake, intakeExtension);

        SequentialAction deposit = new SequentialAction(
                depositActions.readyForDeposit(),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD),
                depositActions.retractDeposit()
        );
        SequentialAction placePurplePixel = new SequentialAction(
                placePurpleActions.placePurpleMid(),
                placePurpleActions.retract()
        );

        preloadMidAuto = drivetrain.actionBuilder(drivetrain.pose)
                //place purple pixel
                .lineToY(12)
                .waitSeconds(1)

                //Place Preload on board
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(51, 40, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(1)

                //Park
                .setTangent(Math.toRadians(90))
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
                new ParallelAction(
                        preloadMidAuto,
                        placePurplePixel
                )
        );
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