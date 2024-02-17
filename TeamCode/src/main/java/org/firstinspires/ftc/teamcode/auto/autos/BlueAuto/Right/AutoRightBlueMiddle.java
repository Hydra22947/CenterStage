package org.firstinspires.ftc.teamcode.auto.autos.BlueAuto.Right;

// RR-specific imports

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import static org.firstinspires.ftc.teamcode.auto.AutoSettings.writeToFile;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.auto.Actions.DepositActions;
import org.firstinspires.ftc.teamcode.auto.Actions.PlacePurpleActions;
import org.firstinspires.ftc.teamcode.auto.Actions.UpdateActions;
import org.firstinspires.ftc.teamcode.auto.AutoConstants;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.util.ClawSide;

@Config
@Autonomous(name = "2+0 - Auto Blue Right MIDDLE")
public class AutoRightBlueMiddle extends LinearOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();
    ElapsedTime time;

    // subsystems
    Elevator elevator;
    Intake intake;
    Outtake outtake;
    Claw claw;
    IntakeExtension intakeExtension;
    AutoConstants autoConstants;


    DepositActions depositActions;
    PlacePurpleActions placePurpleActions;
    UpdateActions updateActions;

    @Override
    public void runOpMode() {
        time = new ElapsedTime();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, telemetry, autoConstants.startPoseBlueRight);

        autoConstants = new AutoConstants();

        elevator = new Elevator(true);
        outtake = new Outtake();
        claw = new Claw();
        intake = new Intake();
        intakeExtension = new IntakeExtension(true);

        intakeExtension.setAuto(true);
        elevator.setAuto(true);

        depositActions = new DepositActions(elevator, intake, claw, outtake , intakeExtension);
        placePurpleActions = new PlacePurpleActions(intake, intakeExtension, claw);
        updateActions = new UpdateActions(elevator, intake, claw, outtake, intakeExtension);

        SequentialAction deposit = new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.MID),
                depositActions.readyForDeposit(),
                placePurpleActions.failSafeClaw(PlacePurpleActions.FailSafe.ACTIVATED),
                new SleepAction(0.5),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD ,600)
        );


        SequentialAction placePurplePixel = new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.INTAKE),
                placePurpleActions.openExtension(0),
                new SleepAction(.4),
                placePurpleActions.release(PlacePurpleActions.OpenClaw.LEFT_OPEN),
                new SleepAction(0.1),
                placePurpleActions.moveIntake(Intake.Angle.MID),
                placePurpleActions.lock(PlacePurpleActions.CloseClaw.LEFT_CLOSE)
        );

        SequentialAction retractDeposit = new SequentialAction(
                depositActions.retractDeposit()
        );


        SequentialAction releaseIntake = new SequentialAction(
                placePurpleActions.release(PlacePurpleActions.OpenClaw.LEFT_OPEN)
        );

        SequentialAction intakePixel = new SequentialAction(
                placePurpleActions.openExtension(720),
                placePurpleActions.moveIntake(Intake.Angle.TOP_5),
                new SleepAction(0.5),
                placePurpleActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE),
                new SleepAction(0.5),
                placePurpleActions.moveStack(),
                placePurpleActions.closeExtension()

        );
        SequentialAction readyIntake = new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.MID)
        );


        while (opModeInInit() && !isStopRequested()) {
            intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
            intake.setAngle(Intake.Angle.OUTTAKE);
            claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);
            outtake.setAngle(Outtake.Angle.INTAKE);
            telemetry.addLine("Initialized");
        }

        waitForStart();

        if (isStopRequested()) return;

        Action traj =
                robot.drive.actionBuilder(robot.drive.pose)
                .lineToY(-8)
                .stopAndAdd(placePurplePixel)
                .setTangent(0)
                .waitSeconds(.1)
                .stopAndAdd(placePurpleActions.closeExtension())
                .splineToSplineHeading(new Pose2d(-37, 9.5, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                .stopAndAdd(intakePixel)
                .waitSeconds(2)
                .stopAndAdd(readyIntake)
                .strafeToLinearHeading(new Vector2d(30, 9),Math.toRadians(0))
                .afterDisp(0.9 ,depositActions.readyForDeposit())
                .afterDisp(1 ,placePurpleActions.moveIntake(Intake.Angle.MID))
                .splineToLinearHeading(new Pose2d(51 ,34.5, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                .stopAndAdd(deposit)
                .waitSeconds(.5)
                .setTangent(Math.toRadians(90))
                .stopAndAdd(retractDeposit)
                .lineToY(10)
                        //Park
                .build();



        waitForStart();

        if (isStopRequested()) return;

        runBlocking(new ParallelAction(
                traj,
                updateActions.updateSystems()
        ));

        writeToFile(robot.drive.pose.heading.log());
    }


}