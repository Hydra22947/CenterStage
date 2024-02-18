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

import javax.xml.transform.sax.SAXSource;

@Config
@Autonomous(name = "2+0 - Auto Blue Right RIGHT")
public class AutoRightBlueRight extends LinearOpMode {
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

        depositActions = new DepositActions(elevator, intake, claw, outtake, intakeExtension);
        placePurpleActions = new PlacePurpleActions(intake, intakeExtension, claw);
        updateActions = new UpdateActions(elevator, intake, claw, outtake, intakeExtension);

        SequentialAction deposit = new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.MID),
                depositActions.readyForDeposit(1300),
                placePurpleActions.failSafeClaw(PlacePurpleActions.FailSafe.ACTIVATED),
                new SleepAction(0.75),
                placePurpleActions.moveClaw(Claw.ClawState.INTERMEDIATE, ClawSide.BOTH),
                new SleepAction(0.75),
                placePurpleActions.moveClaw(Claw.ClawState.OPEN, ClawSide.BOTH)
        );
        SequentialAction transfer = new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.OUTTAKE),
                new SleepAction(0.5),
                placePurpleActions.moveClaw(Claw.ClawState.OPEN, ClawSide.RIGHT),
                placePurpleActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.BOTH),
                new SleepAction(.5),
                placePurpleActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH)
        );
        SequentialAction placePurplePixelClose = new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.INTAKE),
                new SleepAction(1),
                placePurpleActions.release(PlacePurpleActions.OpenClaw.RIGHT_OPEN),
                new SleepAction(1),
                placePurpleActions.moveIntake(Intake.Angle.MID),
                placePurpleActions.lock(PlacePurpleActions.CloseClaw.RIGHT_CLOSE)
        );
        SequentialAction placePurplePixel = new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.INTAKE),
                placePurpleActions.openExtension(1640),
                new SleepAction(1.45),
                placePurpleActions.release(PlacePurpleActions.OpenClaw.RIGHT_OPEN),
                new SleepAction(0.1),
                placePurpleActions.moveIntake(Intake.Angle.MID),
                placePurpleActions.lock(PlacePurpleActions.CloseClaw.RIGHT_CLOSE)
        );


        SequentialAction retractDeposit = new SequentialAction(
                depositActions.retractDeposit()
        );


        SequentialAction releaseIntake = new SequentialAction(
                placePurpleActions.release(PlacePurpleActions.OpenClaw.LEFT_OPEN)
        );

        SequentialAction intakePixel = new SequentialAction(
                placePurpleActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.BOTH),
                placePurpleActions.moveIntake(Intake.Angle.TOP_5_AUTO),
                new SleepAction(1),
                placePurpleActions.openExtension(700),
                new SleepAction(1),
                placePurpleActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE),
                new SleepAction(0.5),
                placePurpleActions.moveStack(),
                placePurpleActions.closeExtension()

        );
        SequentialAction readyIntake = new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.INTAKE),
                new SleepAction(1)
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

                        .strafeToLinearHeading(new Vector2d(-32, 28), Math.toRadians(0))
                        .waitSeconds(.2)
                        .lineToX(-48)
                        .lineToX(-32)
                        .stopAndAdd(readyIntake)
                        .stopAndAdd(placePurplePixelClose)
                        .strafeToLinearHeading(new Vector2d(-40,10.9), Math.toRadians(0))
                        .waitSeconds(.5)
                        .stopAndAdd(intakePixel)
                        .waitSeconds(2)
                        .stopAndAdd(transfer)

                        .stopAndAdd(readyIntake)
                        .strafeToLinearHeading(new Vector2d(30, 9), Math.toRadians(0))
                        .stopAndAdd(placePurpleActions.moveIntake(Intake.Angle.MID))
                        .stopAndAdd(depositActions.readyForDeposit(1300))
                        .splineToLinearHeading(new Pose2d(51.3, 27.5, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                        .stopAndAdd(deposit)
                        .waitSeconds(.5)
                        .stopAndAdd(retractDeposit)
                        .setTangent(Math.toRadians(90))

                        //Park - Close to other board
                        //.lineToY(10)

                        //Park - Corner
                        //.lineToY(64)
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