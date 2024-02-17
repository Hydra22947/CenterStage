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
import com.arcrobotics.ftclib.command.WaitCommand;
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
@Autonomous(name = "2+0 - Auto Blue Right LEFT")
public class AutoRightBlueLeft extends LinearOpMode {
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
        placePurpleActions = new PlacePurpleActions(intake, intakeExtension);
        updateActions = new UpdateActions(elevator, intake, claw, outtake, intakeExtension);

        SequentialAction deposit = new SequentialAction(
                depositActions.readyForDeposit(),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD ,600)
        );

        SequentialAction placePixel = new SequentialAction(
                new SleepAction(0.5),
                depositActions.retractDeposit()
        );

        SequentialAction intakePixel = new SequentialAction(
                placePurpleActions.openExtension(PlacePurpleActions.Length.TWO_PLUS_ONE),
                placePurpleActions.moveIntake(Intake.Angle.TOP_5),
                new SleepAction(0.5),
                placePurpleActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE),
                new SleepAction(0.5),
                placePurpleActions.moveStack(),
                placePurpleActions.closeExtension()

        );


        SequentialAction placePurplePixel = new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.INTAKE),
                new SleepAction(.1),
                placePurpleActions.openExtension(PlacePurpleActions.Length.ALMOST_HALF),
                new SleepAction(.75),
                placePurpleActions.release(PlacePurpleActions.OpenClaw.LEFT_OPEN),
                new SleepAction(.5),
                placePurpleActions.closeExtension(),
                placePurpleActions.moveIntake(Intake.Angle.MID)
        );

        SequentialAction retractDeposit = new SequentialAction(
                depositActions.retractDeposit()
        );


        SequentialAction releaseIntake = new SequentialAction(
                placePurpleActions.release(PlacePurpleActions.OpenClaw.LEFT_OPEN)
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
                        .lineToYLinearHeading(12 ,Math.toRadians(-125))
                        .stopAndAdd(placePurplePixel)
                        .setTangent(0)
                        .waitSeconds(.1)
                        .stopAndAdd(placePurpleActions.closeExtension())
                        .splineToSplineHeading(new Pose2d(-30, 10, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)
                        .stopAndAdd(intakePixel)
                        .waitSeconds(0.5)
                        .splineToLinearHeading(new Pose2d(48, 30, Math.toRadians(0)), Math.toRadians(0))
                        .afterDisp(45 ,deposit)
                        .waitSeconds(.5)
                        .setTangent(Math.toRadians(90))
                        .lineToY(60)
                        //Park
                        .setTangent(Math.toRadians(90))
                        .strafeTo(new Vector2d(48, 60))
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