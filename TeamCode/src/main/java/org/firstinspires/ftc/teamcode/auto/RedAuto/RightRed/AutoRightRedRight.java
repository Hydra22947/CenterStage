package org.firstinspires.ftc.teamcode.auto.RedAuto.RightRed;

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
@Autonomous(name = "2+0 - Auto Right Red Right")
public class AutoRightRedRight extends LinearOpMode {
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

        robot.init(hardwareMap, telemetry, autoConstants.startPoseRedRight);

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
                depositActions.readyForDeposit(950),
                new SleepAction(1),
                placePurpleActions.moveClaw(Claw.ClawState.OPEN, ClawSide.BOTH),
                new SleepAction(1),
                depositActions.retractDeposit()
        );

        SequentialAction placePurplePixel = new SequentialAction(
                placePurpleActions.moveIntake(Intake.Angle.INTAKE),
                new SleepAction(.5),
                placePurpleActions.openExtension(640),
                new SleepAction(1),
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
                        .stopAndAdd(depositActions.readyForDeposit(950))
                        .splineToLinearHeading(new Pose2d(48, -31.25, Math.toRadians(0)), Math.toRadians(0))
                        .stopAndAdd(placePurplePixel)
                        .setTangent(0)
                        .stopAndAdd(placePurpleActions.closeExtension())
                        .waitSeconds(.1)
                        .strafeTo(new Vector2d(50.5, -42.5))
                        .stopAndAdd(deposit)
                        .waitSeconds(0.5)
                        //Park
                        .setTangent(Math.toRadians(90))
                        .strafeTo(new Vector2d(45, -60))
                        .stopAndAdd(retractDeposit)
                        .build();

        waitForStart();

        if (isStopRequested()) return;

        runBlocking(new ParallelAction(
                traj,
                updateActions.updateSystems()
        ));

        while (opModeIsActive())
        {
            robot.drive.updatePoseEstimate();
        }

        writeToFile(robot.drive.pose.heading.log());
    }


}