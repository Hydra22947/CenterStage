package org.firstinspires.ftc.teamcode.auto.rr1;

// RR-specific imports

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.auto.old_with_cycles.AutoConstants;
import org.firstinspires.ftc.teamcode.auto.rr1.Actions.DepositActions;
import org.firstinspires.ftc.teamcode.auto.rr1.Actions.PlacePurpleActions;
import org.firstinspires.ftc.teamcode.auto.rr1.Actions.UpdateActions;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.util.ClawSide;

@Config
@Autonomous(name = "Auto Blue Left")
public class AutoRightBlue extends LinearOpMode {
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

        robot.init(hardwareMap, telemetry, autoConstants.startPoseBlueLeft);

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
                depositActions.placePixel(DepositActions.Cycles.PRELOAD ,600),
                depositActions.retractDeposit()
        );
        SequentialAction readyForDeposit = new SequentialAction(
                depositActions.readyForDeposit()
        );
        SequentialAction placePurplePixel = new SequentialAction(
                placePurpleActions.retract(),
                placePurpleActions.openExtension(PlacePurpleActions.Length.HALF),
                placePurpleActions.release(PlacePurpleActions.OpenClaw.LEFT_OPEN)


        );

        SequentialAction retarctExtension = new SequentialAction(
                placePurpleActions.retract(),
                placePurpleActions.lock(PlacePurpleActions.CloseClaw.LEFT_CLOSE),
                placePurpleActions.openExtension(PlacePurpleActions.Length.EXTENSION_CLOSED)



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
                        .stopAndAdd(readyForDeposit)
                        .splineToLinearHeading(new Pose2d(40 , 24, Math.toRadians(0)), Math.toRadians(0))
                        .stopAndAdd(placePurplePixel)
                        .stopAndAdd(deposit)
                        .waitSeconds(1)
                        .stopAndAdd(retarctExtension)
                        .setTangent(0)
                        //Place Preload on board
                        .splineToLinearHeading(new Pose2d(52, 37, Math.toRadians(0)), Math.toRadians(0))
                        .waitSeconds(.1)
                        .stopAndAdd(deposit)
                        .waitSeconds(0.5)
                        //Park
                        .lineToX(40)
                        .setTangent(Math.toRadians(90))
                        .lineToY(60)
                        .strafeTo(new Vector2d(52, 59))
                        .build();

        waitForStart();

        if (isStopRequested()) return;

        runBlocking(new ParallelAction(
                traj,
                updateActions.updateSystems()
        ));

    }


}