package org.firstinspires.ftc.teamcode.auto.BlueLeft.DetectMid;

// RR-specific imports

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;
import static org.firstinspires.ftc.teamcode.auto.AutoSettingsForAll.AutoSettings.writeToFile;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
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
import org.firstinspires.ftc.teamcode.auto.AutoSettingsForAll.AutoConstants;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.util.ClawSide;

@Config
@Autonomous(name = "2+2 - Auto BlueLeftMid + Cycle")
public class BlueLeftMidPreloadAndCycle extends LinearOpMode {

    //TODO: ONLY ON PRELOAD RIGHT AND MIDDLE ADD APRILTAG FAILSAFE , FOR ELIOR
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
    PlacePurpleActions intakeActions;
    UpdateActions updateActions;


    enum PropLocation
    {
        LEFT,
        CENTER,
        RIGHT
    }

    public static PropLocation propLocation = PropLocation.CENTER;

    public static int tempHeight = 1450;
    public static int minHeight = 950;



    public static int MIDDLE_EXTENSION = 300;

    public static double delayBackDrop = 1;
    public static double delayCycle = 1;
    SequentialAction blueLeftRight;

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

        depositActions = new DepositActions(elevator, intake, claw, outtake, intakeExtension);
        intakeActions = new PlacePurpleActions(intake, intakeExtension, claw);
        updateActions = new UpdateActions(elevator, intake, claw, outtake, intakeExtension);


        SequentialAction depositBlue = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.MID),
                new SleepAction(.2),
                depositActions.moveElevator(minHeight),
                new SleepAction(1),
                intakeActions.moveIntake(Intake.Angle.TOP_54),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD, 0),
                new SleepAction(0.25),
                depositActions.retractDeposit()

        );
        SequentialAction readyForDepositAction = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.MID),
                new SleepAction(0.5),
                depositActions.readyForDeposit(tempHeight)

        );

        SequentialAction depositSecondCycle = new SequentialAction(
                new SleepAction(3),

                readyForDepositAction,
                intakeActions.moveIntake(Intake.Angle.MID),
                intakeActions.failSafeClaw(PlacePurpleActions.FailSafe.ACTIVATED),

                new SleepAction(1),
                depositActions.placeIntermediatePixel(DepositActions.Cycles.PRELOAD, 500),

                new SleepAction(0.1),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD, 1000),

                new SleepAction(0.4),

                depositActions.moveElevator(tempHeight),
                depositActions.retractDeposit()
        );

        SequentialAction placePurplePixelAction_RIGHT = new SequentialAction(
                new ParallelAction(
                        intakeActions.moveIntake(Intake.Angle.INTAKE),
                        intakeActions.openExtension( MIDDLE_EXTENSION)
                ),
                new SleepAction(0.5),
                intakeActions.release(PlacePurpleActions.OpenClaw.BOTH_OPEN)
        );

        SequentialAction retractPurpleAction = new SequentialAction(
                intakeActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.BOTH),
                new SleepAction(0.5),
                intakeActions.closeExtension(),
                intakeActions.moveIntake(Intake.Angle.MID)
        );

        SequentialAction openIntakeWhitePixelAction54 = new SequentialAction(
                new SleepAction(3.5),
                intakeActions.moveIntake(Intake.Angle.TOP_54),
                new SleepAction(.5),
                intakeActions.release(PlacePurpleActions.OpenClaw.BOTH_OPEN),
                new SleepAction(1),
                intakeActions.openExtension(750 )

        );

        SequentialAction closeIntakeWhitePixelAction = new SequentialAction(
                intakeActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE),
                new SleepAction(.5),
                intakeActions.moveStack(),
                new SleepAction(.5),
                new ParallelAction(
                intakeActions.moveIntake(Intake.Angle.OUTTAKE),
                intakeActions.closeExtension()
                )
        );


        SequentialAction transferAction = new SequentialAction(
                new InstantAction(() -> intakeExtension.setAggresive(true)),
                new ParallelAction(
                        intakeActions.openExtension(0),
                        intakeActions.moveIntake(Intake.Angle.OUTTAKE),
                        new SleepAction(0.5),
                        intakeActions.moveClaw(Claw.ClawState.OPEN, ClawSide.BOTH),
                        intakeActions.moveIntakeClaw(Intake.ClawState.INDETERMINATE, ClawSide.BOTH),
                         new SleepAction(.75),
                        intakeActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH)
                )
        );

        SequentialAction placePurplePixelSequence = new SequentialAction(
                depositActions.readyForDeposit(minHeight),
                new SleepAction(1.3),
                placePurplePixelAction_RIGHT,
                retractPurpleAction

        );

        SequentialAction intake54Action = new SequentialAction(
                openIntakeWhitePixelAction54,
                new SleepAction(0.4),
                new ParallelAction(
                        closeIntakeWhitePixelAction,
                        new SleepAction(0.3),
                        transferAction
                )
        );

        Action placePurpleTraj_MID = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(30 ,34.25), Math.toRadians(0))
                .waitSeconds(.5)
                .build();

        Action placeYellowTraj_MID = robot.drive.actionBuilder(new Pose2d(30, 34.25, Math.toRadians(0)))
                // .waitSeconds(delayBackDrop)
                .strafeTo(new Vector2d(52,34.25))
                .waitSeconds(.5)
                .build();

        Action goForIntakeTop54 = robot.drive.actionBuilder(new Pose2d(50, 34.25 , Math.toRadians(0)))
                // .waitSeconds(delayCycle)
                .setTangent(190)
                .splineToSplineHeading(new Pose2d(10, 58, Math.toRadians(0)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-32, 58, Math.toRadians(0)),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-41.8, 38, Math.toRadians(10)), Math.toRadians(-180))
                .strafeTo(new Vector2d(-41.8 , 42.5))

                .waitSeconds(0.7)
                .build();

        Action goPlaceWhiteMID = robot.drive.actionBuilder(new Pose2d(-41.8, 42.5 ,Math.toRadians(10)))
                .splineToLinearHeading(new Pose2d(-32, 58, Math.toRadians(0)), Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(22, 58, Math.toRadians(0)), Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(48.2, 38, Math.toRadians(0)) , Math.toRadians(-90))
                .strafeTo(new Vector2d(49 , 38))
                .build();

        Action goPark = robot.drive.actionBuilder(new Pose2d( 48 , 34.25  , Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(46 , 34) ,  Math.toRadians(-90))
                .build();

        ParallelAction placePurplePixel = new ParallelAction(
                placePurpleTraj_MID,
                placePurplePixelSequence
        );

        ParallelAction placePreloadOnBoard = new ParallelAction(
                placeYellowTraj_MID,
                depositBlue
        );

        ParallelAction intake54 = new ParallelAction(
                goForIntakeTop54,
                intake54Action

        );

        ParallelAction deposit54 = new ParallelAction(
                goPlaceWhiteMID,
                depositSecondCycle
        );


        blueLeftRight = new SequentialAction(
                placePurplePixel
                , placePreloadOnBoard
                , intake54
                , deposit54
                , goPark
        );

        while (opModeInInit() && !isStopRequested()) {
            intake.setAngle(Intake.Angle.MID);

            intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
            claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);
            outtake.setAngle(Outtake.Angle.INTAKE);
            telemetry.addLine("Initialized");
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        switch (propLocation)
        {
            case LEFT:
                runBlocking(new ParallelAction(
                        //trajBlueLeft,
                        updateActions.updateSystems()
                ));
                break;
            case CENTER:

                runBlocking(new ParallelAction(
                        blueLeftRight,
                        updateActions.updateSystems()
                ));
                break;
            case RIGHT:
                runBlocking(new ParallelAction(
                        //trajBlueRight,
                        updateActions.updateSystems()
                ));
                break;
        }

        while (opModeIsActive()) {
            robot.drive.updatePoseEstimate();
        }

        writeToFile(robot.drive.pose.heading.log());
    }


}