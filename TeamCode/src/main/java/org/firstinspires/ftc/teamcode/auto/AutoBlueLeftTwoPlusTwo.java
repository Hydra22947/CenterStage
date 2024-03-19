package org.firstinspires.ftc.teamcode.auto;

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
@Autonomous(name = "2+2 - Auto Blue Left")
public class AutoBlueLeftTwoPlusTwo extends LinearOpMode {
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

    public static PropLocation propLocation = PropLocation.RIGHT;

    public static int tempHeight = 1450;
    public static int minHeight = 1000;


    public static int RIGHT_EXTENSION = 270;
    public static int MIDDLE_EXTENSION = 300;
    public static int LEFT_EXTENSION = 200;

    Pose2d test;

    SequentialAction blueLeftLeft;
    SequentialAction blueLeftMiddle;
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
                intakeActions.moveIntake(Intake.Angle.TELEOP_MID),
                new SleepAction(.2),
                depositActions.moveElevator(minHeight),
                new SleepAction(0.2),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD, 0),
                depositActions.moveElevator(minHeight+300),
                new SleepAction(0.2),
                new InstantAction(()-> depositActions.retractElevator())




        );
        SequentialAction readyForDepositAction = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.MID),
                new SleepAction(0.5),
                depositActions.readyForDeposit(tempHeight)

        );

        SequentialAction depositSecondCycle = new SequentialAction(
                readyForDepositAction,


                intakeActions.moveIntake(Intake.Angle.TELEOP_MID),

                intakeActions.failSafeClaw(PlacePurpleActions.FailSafe.ACTIVATED),
                new SleepAction(1.2),
                depositActions.placeIntermediatePixel(DepositActions.Cycles.PRELOAD, 500),

                new SleepAction(0.3),
                depositActions.moveElevator(tempHeight - 100),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD, 1000),
                depositActions.moveElevator(tempHeight - 150),
                new SleepAction(0.3),
                depositActions.retractDeposit()
        );


        SequentialAction placePurplePixelAction_MIDDLE = new SequentialAction(
                new ParallelAction(
                        intakeActions.moveIntake(Intake.Angle.INTAKE),
                        intakeActions.openExtension( MIDDLE_EXTENSION)),
                new SleepAction(0.5),
                intakeActions.release(PlacePurpleActions.OpenClaw.BOTH_OPEN)
        );

        SequentialAction placePurplePixelAction_LEFT = new SequentialAction(
                new ParallelAction(
                        intakeActions.moveIntake(Intake.Angle.INTAKE),
                        intakeActions.openExtension( LEFT_EXTENSION)),
                new SleepAction(0.5),
                intakeActions.release(PlacePurpleActions.OpenClaw.BOTH_OPEN)
        );
       SequentialAction placePurplePixelAction_RIGHT = new SequentialAction(
               intakeActions.moveIntake(Intake.Angle.INTAKE),
               new SleepAction(0.6),
               new ParallelAction(
               intakeActions.openExtension( RIGHT_EXTENSION)),
               new SleepAction(0.5),
               intakeActions.moveIntakeClaw(Intake.ClawState.OPEN,ClawSide.BOTH)
       );

        SequentialAction retractPurpleAction = new SequentialAction(
                intakeActions.release(PlacePurpleActions.OpenClaw.BOTH_OPEN),
                new SleepAction(0.2),
                new ParallelAction(
                        new InstantAction(() -> intakeExtension.setAggresive(false)),
                        intakeActions.closeExtension(),
                        new SleepAction(0.2),
                        intakeActions.moveIntake(Intake.Angle.MID)
                )
        );

        SequentialAction openIntakeWhitePixelAction54 = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.TOP_54),
                new ParallelAction(
                new SleepAction(.5),
                intakeActions.lock(PlacePurpleActions.CloseClaw.SUPER_RIGHT_CLOSE),
                intakeActions.release(PlacePurpleActions.OpenClaw.LEFT_OPEN)),
                new SleepAction(0.7),
                new InstantAction(()-> intakeExtension.setAggresive(false)),
                intakeActions.openExtension(430 )

        );

        SequentialAction openIntakeWhitePixelAction32 = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.TOP_32),
                new SleepAction(.5),
                intakeActions.lock(PlacePurpleActions.CloseClaw.SUPER_RIGHT_CLOSE),
                intakeActions.release(PlacePurpleActions.OpenClaw.LEFT_OPEN),
                new SleepAction(1),
                new InstantAction(() -> intakeExtension.setAggresive(false)),
                intakeActions.openExtension(430 )

        );


        SequentialAction closeIntakeWhitePixelAction = new SequentialAction(
                intakeActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE),
                new SleepAction(.3),
                intakeActions.moveIntake(Intake.Angle.AUTO_FIX_INTAKE),
                new SleepAction(.3),
                new InstantAction(() -> intakeExtension.setAggresive(true)),
                intakeActions.closeExtension()
                //returnTransfer()
        );

       SequentialAction placePurplePixelSequence = new SequentialAction(
                depositActions.readyForDeposit(minHeight),
                new SleepAction(0.1),
                placePurplePixelAction_RIGHT,
                retractPurpleAction

        );

        SequentialAction  intake54Action = new SequentialAction(
                openIntakeWhitePixelAction54,
                new SleepAction(1.8),
                closeIntakeWhitePixelAction
        );

        SequentialAction intake32Action = new SequentialAction(
                openIntakeWhitePixelAction32,
                new SleepAction(1.5),
                returnLockTop32()
        );

        Action placePurpleTraj_RIGHT = robot.drive.actionBuilder(robot.drive.pose)
                .afterTime(0.5 ,placePurplePixelSequence)
                .strafeToLinearHeading(new Vector2d(20 ,34.25), Math.toRadians(0))
                .build();

        Action placeYellowTraj_RIGHT = robot.drive.actionBuilder(new Pose2d(20, 34.25, Math.toRadians(0)))
                .afterTime(0.9 , depositBlue)
                .strafeTo(new Vector2d(51,28))
                .build();


        Action goForIntakeTop54 = robot.drive.actionBuilder(new Pose2d(53, 28 , Math.toRadians(0)))
                .strafeTo(new Vector2d(50,30.25 ))
                .setTangent(190)
                .splineToSplineHeading(new Pose2d(10, 58, Math.toRadians(0)), Math.toRadians(180))
                .afterTime(2.3, intake54Action)
                .splineToSplineHeading(new Pose2d(-32, 58, Math.toRadians(0)),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-38.5, 43.6, Math.toRadians(12)), Math.toRadians(-180))
                .strafeTo(new Vector2d(-47.5, 44))
                .waitSeconds(0.2)
                .build();


        Action goForIntakeTop32 = robot.drive.actionBuilder(new Pose2d(50.5, 39 , Math.toRadians(0)))
                .strafeTo(new Vector2d(48,43 ))
                .setTangent(190)
                .splineToSplineHeading(new Pose2d(10, 58, Math.toRadians(0)), Math.toRadians(180))
                .afterTime(2.3, intake32Action)
                .splineToSplineHeading(new Pose2d(-32, 58, Math.toRadians(0)),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-39.5, 43.6, Math.toRadians(15)), Math.toRadians(-180))
                .strafeTo(new Vector2d(-48.5,  44.5))
                .waitSeconds(0.2)
                .build();

        Action goPlaceWhiteRIGHT_OR_TOP54 = robot.drive.actionBuilder(new Pose2d(-47.5, 44.5,Math.toRadians(15)))
                .setTangent(Math.toRadians(90))
                .afterTime(0,returnFixintake())
                .splineToSplineHeading(new Pose2d(-32, 58, Math.toRadians(0)), Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(22, 58), Math.toRadians(0))
                .afterTime(0, depositSecondCycle)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(50.5, 40), Math.toRadians(0))
                .build();

        Action goPlaceWhiteRIGHT_OR_TOP32 = robot.drive.actionBuilder(new Pose2d(-48.5, 44.8,Math.toRadians(15)))

                .setTangent(Math.toRadians(90))
                .afterTime(0,returnFixintake())
                .splineToSplineHeading(new Pose2d(-32, 58, Math.toRadians(0)), Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(22, 58), Math.toRadians(0))
                .afterTime(0, returnDepositTop32())
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(50.5, 41), Math.toRadians(0))
                .build();

        Action goPark = robot.drive.actionBuilder(new Pose2d( 47 , 41  , Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(46 , 34) ,  Math.toRadians(-90))
                .build();


        ParallelAction placePurplePixel = new ParallelAction(
                placePurpleTraj_RIGHT

        );

        ParallelAction placePreloadOnBoard = new ParallelAction(
                placeYellowTraj_RIGHT

        );

        ParallelAction intake54 = new ParallelAction(
                goForIntakeTop54

        );

        ParallelAction intake32 = new ParallelAction(
                goForIntakeTop32

        );

        ParallelAction deposit54 = new ParallelAction(
                goPlaceWhiteRIGHT_OR_TOP54

        );


        ParallelAction deposit32 = new ParallelAction(
                goPlaceWhiteRIGHT_OR_TOP32

        );




        blueLeftRight= new SequentialAction(
                placePurplePixel
                , placePreloadOnBoard
                , intake54
                , deposit54
                , intake32
                , deposit32
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
//                        blueLeftRight,
                        updateActions.updateSystems()
                ));
                break;
            case RIGHT:
                runBlocking(new ParallelAction(blueLeftRight
                        , updateActions.updateSystems()));
                break;
        }

        while (opModeIsActive()) {
            robot.drive.updatePoseEstimate();
        }

        writeToFile(robot.drive.pose.heading.log());
    }

    SequentialAction returnTransfer ()
    {
        return  new SequentialAction(
            new InstantAction(() -> intakeExtension.setAggresive(true)),
            new InstantAction(() -> intakeExtension.setTarget(0)),
            new InstantAction(() -> intake.move(Intake.Angle.OUTTAKE)),
            new InstantAction(() -> claw.setBothClaw(Claw.ClawState.OPEN)),
            new SleepAction(1),
            new InstantAction(() -> intake.updateClawState(Intake.ClawState.INDETERMINATE, ClawSide.BOTH)),
            new SleepAction(0.3),
            new InstantAction(() -> claw.setBothClaw(Claw.ClawState.CLOSED))
    );
    }


    SequentialAction returnLockTop32 ()
    {
        return  new SequentialAction(
                intakeActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE),
                new SleepAction(.3 ),
                intakeActions.moveIntake(Intake.Angle.AUTO_FIX_INTAKE),
                new SleepAction(.2 ),
                new InstantAction(() -> intakeExtension.setAggresive(true)),
                intakeActions.closeExtension()
              //  returnTransfer()

        );
    }
    SequentialAction returnDepositTop32 ()
    {
        return  new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.MID),
                depositActions.readyForDeposit(tempHeight),

                intakeActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE),
                intakeActions.moveIntake(Intake.Angle.MID),

                intakeActions.failSafeClaw(PlacePurpleActions.FailSafe.ACTIVATED),
                new SleepAction(1),
                depositActions.placeIntermediatePixel(DepositActions.Cycles.PRELOAD, 500),

                new SleepAction(0.6),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD, 1000),
                new SleepAction(0.3),
                depositActions.moveElevator(tempHeight + 800),
                depositActions.retractDeposit()
        );
    }


    SequentialAction returnFixintake () {
        return new SequentialAction(
                new SleepAction(.3),
                intakeActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.BOTH),
                new SleepAction(0.1),
                intakeActions.moveClaw(Claw.ClawState.OPEN, ClawSide.BOTH),
                new SleepAction(0.2),
                intakeActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH),
                new SleepAction(0.3),
                intakeActions.moveClaw(Claw.ClawState.OPEN, ClawSide.BOTH),
                new SleepAction(0.2),
                intakeActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH),
                new SleepAction(0.2),
                intakeActions.moveClaw(Claw.ClawState.OPEN, ClawSide.BOTH),
                new SleepAction(0.2),
                intakeActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH)

        );
    }
}