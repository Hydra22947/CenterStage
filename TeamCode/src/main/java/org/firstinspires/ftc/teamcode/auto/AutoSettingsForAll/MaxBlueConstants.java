package org.firstinspires.ftc.teamcode.auto.AutoSettingsForAll;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.auto.Actions.DepositActions;
import org.firstinspires.ftc.teamcode.auto.Actions.IntakeActions;
import org.firstinspires.ftc.teamcode.auto.Actions.UpdateActions;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.util.ClawSide;

public class MaxBlueConstants {

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
    IntakeActions intakeActions;
    UpdateActions updateActions;



    public static int tempHeight = 1450;
    public static int minHeight = 950;


    public static int RIGHT_EXTENSION = 270;
    public static int MIDDLE_EXTENSION = 300;
    public static int LEFT_EXTENSION = 200;

    Pose2d test;

    SequentialAction blueLeftLeft;
    SequentialAction blueLeftMiddle;
    SequentialAction blueLeftRight;


    public Action placePurpleTraj_RIGHT;
    public Action placePurpleTraj_MIDDLE;
    public Action placePurpleTraj_LEFT;

    public Action placePurplePixelSequence_RIGHT;
    public Action placePurplePixelSequence_MIDDLE;
    public Action placePurplePixelSequence_LEFT;

    public Action placeYellowTraj_RIGHT;
    public Action placeYellowTraj_MIDDLE;
    public Action placeYellowTraj_LEFT;

    public Action goForIntakeTopMIDDLE;
    public Action goForIntakeTopRIGHT;
    public Action goForIntakeTopLEFT;

    public Action goPark;

    public SequentialAction openIntakeWhitePixelAction54;
    public SequentialAction openIntakeWhitePixelAction32;

    public ParallelAction intake32;
    public ParallelAction intake54_LEFT;
    public ParallelAction intake54_MIDDLE;
    public ParallelAction intake54_RIGHT;

    public ParallelAction placePurplePixel_LEFT;
    public ParallelAction placePurplePixel_MIDDLE;
    public ParallelAction placePurplePixel_RIGHT;

    public ParallelAction placeYellowPixel_LEFT;
    public ParallelAction placeYellowPixel_RIGHT;
    public ParallelAction placeYellowPixel_MIDDLE;

    public ParallelAction goPlaceCyclesRight;
    public ParallelAction goPlaceCyclesLeftOrMid;
    public ParallelAction goParkAction;

    Telemetry telemetry;
    HardwareMap hW;
    public MaxBlueConstants (Elevator elevator, Intake intake, Outtake outtake, Claw claw, IntakeExtension intakeExtension , HardwareMap hW , Telemetry telemetry)
    {

        this.hW = hW;
        this.telemetry = telemetry;

        robot.init(hW, telemetry, autoConstants.startPoseBlueLeft);

       this.elevator = elevator;
       this.outtake = outtake;
       this.claw = claw;
       this.intake = intake;
       this.intakeExtension = intakeExtension;

       depositActions = new DepositActions(elevator , intake , claw , outtake , intakeExtension);
       intakeActions = new IntakeActions(intake, intakeExtension, claw);
       updateActions = new UpdateActions(elevator , intake , claw , outtake , intakeExtension);

        openIntakeWhitePixelAction54 = new SequentialAction(
                new SleepAction(4),
                intakeActions.moveIntake(Intake.Angle.TOP_54),
                new SleepAction(.5),
                intakeActions.release(IntakeActions.OpenClaw.BOTH_OPEN),
                new SleepAction(1.5),
                intakeActions.openExtension(750 )

        );

         openIntakeWhitePixelAction32 = new SequentialAction(
                new SleepAction(4),
                intakeActions.moveIntake(Intake.Angle.TOP_32),
                new SleepAction(.5),
                intakeActions.release(IntakeActions.OpenClaw.BOTH_OPEN),
                new SleepAction(1.5),
                intakeActions.openExtension(750 )

        );

        // --------------------------------------------------------------------------------------//

        placePurpleTraj_RIGHT = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(20 ,34.25), Math.toRadians(0))
                .waitSeconds(.5)
                .build();

        placePurpleTraj_MIDDLE = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(30 ,24), Math.toRadians(0))
                .waitSeconds(.5)
                .build();

        placePurpleTraj_LEFT = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(48 ,34.25), Math.toRadians(0))
                .waitSeconds(.5)
                .build();

        // --------------------------------------------------------------------------------------//

         placeYellowTraj_RIGHT = robot.drive.actionBuilder(new Pose2d(20, 31.25, Math.toRadians(0)))
                .strafeTo(new Vector2d(51, 31.25))
                .waitSeconds(.5)
                .build();

         placeYellowTraj_MIDDLE = robot.drive.actionBuilder(new Pose2d(30, 24, Math.toRadians(0)))
                .strafeTo(new Vector2d(50, 34.25))
                .waitSeconds(.5)
                .build();

         placeYellowTraj_LEFT = robot.drive.actionBuilder(new Pose2d(48, 34.25, Math.toRadians(0)))
                .strafeTo(new Vector2d(50, 37.5))
                .waitSeconds(.5)
                .build();

        // --------------------------------------------------------------------------------------//


         goForIntakeTopMIDDLE = robot.drive.actionBuilder(new Pose2d(50, 34.25 , Math.toRadians(0)))
                .setTangent(190)
                .splineToSplineHeading(new Pose2d(10, 58, Math.toRadians(0)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-32, 58, Math.toRadians(0)),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-41.8, 43.8, Math.toRadians(5)), Math.toRadians(-180))
                .waitSeconds(0.7)
                .build();


        goForIntakeTopRIGHT = robot.drive.actionBuilder(new Pose2d(20, 31.25 , Math.toRadians(0)))
                .setTangent(190)
                .splineToSplineHeading(new Pose2d(10, 58, Math.toRadians(0)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-32, 58, Math.toRadians(0)),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-41.8, 38, Math.toRadians(0)), Math.toRadians(-180))
                .waitSeconds(0.7)
                .build();


         goForIntakeTopLEFT = robot.drive.actionBuilder(new Pose2d(48, 34.25 , Math.toRadians(0)))
                .setTangent(190)
                .splineToSplineHeading(new Pose2d(10, 58, Math.toRadians(0)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-32, 58, Math.toRadians(0)),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-41.8, 38, Math.toRadians(0)), Math.toRadians(-180))
                .waitSeconds(0.7)
                .build();


         goPark = robot.drive.actionBuilder(new Pose2d( 48 , 34.25  , Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(46 , 34) ,  Math.toRadians(-90))
                .build();

        // --------------------------------------------------------------------------------------//

        Action goForIntakeTop_RIGHT = robot.drive.actionBuilder(new Pose2d(20, 31.25, Math.toRadians(0)))
                .setTangent(190)
                .splineToSplineHeading(new Pose2d(10, 58, Math.toRadians(0)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-32, 58, Math.toRadians(0)),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-41.8, 43.8, Math.toRadians(5)), Math.toRadians(-180))
                .waitSeconds(0.7)
                .build();

        Action goForIntakeTop_MID_OR_LEFT = robot.drive.actionBuilder(new Pose2d(48, 34.25 , Math.toRadians(0)))
                .setTangent(190)
                .splineToSplineHeading(new Pose2d(10, 58, Math.toRadians(0)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-32, 58, Math.toRadians(0)),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-41.8, 43.8, Math.toRadians(5)), Math.toRadians(-180))
                .waitSeconds(0.7)
                .build();

        // --------------------------------------------------------------------------------------//

        SequentialAction openIntakeWhitePixelAction54 = new SequentialAction(
                new SleepAction(4),
                intakeActions.moveIntake(Intake.Angle.TOP_54),
                new SleepAction(.5),
                intakeActions.release(IntakeActions.OpenClaw.BOTH_OPEN),
                new SleepAction(1.5),
                intakeActions.openExtension(750 )

        );

        SequentialAction openIntakeWhitePixelAction32 = new SequentialAction(
                new SleepAction(4),
                intakeActions.moveIntake(Intake.Angle.TOP_32),
                new SleepAction(.5),
                intakeActions.release(IntakeActions.OpenClaw.BOTH_OPEN),
                new SleepAction(1.5),
                intakeActions.openExtension(750 )

        );

        SequentialAction closeIntakeWhitePixelAction = new SequentialAction(
                intakeActions.lock(IntakeActions.CloseClaw.BOTH_CLOSE),
                new SleepAction(.5),
                intakeActions.moveStack(),
                new SleepAction(.5),
                intakeActions.moveIntake(Intake.Angle.OUTTAKE),
                intakeActions.closeExtension()
        );

        SequentialAction retractPurpleAction = new SequentialAction(
                intakeActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.BOTH),
                new SleepAction(0.5),
                intakeActions.closeExtension(),
                intakeActions.moveIntake(Intake.Angle.MID)
        );

        SequentialAction transferAction = new SequentialAction(
                new InstantAction(() -> intakeExtension.setAggresive(true)),
                new ParallelAction(intakeActions.openExtension(0), intakeActions.moveIntake(Intake.Angle.OUTTAKE)),
                new SleepAction(0.5),
                intakeActions.moveClaw(Claw.ClawState.OPEN, ClawSide.BOTH),
                intakeActions.moveIntakeClaw(Intake.ClawState.INDETERMINATE, ClawSide.BOTH),
                new SleepAction(.75),
                intakeActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH)
        );

        SequentialAction intake54Action = new SequentialAction(
                openIntakeWhitePixelAction54,
                new SleepAction(0.4),
                closeIntakeWhitePixelAction,
                new ParallelAction(
                        new SleepAction(0.3),
                        transferAction
                )

        );

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

        SequentialAction intake32Action = new SequentialAction(
                openIntakeWhitePixelAction32,
                new SleepAction(0.4),
                closeIntakeWhitePixelAction,
                new ParallelAction(
                        new SleepAction(0.3),
                        transferAction
                )

        );

        SequentialAction readyForDepositAction = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.MID),
                new SleepAction(0.5),
                depositActions.readyForDeposit(tempHeight)

        );

        SequentialAction depositCyclesSequence = new SequentialAction(
                new SleepAction(2),
                readyForDepositAction,


                intakeActions.moveIntake(Intake.Angle.MID),

                intakeActions.failSafeClaw(IntakeActions.FailSafe.ACTIVATED),
                new SleepAction(1),
                depositActions.placeIntermediatePixel(DepositActions.Cycles.PRELOAD, 500),

                new SleepAction(0.1),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD, 1000),

                new SleepAction(0.4),
                depositActions.moveElevator(tempHeight),
                depositActions.retractDeposit()
        );


         placePurplePixelSequence_RIGHT = new SequentialAction(
                depositActions.readyForDeposit(minHeight),
                new SleepAction(1.3),
                placePurpleTraj_RIGHT,
                retractPurpleAction

        );

         placePurplePixelSequence_MIDDLE = new SequentialAction(
                depositActions.readyForDeposit(minHeight),
                new SleepAction(1.3),
                placePurpleTraj_MIDDLE,
                retractPurpleAction

        );

        placePurplePixelSequence_LEFT = new SequentialAction(
                depositActions.readyForDeposit(minHeight),
                new SleepAction(1.3),
                placePurpleTraj_LEFT,
                retractPurpleAction

        );

         intake32 = new ParallelAction(
                goForIntakeTopLEFT,
                intake32Action
        );

         intake54_LEFT = new ParallelAction(
                goForIntakeTopLEFT,
                intake54Action
        );

        intake54_RIGHT = new ParallelAction(
                goForIntakeTopRIGHT,
                intake54Action
        );

        intake54_MIDDLE = new ParallelAction(
                goForIntakeTopMIDDLE,
                intake54Action
        );


         placePurplePixel_RIGHT =  new ParallelAction(
                 placePurpleTraj_RIGHT,
                 placePurplePixelSequence_RIGHT
         );
         placePurplePixel_MIDDLE = new ParallelAction(
                 placePurpleTraj_MIDDLE,
                 placePurplePixelSequence_RIGHT
         );
         placePurplePixel_LEFT =  new ParallelAction(
                placePurpleTraj_LEFT,
                placePurplePixelSequence_LEFT
         );


        placeYellowPixel_RIGHT =  new ParallelAction(
                placeYellowTraj_RIGHT,
                depositBlue
        );
        placeYellowPixel_MIDDLE = new ParallelAction(
                placeYellowTraj_MIDDLE,
                depositBlue
        );
        placeYellowPixel_LEFT =  new ParallelAction(
                placeYellowTraj_LEFT,
                depositBlue
        );

        goPlaceCyclesRight = new ParallelAction(
                goForIntakeTop_RIGHT,
                depositCyclesSequence

        );
        goPlaceCyclesLeftOrMid = new ParallelAction(
                goForIntakeTop_MID_OR_LEFT,
                depositCyclesSequence

        );

        goParkAction = new ParallelAction();

    }
}
