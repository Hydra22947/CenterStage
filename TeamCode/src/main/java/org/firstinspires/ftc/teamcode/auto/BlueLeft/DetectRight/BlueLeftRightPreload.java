package org.firstinspires.ftc.teamcode.auto.BlueLeft.DetectRight;

// RR-specific imports

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;
import static org.firstinspires.ftc.teamcode.auto.AutoSettingsForAll.AutoSettings.writeToFile;

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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.auto.Actions.DepositActions;
import org.firstinspires.ftc.teamcode.auto.Actions.PlacePurpleActions;
import org.firstinspires.ftc.teamcode.auto.Actions.UpdateActions;
import org.firstinspires.ftc.teamcode.auto.AutoSettingsForAll.AutoConstants;
import org.firstinspires.ftc.teamcode.auto.MaxAuto.Auto;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.util.ClawSide;

@Config
@Autonomous(name = "2+0 - Auto BlueLeftRight Preload")
public class BlueLeftRightPreload extends Auto {

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


    public static int RIGHT_EXTENSION = 270;

    public static double delayBackDrop = 1;
    SequentialAction blueLeftRight;


    public BlueLeftRightPreload (Telemetry telemetry, HardwareMap hardwareMap, Intake intake, IntakeExtension intakeExtensiom, Outtake outtake, Claw claw, Elevator elevator) {
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

        SequentialAction placePurplePixelAction_RIGHT = new SequentialAction(
                new ParallelAction(
                        intakeActions.moveIntake(Intake.Angle.INTAKE),
                        new SleepAction(0.1),
                        intakeActions.openExtension( RIGHT_EXTENSION)),
                new SleepAction(0.3),
                intakeActions.release(PlacePurpleActions.OpenClaw.BOTH_OPEN)
        );

        SequentialAction retractPurpleAction = new SequentialAction(
                intakeActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.BOTH),
                new ParallelAction(
                        new SleepAction(0.5),
                        intakeActions.closeExtension(),
                        intakeActions.moveIntake(Intake.Angle.MID)
                )
        );



        SequentialAction placePurplePixelSequence = new SequentialAction(
                depositActions.readyForDeposit(minHeight),
                new SleepAction(1.3),
                placePurplePixelAction_RIGHT,
                retractPurpleAction

        );

        Action placePurpleTraj_RIGHT = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(20 ,34.25), Math.toRadians(0))
                .waitSeconds(.5)
                .build();

        Action placeYellowTraj_RIGHT = robot.drive.actionBuilder(new Pose2d(20, 31.25, Math.toRadians(0)))
                .strafeTo(new Vector2d(52,31.25))
                .waitSeconds(.5)
                .build();


        Action goPark = robot.drive.actionBuilder(new Pose2d( 48 , 34.25  , Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(46 , 34) ,  Math.toRadians(-90))
                .build();

        ParallelAction placePurplePixel = new ParallelAction(
                placePurpleTraj_RIGHT,
                placePurplePixelSequence
        );

        ParallelAction placePreloadOnBoard = new ParallelAction(
                // .waitSeconds(delayBackDrop)
                placeYellowTraj_RIGHT,
                depositBlue
        );


        blueLeftRight = new SequentialAction(
                placePurplePixel
                , placePreloadOnBoard
                , goPark
        );


    }

    @Override
    public void run() {

        runBlocking(new ParallelAction(
                blueLeftRight,
                updateActions.updateSystems()
        ));
    }
}