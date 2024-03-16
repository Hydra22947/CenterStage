package org.firstinspires.ftc.teamcode.auto.BlueLeft.DetectLeft;

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
@Autonomous(name = "2+0 - Auto BlueLeftLeft Preload")
public class BlueLeftLeftPreload extends Auto {

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

    public static int tempHeight = 1450;
    public static int minHeight = 950;
    public static int LEFT_EXTENSION = 200;

    public static double delayBackDrop = 1;
    SequentialAction blueLeftLeft;


    public BlueLeftLeftPreload(Telemetry telemetry, HardwareMap hardwareMap, Intake intake, IntakeExtension intakeExtensiom, Outtake outtake, Claw claw, Elevator elevator) {
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

        SequentialAction placePurplePixelAction_LEFT = new SequentialAction(
                new ParallelAction(
                        intakeActions.moveIntake(Intake.Angle.INTAKE),
                        new SleepAction(0.1),
                        intakeActions.openExtension(LEFT_EXTENSION)),
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
                placePurplePixelAction_LEFT,
                retractPurpleAction

        );

        Action placePurpleTraj_LEFT = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(30, 34.25), Math.toRadians(0))
                .waitSeconds(.5)
                .build();

        Action placeYellowTraj_LEFT = robot.drive.actionBuilder(new Pose2d(30, 34.25, Math.toRadians(0)))
                .strafeTo(new Vector2d(52, 38))
                .waitSeconds(.5)
                .build();


        Action goPark = robot.drive.actionBuilder(new Pose2d(52, 38, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(46, 28), Math.toRadians(-90))
                .build();

        ParallelAction placePurplePixel = new ParallelAction(
                placePurpleTraj_LEFT,
                placePurplePixelSequence
        );

        ParallelAction placePreloadOnBoard = new ParallelAction(
                // .waitSeconds(delayBackDrop)
                placeYellowTraj_LEFT,
                depositBlue
        );


        blueLeftLeft = new SequentialAction(
                placePurplePixel
                , placePreloadOnBoard
                , goPark
        );


    }
    @Override
    public void run ()
    {
        runBlocking(new ParallelAction(
                blueLeftLeft,
                updateActions.updateSystems()
        ));


    }

}