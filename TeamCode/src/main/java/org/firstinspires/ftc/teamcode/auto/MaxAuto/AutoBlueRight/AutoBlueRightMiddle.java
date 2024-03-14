package org.firstinspires.ftc.teamcode.auto.MaxAuto.AutoBlueRight;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
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
import org.firstinspires.ftc.teamcode.auto.Actions.PlacePurpleActions;
import org.firstinspires.ftc.teamcode.auto.AutoSettingsForAll.AutoConstants;
import org.firstinspires.ftc.teamcode.auto.Actions.UpdateActions;
import org.firstinspires.ftc.teamcode.auto.MaxAuto.Auto;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.util.ClawSide;

public class AutoBlueRightMiddle {

    private final RobotHardware robot = RobotHardware.getInstance();
    ElapsedTime time;


    private AutoConstants autoConstants;

    UpdateActions updateActions;

    BlueRightSubsystemActions subsystemActions;
    DepositActions depositActions;
    PlacePurpleActions intakeActions;
    public SequentialAction blueLeftMiddle;

    public AutoBlueRightMiddle(Telemetry telemetry, HardwareMap hardwareMap, Intake intake, IntakeExtension intakeExtension, Outtake outtake, Claw claw, Elevator elevator) {
        time = new ElapsedTime();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, telemetry, autoConstants.startPoseBlueLeft);

        autoConstants = new AutoConstants();


        updateActions = new UpdateActions(elevator, intake, claw, outtake, intakeExtension);
        subsystemActions = new BlueRightSubsystemActions(intake, intakeExtension, outtake, claw, elevator);
        depositActions = new DepositActions(elevator, intake, claw, outtake, intakeExtension);
        intakeActions = new PlacePurpleActions(intake, intakeExtension, claw);
        int tempHeight = 1450;

        SequentialAction depositBlue = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.MID),
                new SleepAction(0.5),
                depositActions.placePixel(DepositActions.Cycles.PRELOAD, 0),
                new SleepAction(.2),
                depositActions.moveElevator(1300),
                new SleepAction(.2),
                intakeActions.moveIntake(Intake.Angle.TOP_54_AUTO),
                depositActions.retractDeposit()

        );
        SequentialAction readyForDepositAction = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.MID),
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


        SequentialAction retractPurpleAction = new SequentialAction(
                new SleepAction(0.3),
                intakeActions.closeExtension(),
                intakeActions.moveIntake(Intake.Angle.MID),
                intakeActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE)
        );

        SequentialAction openIntakeWhitePixelAction = new SequentialAction(
                new SleepAction(1),
                intakeActions.moveIntake(Intake.Angle.TOP_5_AUTO),
                intakeActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.BOTH)

        );

        SequentialAction closeIntakeWhitePixelAction = new SequentialAction(
                intakeActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE),
                new SleepAction(.5),
                intakeActions.moveStack(),
                new SleepAction(.5),
                intakeActions.moveIntake(Intake.Angle.OUTTAKE)
        );


        SequentialAction transferAction = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.OUTTAKE),
                new SleepAction(0.5),
                intakeActions.moveClaw(Claw.ClawState.OPEN, ClawSide.BOTH),
                intakeActions.moveIntakeClaw(Intake.ClawState.INDETERMINATE, ClawSide.BOTH),
                new SleepAction(.75),
                intakeActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH)
        );


        SequentialAction retractDeposit = new SequentialAction(
                depositActions.retractDeposit()
        );


        SequentialAction intake5Action = new SequentialAction(
                openIntakeWhitePixelAction,
                new SleepAction(.5),
                closeIntakeWhitePixelAction
        );

        //Trajectories

        Action placePurpleTraj = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(-34.5, 34), Math.toRadians(-90))
                .build();

        Action intake5Traj = robot.drive.actionBuilder(new Pose2d(-34.5, 34, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(-49.5, 24), Math.toRadians(0))
                .waitSeconds(.1)
                .strafeToLinearHeading(new Vector2d(-53.5, 26), Math.toRadians(0))
                .waitSeconds(.1)
                .build();

        Action depositPreloadTraj = robot.drive.actionBuilder(new Pose2d(-53.5, 26, Math.toRadians(0)))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-44.25, 10), Math.toRadians(0))


                //deposit
                .strafeToLinearHeading(new Vector2d(30, 8), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(52, 32, Math.toRadians(0)), Math.toRadians(0)).setTangent(0)

                .build();
        Action parkTraj = robot.drive.actionBuilder(new Pose2d(52, 32, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(46, 32), Math.toRadians(-90))
                .build();

        ParallelAction placePurplePixel = new ParallelAction(
                placePurpleTraj
        );


        ParallelAction intake54 = new ParallelAction(
                intake5Traj,
                intake5Action
        );

        ParallelAction depositPreload = new ParallelAction(
                depositPreloadTraj,
                depositSecondCycle
        );

        ParallelAction park = new ParallelAction(
                parkTraj,
                retractDeposit
        );

        blueLeftMiddle = new SequentialAction(
                placePurplePixel,
                intake54,
                depositPreload,
                park
        );
    }

    public Action run() {
        return new ParallelAction(
                blueLeftMiddle,
                updateActions.updateSystems()
        );
    }

}
