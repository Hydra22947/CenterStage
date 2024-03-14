package org.firstinspires.ftc.teamcode.auto.MaxAuto.AutoBlueRight;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.auto.Actions.DepositActions;
import org.firstinspires.ftc.teamcode.auto.Actions.PlacePurpleActions;
import org.firstinspires.ftc.teamcode.auto.Actions.UpdateActions;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.util.ClawSide;

public class BlueRightSubsystemActions {

    DepositActions depositActions;
    PlacePurpleActions intakeActions;
    UpdateActions updateActions;

    public BlueRightSubsystemActions(Intake intake, IntakeExtension intakeExtension, Outtake outtake, Claw claw, Elevator elevator) {

        depositActions = new DepositActions(elevator, intake, claw, outtake, intakeExtension);
        intakeActions = new PlacePurpleActions(intake, intakeExtension, claw);
        updateActions = new UpdateActions(elevator, intake, claw, outtake, intakeExtension);
    }

    public static int tempHeight = 1450;

    public SequentialAction depositBlue = new SequentialAction(
            intakeActions.moveIntake(Intake.Angle.MID),
            new SleepAction(0.5),
            depositActions.placePixel(DepositActions.Cycles.PRELOAD, 0),
            new SleepAction(.2),
            depositActions.moveElevator(1300),
            new SleepAction(.2),
            intakeActions.moveIntake(Intake.Angle.TOP_54_AUTO),
            depositActions.retractDeposit()

    );
    public SequentialAction readyForDepositAction = new SequentialAction(
            intakeActions.moveIntake(Intake.Angle.MID),
            depositActions.readyForDeposit(tempHeight)

    );

    public SequentialAction depositSecondCycle = new SequentialAction(
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


    public SequentialAction retractPurpleAction = new SequentialAction(
            new SleepAction(0.3),
            intakeActions.closeExtension(),
            intakeActions.moveIntake(Intake.Angle.MID),
            intakeActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE)
    );

    public SequentialAction openIntakeWhitePixelAction = new SequentialAction(
            new SleepAction(1),
            intakeActions.moveIntake(Intake.Angle.TOP_5_AUTO),
            intakeActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.BOTH)

    );

    public SequentialAction closeIntakeWhitePixelAction = new SequentialAction(
            intakeActions.lock(PlacePurpleActions.CloseClaw.BOTH_CLOSE),
            new SleepAction(.5),
            intakeActions.moveStack(),
            new SleepAction(.5),
            intakeActions.moveIntake(Intake.Angle.OUTTAKE)
    );


    public SequentialAction transferAction = new SequentialAction(
            intakeActions.moveIntake(Intake.Angle.OUTTAKE),
            new SleepAction(0.5),
            intakeActions.moveClaw(Claw.ClawState.OPEN, ClawSide.BOTH),
            intakeActions.moveIntakeClaw(Intake.ClawState.INDETERMINATE, ClawSide.BOTH),
            new SleepAction(.75),
            intakeActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH)
    );


    public SequentialAction retractDeposit = new SequentialAction(
            depositActions.retractDeposit()
    );




    public SequentialAction intake5Action = new SequentialAction(
            openIntakeWhitePixelAction,
            new SleepAction(.5),
            closeIntakeWhitePixelAction
    );
}
