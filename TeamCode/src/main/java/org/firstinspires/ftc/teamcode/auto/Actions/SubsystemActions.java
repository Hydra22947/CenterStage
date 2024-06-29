package org.firstinspires.ftc.teamcode.auto.Actions;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.auto.Actions.DepositActions;
import org.firstinspires.ftc.teamcode.auto.Actions.IntakeActions;
import org.firstinspires.ftc.teamcode.auto.Actions.UpdateActions;
import org.firstinspires.ftc.teamcode.auto.AutoSettingsForAll.AutoConstants;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.util.ClawSide;

public class SubsystemActions {


    private final RobotHardware robot = RobotHardware.getInstance();
    ElapsedTime time;


    private DepositActions depositActions;
    private IntakeActions intakeActions;
    private UpdateActions updateActions;

    public SequentialAction intake5CloseAction, intake43OpenAction, transfer, depositAction, readyForDepositAction, deposit43Action;
    public ParallelAction placePreloadAndIntakeAction;
    int tempHeight = 1250;

    Action getIntake5CloseAction()
    {
        return new SequentialAction(
            intakeActions.lock(IntakeActions.CloseClaw.BOTH_CLOSE),
            new SleepAction(.5),
            intakeActions.moveStack(),

            intakeActions.moveIntake(Intake.Angle.OUTTAKE),

            new SleepAction(0.5),
            intakeActions.moveIntakeClaw(Intake.ClawState.INDETERMINATE, ClawSide.BOTH),
            new SleepAction(.5),
            intakeActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH)
    );
    }
    Action getIntake5OpenAction()
    {
        return new SequentialAction(
                intakeActions.openExtension(100),
                intakeActions.moveIntake(Intake.Angle.TOP_5_AUTO),
                intakeActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.BOTH)

        );
    }
    Action getPlacePurplePixelAction()
    {
        return new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.AUTO_MID),
                new SleepAction(.25),
                depositActions.moveOuttake(Outtake.Angle.FLOOR),
                new SleepAction(1),
                depositActions.moveClaw(Claw.ClawState.OPEN, ClawSide.RIGHT),
                new SleepAction(1),
                depositActions.moveOuttake(Outtake.Angle.INTAKE),
                intakeActions.moveIntake(Intake.Angle.MID)

        );
    }
    public SubsystemActions(DepositActions depositActions, IntakeActions intakeActions, UpdateActions updateActions) {


        this.depositActions = depositActions;
        this.intakeActions = intakeActions;
        this.updateActions = updateActions;


        intake43OpenAction = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.TOP_54_AUTO),
                new ParallelAction(

                        intakeActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.BOTH),
                        intakeActions.openExtension(450)
                )
        );


        transfer = new SequentialAction(
                intakeActions.moveIntake(Intake.Angle.OUTTAKE),
                depositActions.moveOuttake(Outtake.Angle.INTAKE),
                new SleepAction(.25),
                depositActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH),
                new SleepAction(.25),
                intakeActions.moveIntakeClaw(Intake.ClawState.INDETERMINATE, ClawSide.BOTH)
                );

        depositAction = new SequentialAction(
                depositActions.placePixel(),
                new SleepAction(.5),
                depositActions.moveElevator(tempHeight + 400),
                depositActions.retractDeposit()
        );


        readyForDepositAction = new SequentialAction(
                intakeActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH),
                intakeActions.moveIntake(Intake.Angle.TELEOP_MID),
                new SleepAction(.75),
                depositActions.readyForDeposit(tempHeight));


        deposit43Action = new SequentialAction(
                depositActions.placeIntermediatePixel(DepositActions.Cycles.PRELOAD, 500),
                new SleepAction(0.6),
                depositActions.placePixel(),
                new SleepAction(0.25),
                depositActions.moveElevator(tempHeight + 300),
                depositActions.retractDeposit());

    }

    public ParallelAction placePreloadAndIntakeAction()
    {
        return new ParallelAction(
                getPlacePurplePixelAction()

        );
    }
}
