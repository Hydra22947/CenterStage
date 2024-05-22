package org.firstinspires.ftc.teamcode.auto.NewAuto;

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
    int tempHeight = 1250;

    public SubsystemActions(Telemetry telemetry, HardwareMap hardwareMap) {
        time = new ElapsedTime();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap, telemetry, autoConstants.startPoseBlueRight);

        autoConstants = new AutoConstants();

        elevator = new Elevator(true);
        outtake = new Outtake();
        claw = new Claw();
        intake = new Intake();
        intakeExtension = new IntakeExtension(true);

        intakeExtension.setAuto(true);
        elevator.setAuto(true);

        depositActions = new DepositActions(elevator, intake, claw, outtake, intakeExtension);
        intakeActions = new IntakeActions(intake, intakeExtension, claw);
        updateActions = new UpdateActions(elevator, intake, claw, outtake, intakeExtension);
    }

    public SequentialAction placePreloadAndIntakeAction = new SequentialAction(
    );


    SequentialAction intake5OpenAction = new SequentialAction(
            intakeActions.moveIntake(Intake.Angle.TOP_5_AUTO),
            intakeActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.BOTH)

    );

    public SequentialAction intake5CloseAction = new SequentialAction(
            intakeActions.lock(IntakeActions.CloseClaw.BOTH_CLOSE),
            new SleepAction(.5),
            intakeActions.moveStack(),

            intakeActions.moveIntake(Intake.Angle.OUTTAKE),

            new SleepAction(0.5),
            intakeActions.moveIntakeClaw(Intake.ClawState.INDETERMINATE, ClawSide.BOTH),
            new SleepAction(.5),
            intakeActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH)

    );


    public SequentialAction intake43OpenAction = new SequentialAction(


            intakeActions.moveIntake(Intake.Angle.TOP_54_AUTO),
            new InstantAction(() -> intakeExtension.setAggresive(true)),

            new ParallelAction(

                    intakeActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.BOTH),
                    intakeActions.openExtension(450)
            )
    );
    SequentialAction intake43CloseAction = new SequentialAction(
            new SleepAction(.2),
            intakeActions.lock(IntakeActions.CloseClaw.BOTH_CLOSE),
            new SleepAction(.5),
            intakeActions.closeExtension(),
            new SleepAction(.5),
            intakeActions.moveIntake(Intake.Angle.OUTTAKE),


            intakeActions.moveClaw(Claw.ClawState.OPEN, ClawSide.BOTH),
            new SleepAction(.5),
            intakeActions.moveIntakeClaw(Intake.ClawState.INDETERMINATE, ClawSide.BOTH),
            new SleepAction(.25),
            intakeActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH),
            intakeActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.BOTH)


    );
    SequentialAction intake43OpenActionRight = new SequentialAction(


            intakeActions.moveIntake(Intake.Angle.TOP_43),
            new InstantAction(() -> intakeExtension.setAggresive(true)),

            new ParallelAction(

                    intakeActions.moveIntakeClaw(Intake.ClawState.OPEN, ClawSide.BOTH),
                    intakeActions.openExtension(450)
            )
    );


    public SequentialAction placePurplePixel = new SequentialAction(
            depositActions.moveOuttake(Outtake.Angle.FLOOR),
            depositActions.placePixel()
    );


    public SequentialAction depositAction = new SequentialAction(

            depositActions.placeIntermediatePixel(DepositActions.Cycles.PRELOAD, 0),
            new SleepAction(1.5),
            depositActions.placePixel(),
            new SleepAction(.5),
            depositActions.moveElevator(tempHeight + 400),
            depositActions.retractDeposit()
    );


    SequentialAction readyForDepositAction = new SequentialAction(
            intakeActions.moveClaw(Claw.ClawState.CLOSED, ClawSide.BOTH),
            intakeActions.moveIntake(Intake.Angle.TELEOP_MID),
            new SleepAction(.75),
            depositActions.readyForDeposit(tempHeight));


    SequentialAction deposit43Action = new SequentialAction(
            depositActions.placeIntermediatePixel(DepositActions.Cycles.PRELOAD, 500),
            new SleepAction(0.6),
            depositActions.placePixel(),

            new SleepAction(0.25),
            depositActions.moveElevator(tempHeight + 300),
            depositActions.retractDeposit());


}
