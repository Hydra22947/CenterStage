package org.firstinspires.ftc.teamcode.auto.rr1.autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoSettings;
import org.firstinspires.ftc.teamcode.auto.old_with_cycles.AutoConstants;
import org.firstinspires.ftc.teamcode.auto.rr1.Actions.UpdateActions;
import org.firstinspires.ftc.teamcode.auto.rr1.HydraAutoBase;
import org.firstinspires.ftc.teamcode.auto.rr1.commands.RunAction;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.ClawSide;

@Autonomous
public class BlueClose2PlusZero extends HydraAutoBase {
    private Action lineTraj, backboardTraj, parkTraj;
    UpdateActions updateActions;
    public BlueClose2PlusZero() {
        super(AutoSettings.AllianceColor.BLUE, AutoSettings.AllianceSide.FAR, AutoConstants.startPoseBlueLeft);
    }

    @Override
    public void initAuto() {
        updateActions = new UpdateActions(elevator, intake, claw, outtake, intakeExtension);

        lineTraj = robot.drive.actionBuilder(AutoConstants.startPoseBlueLeft)
                .lineToY(12)
                .build();

        backboardTraj = robot.drive.actionBuilder(new Pose2d(16, 12, Math.toRadians(-90)))
                .setTangent(0)
                //Place Preload on board
                .splineToLinearHeading(new Pose2d(51, 40, Math.toRadians(0)), Math.toRadians(0))
                .build();

        parkTraj = robot.drive.actionBuilder(new Pose2d(51, 40, Math.toRadians(0)))
                .setTangent(Math.toRadians(90))
                .lineToY(60)
                .build();

    }

    @Override
    public void startAuto() {
        schedule(
                new ParallelCommandGroup(
                        new RunAction(updateActions.updateSystems()),
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new RunAction(lineTraj),
                                        new SequentialCommandGroup(
                                                new WaitCommand(500),
                                                new InstantCommand(() -> intake.setAngle(Intake.Angle.INTAKE))
                                        )
                                ),
                                new WaitCommand(1050),
                                new InstantCommand(() -> intake.updateClawState(Intake.ClawState.OPEN, ClawSide.RIGHT)),
                                new WaitCommand(1050),
                                new ParallelCommandGroup(
                                        new InstantCommand(() ->elevator.setTarget(1000)),
                                        new InstantCommand(() -> claw.setBothClaw(Claw.ClawState.CLOSED)),
                                        new InstantCommand(() -> intake.setAngle(Intake.Angle.OUTTAKE)),
                                        new RunAction(backboardTraj)
                                ),
                                new WaitCommand(1000),
                                new InstantCommand(() -> claw.setBothClaw(Claw.ClawState.OPEN)),
                                new WaitCommand(300),
                                new RunAction(parkTraj)
                        )
                )
        );
    }
}