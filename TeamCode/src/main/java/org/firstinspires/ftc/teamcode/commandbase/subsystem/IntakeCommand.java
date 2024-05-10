package org.firstinspires.ftc.teamcode.commandbase.subsystem;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commandbase.instant.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.commandbase.instant.IntakePositionCommand;
import org.firstinspires.ftc.teamcode.commandbase.instant.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.commandbase.instant.OuttakePositionCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.util.ClawSide;


public class IntakeCommand extends SequentialCommandGroup {
    public IntakeCommand(IntakeSubsystem.Angle angle) {
        super(
                new IntakeClawCommand(IntakeSubsystem.ClawState.OPEN, ClawSide.BOTH),
                new IntakePositionCommand(angle)
        );
    }
}