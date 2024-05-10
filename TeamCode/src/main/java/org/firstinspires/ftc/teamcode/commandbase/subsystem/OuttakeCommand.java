package org.firstinspires.ftc.teamcode.commandbase.subsystem;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commandbase.instant.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.commandbase.instant.OuttakePositionCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.util.ClawSide;


public class OuttakeCommand extends SequentialCommandGroup {
    public OuttakeCommand(OuttakeSubsystem.Angle angle) {
        super(
                new OuttakeClawCommand(OuttakeSubsystem.ClawState.CLOSED, ClawSide.BOTH),
                new OuttakePositionCommand(angle)
        );
    }
}