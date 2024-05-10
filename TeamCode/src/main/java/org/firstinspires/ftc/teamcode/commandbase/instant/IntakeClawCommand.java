package org.firstinspires.ftc.teamcode.commandbase.instant;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.util.ClawSide;


public class IntakeClawCommand extends InstantCommand {
    public IntakeClawCommand(IntakeSubsystem.ClawState state, ClawSide side) {
        super(
                () -> RobotHardware.getInstance().intake.updateState(state, side)
        );
    }
}