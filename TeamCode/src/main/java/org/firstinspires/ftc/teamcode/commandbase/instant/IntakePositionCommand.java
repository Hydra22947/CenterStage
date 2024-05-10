package org.firstinspires.ftc.teamcode.commandbase.instant;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;


public class IntakePositionCommand extends InstantCommand {
    public IntakePositionCommand(IntakeSubsystem.Angle angle) {
        super(
                () -> RobotHardware.getInstance().intake.setAngle(angle)
        );
    }
}