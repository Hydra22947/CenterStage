package org.firstinspires.ftc.teamcode.commandbase.instant;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.util.ClawSide;


public class OuttakePositionCommand extends InstantCommand {
    public OuttakePositionCommand(OuttakeSubsystem.Angle angle) {
        super(
                () -> RobotHardware.getInstance().outtake.setAngle(angle)
        );
    }
}