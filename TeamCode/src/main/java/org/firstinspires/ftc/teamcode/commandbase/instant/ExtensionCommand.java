package org.firstinspires.ftc.teamcode.commandbase.instant;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.RobotHardware;
public class ExtensionCommand extends InstantCommand {
    public ExtensionCommand(int target) {
        super(
                () -> RobotHardware.getInstance().intakeExtension.setTarget(target)
        );
    }
}