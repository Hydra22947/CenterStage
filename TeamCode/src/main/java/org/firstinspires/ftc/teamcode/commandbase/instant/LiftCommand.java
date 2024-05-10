package org.firstinspires.ftc.teamcode.commandbase.instant;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class LiftCommand extends InstantCommand {
    public LiftCommand(int target) {
        super(
                () -> RobotHardware.getInstance().elevator.setTarget(target)
        );
    }
}