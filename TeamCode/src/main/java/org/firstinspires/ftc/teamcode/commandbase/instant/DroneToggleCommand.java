package org.firstinspires.ftc.teamcode.commandbase.instant;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class DroneToggleCommand extends InstantCommand {
    public DroneToggleCommand() {
        super(
                () -> RobotHardware.getInstance().drone.toggle()
        );
    }
}