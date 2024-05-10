package org.firstinspires.ftc.teamcode.commandbase.instant;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.ClawSide;


public class IntakeEyeCommand extends InstantCommand {
    public IntakeEyeCommand(double distance) {
        super(
                () -> RobotHardware.getInstance().intake.setSeeFarFrom(distance)
        );
    }
}