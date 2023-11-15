package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakePivot;

public class IntakeCommand extends InstantCommand {

    public IntakeCommand(Intake intake, int direction) {
        super(
                ()-> intake.setTarget(intake.getTarget() * direction)
        );
    }
}
