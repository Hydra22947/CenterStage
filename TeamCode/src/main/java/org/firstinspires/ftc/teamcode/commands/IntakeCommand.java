package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeCommand extends SequentialCommandGroup {

    public IntakeCommand(Intake intake, Intake.Angle angle) {
        super(
                new InstantCommand(() -> intake.setAngle(angle))
        );
    }
}
