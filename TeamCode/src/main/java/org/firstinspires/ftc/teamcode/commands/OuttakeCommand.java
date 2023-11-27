package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

public class OuttakeCommand extends SequentialCommandGroup {

    public OuttakeCommand(Outtake outtake, Outtake.Angle angle) {
        super(
                new InstantCommand(() -> outtake.setAngle(angle))
        );
    }
}
