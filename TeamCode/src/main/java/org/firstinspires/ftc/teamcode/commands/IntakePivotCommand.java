package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakePivot;
import org.firstinspires.ftc.teamcode.util.values.ClawSide;

public class IntakePivotCommand extends InstantCommand {

    public IntakePivotCommand(IntakePivot intake, IntakePivot.Angle angle, IntakePivot.Type type) {
        super(
                ()-> intake.updateState(angle, type)
        );
    }
}
