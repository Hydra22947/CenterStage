package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;


public class IntakeExtensionCommand extends InstantCommand {
    public IntakeExtensionCommand(IntakeExtension extension, IntakeExtension.ExtensionState state) {
        super(
                ()-> extension.updateState(state)
        );
    }
}
