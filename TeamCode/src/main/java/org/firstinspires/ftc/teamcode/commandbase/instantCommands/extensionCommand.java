package org.firstinspires.ftc.teamcode.commandbase.instantCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.commandbase.subsytem.intakeSubsystem;

public class extensionCommand extends InstantCommand {
    public extensionCommand(intakeSubsystem intake, intakeSubsystem.ExtensionState state){
        super(
                () -> intake.update(state)
        );
    }
}
