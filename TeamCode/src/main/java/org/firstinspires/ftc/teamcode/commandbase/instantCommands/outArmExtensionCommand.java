package org.firstinspires.ftc.teamcode.commandbase.instantCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.commandbase.subsytem.outakeSubsystem;

public class outArmExtensionCommand extends InstantCommand {
    public outArmExtensionCommand(outakeSubsystem outake, outakeSubsystem.OutArmExtensionState state){
        super(
                () -> outake.update(state)
        );
    }
}
