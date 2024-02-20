package org.firstinspires.ftc.teamcode.commandbase.instantCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.commandbase.subsytem.outakeSubsystem;

public class outArmCommand extends InstantCommand {
    public outArmCommand(outakeSubsystem outake, outakeSubsystem.OutArmState state){
        super(
                () -> outake.update(state)
        );
    }
}
