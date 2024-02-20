package org.firstinspires.ftc.teamcode.commandbase.instantCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.commandbase.subsytem.outakeSubsystem;

public class outGripperCommand extends InstantCommand {
    public outGripperCommand(outakeSubsystem outake, outakeSubsystem.OutGripperState state){
        super(
                () -> outake.update(state)
        );
    }
}
