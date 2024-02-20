package org.firstinspires.ftc.teamcode.commandbase.instantCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.commandbase.subsytem.outakeSubsystem;

public class sliderCommand extends InstantCommand {
    public sliderCommand(outakeSubsystem outake, outakeSubsystem.SliderState state){
        super(
                () -> outake.update(state)
        );
    }
}
