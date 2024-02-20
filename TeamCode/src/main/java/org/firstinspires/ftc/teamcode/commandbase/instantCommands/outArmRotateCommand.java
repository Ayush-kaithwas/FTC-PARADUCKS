package org.firstinspires.ftc.teamcode.commandbase.instantCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.commandbase.subsytem.intakeSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.outakeSubsystem;

public class outArmRotateCommand extends InstantCommand {
    public outArmRotateCommand(outakeSubsystem outake, outakeSubsystem.OutArmRotateState state){
        super(
                () -> outake.update(state)
        );
    }
}
