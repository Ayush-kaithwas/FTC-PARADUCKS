package org.firstinspires.ftc.teamcode.commandbase.instantCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.commandbase.subsytem.intakeSubsystem;

public class gripRotateCommand extends InstantCommand{
    public gripRotateCommand(intakeSubsystem intake, intakeSubsystem.GripRotateState state){
        super(
                () -> intake.update(state)
        );
    }
}
