package org.firstinspires.ftc.teamcode.commandbase.instantCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.commandbase.subsytem.intakeSubsystem;

public class gripperCommand extends InstantCommand {

    public gripperCommand(intakeSubsystem intake, intakeSubsystem.GripperState state){
        super(
                () -> intake.update(state)
        );
    }
}
