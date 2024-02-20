package org.firstinspires.ftc.teamcode.commandbase.instantCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.commandbase.subsytem.intakeSubsystem;

public class rackCommand extends InstantCommand {

        public rackCommand(intakeSubsystem intake, intakeSubsystem.RackState state){
            super(
                    () -> intake.update(state)
            );
        }
    }

