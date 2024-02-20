package org.firstinspires.ftc.teamcode.commandbase.teleopCommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.commandbase.instantCommands.gripRotateCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.rackCommand;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.intakeSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.outakeSubsystem;

//Parallel
public class pick101 extends ParallelCommandGroup {
    public pick101(intakeSubsystem intake, outakeSubsystem outake) {
        super(
                new rackCommand(intake, intakeSubsystem.RackState.LOW),
                new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK)
        );
    }
}
