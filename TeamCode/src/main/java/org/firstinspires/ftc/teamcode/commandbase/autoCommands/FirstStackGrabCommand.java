package org.firstinspires.ftc.teamcode.commandbase.autoCommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commandbase.instantCommands.gripRotateCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.gripperCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.rackCommand;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.intakeSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.outakeSubsystem;

public class FirstStackGrabCommand extends SequentialCommandGroup {
    public FirstStackGrabCommand(intakeSubsystem intake, outakeSubsystem outake) {
        super(
                new ParallelCommandGroup(new rackCommand(intake, intakeSubsystem.RackState.MID),
                new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID)),
                new WaitCommand(300),
                new gripperCommand(intake, intakeSubsystem.GripperState.OPEN),
                new WaitCommand(800),
                new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE)
        );
    }
}
