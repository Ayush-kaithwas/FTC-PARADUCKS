package org.firstinspires.ftc.teamcode.commandbase.teleopCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outArmCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outGripperCommand;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.outakeSubsystem;

public class drop extends SequentialCommandGroup {
    public drop(outakeSubsystem outake) {
        super(
                new outArmCommand(outake, outakeSubsystem.OutArmState.DROP),
//                new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND),
                new WaitCommand(1500),
                new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPENRIGHT),
                new WaitCommand(800),
                new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPENLEFT)
//                new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE)

        );
    }

}
