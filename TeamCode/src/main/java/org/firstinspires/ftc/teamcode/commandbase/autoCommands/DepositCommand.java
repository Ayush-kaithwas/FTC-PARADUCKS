package org.firstinspires.ftc.teamcode.commandbase.autoCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outArmCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outArmExtensionCommand;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.intakeSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.outakeSubsystem;

public class DepositCommand extends SequentialCommandGroup {
    public DepositCommand(intakeSubsystem intake, outakeSubsystem outake) {
        super(
                new outArmCommand(outake, outakeSubsystem.OutArmState.DROP),
                new WaitCommand(1000),
                new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND),
//                new WaitCommand(1000)
                new WaitCommand(1250)

        );
    }
}
