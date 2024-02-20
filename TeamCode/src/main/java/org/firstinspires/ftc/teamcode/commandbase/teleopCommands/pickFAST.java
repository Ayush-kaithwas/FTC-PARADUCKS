package org.firstinspires.ftc.teamcode.commandbase.teleopCommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commandbase.instantCommands.gripRotateCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.gripperCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outArmCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outGripperCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.rackCommand;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.intakeSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.outakeSubsystem;

public class pickFAST extends SequentialCommandGroup {
    public pickFAST(intakeSubsystem intake, outakeSubsystem outake) {
        super(
                new ParallelCommandGroup(new rackCommand(intake, intakeSubsystem.RackState.LOW),
                        new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK)),
                new WaitCommand(300),
                new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE),
                new WaitCommand(100),
                new ParallelCommandGroup(
                        new outArmCommand(outake, outakeSubsystem.OutArmState.PICK),
                        new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPEN),
                        new gripRotateCommand(intake,intakeSubsystem.GripRotateState.MID),
                        new rackCommand(intake, intakeSubsystem.RackState.TOPseUPAR)),
                new WaitCommand(500),
                new gripRotateCommand(intake, intakeSubsystem.GripRotateState.DROP),
                new WaitCommand(300),
                new gripperCommand(intake, intakeSubsystem.GripperState.OPENUP),
                new WaitCommand(500),
                new ParallelCommandGroup(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE),
                        new gripperCommand(intake, intakeSubsystem.GripperState.OPENUP_BACK))



        );
    }
}
