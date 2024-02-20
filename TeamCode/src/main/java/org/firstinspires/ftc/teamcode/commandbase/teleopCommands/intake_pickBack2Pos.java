package org.firstinspires.ftc.teamcode.commandbase.teleopCommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commandbase.instantCommands.gripRotateCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.gripperCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.rackCommand;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.intakeSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.outakeSubsystem;

public class intake_pickBack2Pos extends SequentialCommandGroup {
    public intake_pickBack2Pos(intakeSubsystem intake, outakeSubsystem outake) {
        super(
                new ParallelCommandGroup(
                        new gripperCommand(intake, intakeSubsystem.GripperState.OPENUP),
                        new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID),
                        new rackCommand(intake, intakeSubsystem.RackState.LOWseUPAR),
                        new rackCommand(intake, intakeSubsystem.RackState.LOW),
                        new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE),
                        new gripperCommand(intake, intakeSubsystem.GripperState.INIT))
                );
}}
