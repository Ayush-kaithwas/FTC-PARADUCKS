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

public class pick2 extends SequentialCommandGroup {
    public pick2(intakeSubsystem intake, outakeSubsystem outake) {
        super(
                new ParallelCommandGroup(new rackCommand(intake, intakeSubsystem.RackState.LOW)),
//                new WaitCommand(300),
                new ParallelCommandGroup(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK)),
                new gripperCommand(intake, intakeSubsystem.GripperState.OPEN),
                new WaitCommand(800),
                new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE),
                new WaitCommand(500),
                new outArmCommand(outake, outakeSubsystem.OutArmState.PICK),
                new WaitCommand(500),
                new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPEN),
                new WaitCommand(800),
                new gripRotateCommand(intake,intakeSubsystem.GripRotateState.MID),
                new WaitCommand(250),
                new rackCommand(intake, intakeSubsystem.RackState.MID),
                new WaitCommand(300),
                new ParallelCommandGroup(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MIDseUPAR)),
//                new WaitCommand(400),
                new ParallelCommandGroup( new rackCommand(intake, intakeSubsystem.RackState.TOPseUPAR)),
//                new WaitCommand(250),
                new gripRotateCommand(intake, intakeSubsystem.GripRotateState.DROP),
                new WaitCommand(400),
                new gripperCommand(intake, intakeSubsystem.GripperState.OPEN),
                new WaitCommand(500),
                new ParallelCommandGroup( new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE)),
//                new WaitCommand(800),
                new ParallelCommandGroup(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MIDseJYAADA)),
//                new WaitCommand(500),
//                new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE),
////                new WaitCommand(500),
//                new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MIDseUPAR),
//                new WaitCommand(500),
                new ParallelCommandGroup(new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE)),
                new ParallelCommandGroup(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MIDseUPAR))

//                new rackCommand(intake, intakeSubsystem.RackState.TOP),

        );
    }
}
