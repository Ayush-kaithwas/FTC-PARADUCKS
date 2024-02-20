package org.firstinspires.ftc.teamcode.commandbase.teleopCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commandbase.instantCommands.gripRotateCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.gripperCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outArmCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outArmExtensionCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outGripperCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.rackCommand;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.intakeSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.outakeSubsystem;

//PICKnPLACE without slider
public class pickandout extends SequentialCommandGroup {
    public pickandout(intakeSubsystem intake, outakeSubsystem outake) {
        super(

                new rackCommand(intake, intakeSubsystem.RackState.LOW),
                new WaitCommand(300),
                new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK),
                new WaitCommand(500),
                new gripperCommand(intake, intakeSubsystem.GripperState.OPEN),
                new WaitCommand(800),
                new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE),
                new WaitCommand(500),
                new gripRotateCommand(intake,intakeSubsystem.GripRotateState.MID),
                new WaitCommand(250),
                new rackCommand(intake, intakeSubsystem.RackState.TOP),
                new WaitCommand(250),
                new gripRotateCommand(intake, intakeSubsystem.GripRotateState.DROP),
                new WaitCommand(250),
                new rackCommand(intake, intakeSubsystem.RackState.TOPseUPAR),
                new WaitCommand(250),
                new gripperCommand(intake, intakeSubsystem.GripperState.OPEN),
                new WaitCommand(250),
                new outArmCommand(outake, outakeSubsystem.OutArmState.PICK),
                new WaitCommand(1000),
//                new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.INIT),//OPTIONAL
                new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPEN),
                new WaitCommand(2000),
                new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE),
                new WaitCommand(500).andThen(
                        new outArmCommand(outake, outakeSubsystem.OutArmState.DROP)
                ).andThen(
//                new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND),
                        new WaitCommand(800)).andThen(
                        new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPENRIGHT)).andThen(
                        new WaitCommand(800)).andThen(
                        new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPENLEFT))
                );
    }
}
