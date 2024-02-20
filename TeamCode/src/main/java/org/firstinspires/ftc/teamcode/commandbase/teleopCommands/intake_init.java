package org.firstinspires.ftc.teamcode.commandbase.teleopCommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commandbase.instantCommands.gripRotateCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.gripperCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outArmCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outGripperCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.rackCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.sliderCommand;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.intakeSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.outakeSubsystem;

public class intake_init extends SequentialCommandGroup {
    public intake_init(intakeSubsystem intake, outakeSubsystem outake) {
        super(
                new ParallelCommandGroup(new rackCommand(intake, intakeSubsystem.RackState.LOW),
                        new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK)
                        ),
                new WaitCommand(300),
//                new ParallelCommandGroup(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK)),
//                new WaitCommand(500),
//                new pick101(intake,outake),
//                new gripperCommand(intake, intakeSubsystem.GripperState.OPEN),
//                new WaitCommand(1500),
                new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE),
                new WaitCommand(100),
                new ParallelCommandGroup(new sliderCommand(outake, outakeSubsystem.SliderState.MID),

//                new WaitCommand(500),
//                new outArmRotateCommand(outake, outakeSubsystem.OutArmRotateState.PICK),
//                new WaitCommand(500),
//                new WaitCommand(800),
                        new gripRotateCommand(intake,intakeSubsystem.GripRotateState.MIDseUPAR),
//                new WaitCommand(250),
                        new rackCommand(intake, intakeSubsystem.RackState.MID)),
                new WaitCommand(300),
                new ParallelCommandGroup(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MIDseJYAADA),
                        new rackCommand(intake, intakeSubsystem.RackState.TOPseUPAR)),
                new WaitCommand(800),
//                new ParallelCommandGroup(new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE),
//                        new rackCommand(intake,intakeSubsystem.RackState.LOWseUPAR),
//                        new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID)),
                new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE)
//                new sliderCommand(outake, outakeSubsystem.SliderState.LOW)


//                new Wait Command(500),
//                new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MIDseUPAR),
//                new WaitCommand(500)
//                new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE),
//                new WaitCommand(500)

//                new ParallelCommandGroup(new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE)),
//                new ParallelCommandGroup(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MIDseUPAR))

//                new rackCommand(intake, intakeSubsystem.RackState.TOP),

        );
    }
}