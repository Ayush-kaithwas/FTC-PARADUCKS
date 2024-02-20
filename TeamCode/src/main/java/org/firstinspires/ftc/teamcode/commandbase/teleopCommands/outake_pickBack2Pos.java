package org.firstinspires.ftc.teamcode.commandbase.teleopCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outArmCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outArmExtensionCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.sliderCommand;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.intakeSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.outakeSubsystem;

public class outake_pickBack2Pos extends SequentialCommandGroup {
    public outake_pickBack2Pos(intakeSubsystem intake, outakeSubsystem outake) {
        super(
                new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.INIT),
                new WaitCommand(300),
                new sliderCommand(outake, outakeSubsystem.SliderState.RETRACT),
                new outArmCommand(outake, outakeSubsystem.OutArmState.PICK)
        );
    }
}


//    public outake_pickBack2Pos(intakeSubsystem intake, outakeSubsystem outake) {
//        super(
////                new rackCommand(intake, intakeSubsystem.RackState.MID),
////                new WaitCommand(300),
////                new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MIDseUPAR),
////                new WaitCommand(500),
////                new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE),
////                new WaitCommand(200),
////                new ParallelCommandGroup(new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE),
////                        new rackCommand(intake, intakeSubsystem.RackState.LOWseUPAR),
//////                        new WaitCommand(300),
////                new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE)),
////                new WaitCommand(200),
////                new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK),
////                new WaitCommand(300),
//////                new rackCommand(intake, intakeSubsystem.RackState.LOW),
//////                new WaitCommand(300),
////                new gripperCommand(intake, intakeSubsystem.GripperState.INIT),
//                new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.INIT),
////                new rackCommand(intake, intakeSubsystem.RackState.LOWseUPAR),
//                new WaitCommand(300),
//                new outArmCommand(outake, outakeSubsystem.OutArmState.PICK)
//        );
//    }

