package org.firstinspires.ftc.teamcode.commandbase.teleopCommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outArmCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outArmExtensionCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outGripperCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.sliderCommand;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.outakeSubsystem;

public class out1 extends SequentialCommandGroup {
    public out1(outakeSubsystem outake) {
        super(
                new ParallelCommandGroup(
                        new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE),
                        new outArmCommand(outake, outakeSubsystem.OutArmState.MidDROP)),//DROP// changed
                new WaitCommand(100),//700//changed
                new outArmCommand(outake, outakeSubsystem.OutArmState.LessthanDROP),//new
                new WaitCommand(100),//new
                new outArmCommand(outake, outakeSubsystem.OutArmState.DROP),//new
                new WaitCommand(100),//new
                new outArmCommand(outake, outakeSubsystem.OutArmState.SlightmoreDROP),
                new WaitCommand(100),//new
                new sliderCommand(outake, outakeSubsystem.SliderState.LOW)

//                new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.MidEXTEND) // test

        );
    }
}
//    public out(outakeSubsystem outake) {
//        super(
////                new outArmCommand(outake, outakeSubsystem.OutArmState.PICK),
////                new WaitCommand(400),
//////                new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.INIT),//OPTIONAL
////                new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPEN),
////                new WaitCommand(2000),
////                new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE),
////                new WaitCommand(1000),
////                new WaitCommand(500).andThen(
////                new outArmRotateCommand(outake, outakeSubsystem.OutArmRotateState.INIT),
////                new WaitCommand(400),
//                new outArmCommand(outake, outakeSubsystem.OutArmState.DROP),
//                new WaitCommand(1000),
//                new ParallelCommandGroup(new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND),
//                        new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE))
////                new WaitCommand(1000)
//
////                .andThen(
////                new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPENRIGHT)).andThen(
////                new WaitCommand(800)).andThen(
////                new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPENLEFT)).andThen(
////                new WaitCommand(500))
////                new outArmCommand(outake, outakeSubsystem.OutArmState.INIT)
////                new WaitCommand(1000),
////                new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE)
//
//        );}