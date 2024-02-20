//package org.firstinspires.ftc.teamcode.auto.RED.trajectories_red;
//
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.commandbase.instantCommands.gripRotateCommand;
//import org.firstinspires.ftc.teamcode.commandbase.instantCommands.gripperCommand;
//import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outArmCommand;
//import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outArmExtensionCommand;
//import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outGripperCommand;
//import org.firstinspires.ftc.teamcode.commandbase.instantCommands.rackCommand;
//import org.firstinspires.ftc.teamcode.commandbase.subsytem.HangerAndDrone;
//import org.firstinspires.ftc.teamcode.commandbase.subsytem.intakeSubsystem;
//import org.firstinspires.ftc.teamcode.commandbase.subsytem.outakeSubsystem;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//
//import java.util.ArrayList;
//import java.util.List;
//
//public class leftTrajectory extends CommandOpMode {
//    intakeSubsystem intake = null;
//    outakeSubsystem outake = null;
//    SampleMecanumDrive drive = null;
//    Pose2d startPose = new Pose2d(7, -70, Math.toRadians(0));
//    Pose2d startleft1Pose = new Pose2d(43, -38, Math.toRadians(0));
//    Pose2d startleft2Pose = new Pose2d(44, -38, Math.toRadians(0));
//    Pose2d startleft3Pose = new Pose2d(46, -38, Math.toRadians(0));
//
////    public static List<TrajectorySequence> allLefttrajectories(SampleMecanumDrive drive, Pose2d startPose) {
////        List<TrajectorySequence> trajectories = new ArrayList<>();
////
////        trajectories.add(left1(drive, startPose));
////        trajectories.add(left2(drive, startPose));
////        trajectories.add(left3(drive, startPose));
////        trajectories.add(left4(drive, startPose));
////
////        return trajectories;
////    }
//
//    TrajectorySequence left1 = drive.trajectorySequenceBuilder(startPose)
////
////                .splineToConstantHeading(new Vector2d(1.5, -38),0)
//            .lineToConstantHeading(new Vector2d(8, -38))
//            .lineToConstantHeading(new Vector2d(1.5, -38))
//            .addDisplacementMarker(() -> {
//                schedule(new ParallelCommandGroup(
////                            new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK),
//                        new rackCommand(intake, intakeSubsystem.RackState.LOW),
//                        new gripperCommand(intake, intakeSubsystem.GripperState.OPEN)
//                ));
//            })
//            .lineToConstantHeading(new Vector2d(43, -38))
//            .build();
//
//
//    //TODO:LEFT2
//    TrajectorySequence left2 = drive.trajectorySequenceBuilder(startleft1Pose)
//
//            .lineToConstantHeading(new Vector2d(45, -37))
//            .addDisplacementMarker(() -> {
////                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
//                schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROP));
//                schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
//                new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND);
//
//
//            })
//            .build();
//
//    //TODO:LEFT3
//    TrajectorySequence left3 = drive.trajectorySequenceBuilder(startleft2Pose)
//
//            .lineToConstantHeading(new Vector2d(46, -38))
//            .addDisplacementMarker(() -> {
//                schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPEN));
//            })
//            .build();
//    //TODO:LEFT4
//    public TrajectorySequence left4 = drive.trajectorySequenceBuilder(startleft3Pose)
//
//            .addDisplacementMarker(() -> {
//        schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.PICK));
//        schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
//    })
//            ///////////////////////////////////////////////////////////
//            //TODO:PARK RIGHT
//            .lineToConstantHeading(new Vector2d(40, -65))
//            .lineToConstantHeading(new Vector2d(48.6, -65))
//            ///////////////////////////////////////////////////////////
////          ///////////////////////////////////////////////////////////
////                //TODO:PARK LEFT
////                .lineToConstantHeading(new Vector2d(40,-27))
////                .lineToConstantHeading(new Vector2d(49,-27))
////                ///////////////////////////////////////////////////////////
//            .addDisplacementMarker(() -> {
//                schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
//            })
//            .build();
//
//
//
//    @Override
//    public void initialize() {
//
//
//    }
//
//
//
