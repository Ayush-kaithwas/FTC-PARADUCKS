package org.firstinspires.ftc.teamcode.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commandbase.instantCommands.gripRotateCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.gripperCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outArmCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outArmExtensionCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outGripperCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.rackCommand;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.HangerAndDrone;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.intakeSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.outakeSubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import java.util.ArrayList;
import java.util.List;

public class trajectory_redclose_LEFT extends CommandOpMode implements loadTrajectory {
    intakeSubsystem intake = null;
    outakeSubsystem outake = null;
    Servo rack;
    SampleMecanumDrive drive = null;
    Pose2d startPose = new Pose2d(7, -70, Math.toRadians(0));
    Pose2d startleft1Pose = new Pose2d(43, -37, Math.toRadians(0));
    Pose2d startleft2Pose = new Pose2d(44, -37, Math.toRadians(0));
    Pose2d startleft3Pose = new Pose2d(46, -37, Math.toRadians(0));
    public List<TrajectorySequence> loadTrajectories(SampleMecanumDrive drive) {
        List<TrajectorySequence> trajectories = new ArrayList<>();
                // TODO: LEFT CYCLE AUTO
                // TODO: LEFT1
                TrajectorySequence left1 = drive.trajectorySequenceBuilder(startPose)
                        .lineToConstantHeading(new Vector2d(2, -38))
                        .addDisplacementMarker(() -> {
                            // Assuming schedule is available in your TrajectorySequenceBuilder
                            schedule(new ParallelCommandGroup(
                                    new rackCommand(intake, intakeSubsystem.RackState.LOW),
                                    new gripperCommand(intake, intakeSubsystem.GripperState.OPEN)
                            ));
                        })
                        .lineToConstantHeading(new Vector2d(43, -38))
                        .build();
                trajectories.add(left1);

                // TODO: LEFT2
                TrajectorySequence left2 = drive.trajectorySequenceBuilder(startleft1Pose)
                        .lineToConstantHeading(new Vector2d(45, -37))
                        .addDisplacementMarker(() -> {
                            // Assuming schedule is available in your TrajectorySequenceBuilder
                            schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROP));
                            schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
                            new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND);
                        })
                        .build();
                trajectories.add(left2);

                // TODO: LEFT3
                TrajectorySequence left3 = drive.trajectorySequenceBuilder(startleft2Pose)
                        .lineToConstantHeading(new Vector2d(46, -37))
                        .addDisplacementMarker(() -> {
                            // Assuming schedule is available in your TrajectorySequenceBuilder
                            schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPEN));
                        })
                        .build();
                trajectories.add(left3);

                // TODO: LEFT4
                TrajectorySequence left4 = drive.trajectorySequenceBuilder(startleft3Pose)
                        .addDisplacementMarker(() -> {
                            // Assuming schedule is available in your TrajectorySequenceBuilder
                            schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.PICK));
                            schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
                        })
                        .lineToConstantHeading(new Vector2d(40, -65))
                        .lineToConstantHeading(new Vector2d(49, -65))
                        .addDisplacementMarker(() -> {
                            // Assuming schedule is available in your TrajectorySequenceBuilder
                            schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
                        })
                        .build();
                trajectories.add(left4);

                return trajectories;
            }

            @Override
            public void initialize() {
            }

    @Override
    public TrajectorySequence loadLeftTrajectory(SampleMecanumDrive drive) {
        return null;
    }

    @Override
    public TrajectorySequence loadRightTrajectory(SampleMecanumDrive drive) {
        return null;
    }

    @Override
    public TrajectorySequence loadCentreTrajectory(SampleMecanumDrive drive) {
        return null;
    }
}
