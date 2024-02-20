package org.firstinspires.ftc.teamcode.auto.RED;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commandbase.instantCommands.gripRotateCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.gripperCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outGripperCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.rackCommand;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.HangerAndDrone;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.intakeSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.outakeSubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectories.trajectory_redclose_LEFT;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;
@Disabled
@Autonomous(group = "drive")
public class ARC extends CommandOpMode {
    intakeSubsystem intake = null;
    outakeSubsystem outake = null;
    SampleMecanumDrive drive = null;

    @Override
    public void runOpMode() throws InterruptedException {
        // Instantiate your drive class (replace SampleMecanumDrive with your actual drive class)
        drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(7, -70, Math.toRadians(0));
        drive.setPoseEstimate(startPose);
        trajectory_redclose_LEFT trajectoriesLoader = new trajectory_redclose_LEFT();

        List<TrajectorySequence> trajectories = trajectoriesLoader.loadTrajectories(drive);
        for (TrajectorySequence trajectory : trajectories) {
            drive.followTrajectorySequence(trajectory);
            schedule(new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE));
//        schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
            schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK));

            schedule(new rackCommand(intake, intakeSubsystem.RackState.LOWseUPAR));
            schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
        }

        waitForStart();

        if (isStopRequested()) return;
//        drive.followTrajectorySequence(Trajectory_LEFT);

    }


    @Override
    public void initialize() {

    }
}
