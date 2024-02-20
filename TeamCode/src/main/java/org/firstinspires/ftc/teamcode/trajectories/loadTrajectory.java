package org.firstinspires.ftc.teamcode.trajectories;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public interface loadTrajectory {
    TrajectorySequence loadLeftTrajectory(SampleMecanumDrive drive);
    TrajectorySequence loadRightTrajectory(SampleMecanumDrive drive);
    TrajectorySequence loadCentreTrajectory(SampleMecanumDrive drive);
}


