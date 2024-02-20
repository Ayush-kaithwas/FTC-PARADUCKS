//package org.firstinspires.ftc.teamcode.trajectorysequence;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//
//import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.SequenceSegment;
//
//import java.util.ArrayList;
//import java.util.Collections;
//import java.util.List;
//
//public class TrajectorySequence {
//    private final List<SequenceSegment> sequenceList;
//    private final List<TrajectoryPair> trajectoryPairs = new ArrayList<>();
//
//    public TrajectorySequence(List<SequenceSegment> sequenceList) {
//        if (sequenceList.size() == 0) throw new EmptySequenceException();
//
//        this.sequenceList = Collections.unmodifiableList(sequenceList);
//    }
//
//    public TrajectorySequence(List<TrajectoryPair> trajectoryPairs, List<SequenceSegment> sequenceList) {
//        this.trajectoryPairs.addAll(trajectoryPairs);
//        if (sequenceList.size() == 0) throw new EmptySequenceException();
//        this.sequenceList = Collections.unmodifiableList(sequenceList);
//    }
//
//    public Pose2d start() {
//        return sequenceList.get(0).getStartPose();
//    }
//
//    public Pose2d end() {
//        return sequenceList.get(sequenceList.size() - 1).getEndPose();
//    }
//
//    public double duration() {
//        double total = 0.0;
//
//        for (SequenceSegment segment : sequenceList) {
//            total += segment.getDuration();
//        }
//
//        for (TrajectoryPair pair : trajectoryPairs) {
//            total += pair.getSleepMillis() / 1000.0;
//            total += pair.getTrajectory().duration();
//        }
//
//        return total;
//    }
//
//    public SequenceSegment get(int i) {
//        return sequenceList.get(i);
//    }
//
//    public int size() {
//        return sequenceList.size();
//    }
//
//    public List<TrajectoryPair> getTrajectoryPairs() {
//        return trajectoryPairs;
//    }
//
//    public static class TrajectoryPair {
//        private final Trajectory trajectory;
//        private final long sleepMillis;
//
//        public TrajectoryPair(Trajectory trajectory, long sleepMillis) {
//            this.trajectory = trajectory;
//            this.sleepMillis = sleepMillis;
//        }
//
//        public Trajectory getTrajectory() {
//            return trajectory;
//        }
//
//        public long getSleepMillis() {
//            return sleepMillis;
//        }
//    }
//}

package org.firstinspires.ftc.teamcode.trajectorysequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.SequenceSegment;

import java.util.Collections;
import java.util.List;

public class TrajectorySequence {
    private final List<SequenceSegment> sequenceList;

    public TrajectorySequence(List<SequenceSegment> sequenceList) {
        if (sequenceList.size() == 0) throw new EmptySequenceException();

        this.sequenceList = Collections.unmodifiableList(sequenceList);
    }

    public Pose2d start() {
        return sequenceList.get(0).getStartPose();
    }

    public Pose2d end() {
        return sequenceList.get(sequenceList.size() - 1).getEndPose();
    }

    public double duration() {
        double total = 0.0;

        for (SequenceSegment segment : sequenceList) {
            total += segment.getDuration();
        }

        return total;
    }

    public SequenceSegment get(int i) {
        return sequenceList.get(i);
    }

    public int size() {
        return sequenceList.size();
    }
}



