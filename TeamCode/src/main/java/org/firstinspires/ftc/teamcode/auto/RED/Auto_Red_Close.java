package org.firstinspires.ftc.teamcode.auto.RED;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
@Disabled
@Autonomous(group = "drive")
public class Auto_Red_Close extends CommandOpMode {
    intakeSubsystem intake = null;
    outakeSubsystem outake = null;
    HangerAndDrone endgame = null;
    Servo rack;
    SampleMecanumDrive drive = null;

    @Override
    public void runOpMode() throws InterruptedException {
        rack = hardwareMap.get(Servo.class, "rack");
        intake = new intakeSubsystem(hardwareMap, telemetry);
        outake = new outakeSubsystem(hardwareMap, telemetry);
        endgame = new HangerAndDrone(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        ElapsedTime runtime = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose2d startPose = new Pose2d(7, -70, Math.toRadians(0));
        Pose2d startleft1Pose = new Pose2d(43, -38, Math.toRadians(0));
        Pose2d startleft2Pose = new Pose2d(44, -38, Math.toRadians(0));

        drive.setPoseEstimate(startPose);
        schedule(new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE));
        schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK));
        schedule(new rackCommand(intake, intakeSubsystem.RackState.LOWseUPAR));
        schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));


        waitForStart();

        if (isStopRequested()) return;
//TODO: LEFT CYCLE AUTO

        TrajectorySequence left1 = drive.trajectorySequenceBuilder(startPose)
//
                .lineToConstantHeading(new Vector2d(10, -42))
//
                .lineToConstantHeading(new Vector2d(2, -38))
//                .splineToConstantHeading(new Vector2d(1, -42),0 )
                .addDisplacementMarker(() -> {
                    schedule(new ParallelCommandGroup(
                            new rackCommand(intake, intakeSubsystem.RackState.LOW),
                            new gripperCommand(intake, intakeSubsystem.GripperState.OPEN)
                    ));
                })
//                .waitSeconds(5000)
                .lineToConstantHeading(new Vector2d(43, -40))

////                .splineToConstantHeading(new Vector2d(30, -58), 0)
//                .addDisplacementMarker(() -> {
//                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROP));
//                    schedule(new WaitCommand(1000));
//                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPEN));
//                })
//                .waitSeconds(500)
//                .lineToConstantHeading(new Vector2d(28,-35))
//


                .build();
        //TODO:LEFT2
        TrajectorySequence left2 = drive.trajectorySequenceBuilder(startleft1Pose)

                .lineToConstantHeading(new Vector2d(45,-38))
                .addDisplacementMarker(() -> {
//                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROP));
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
                    new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND);


//                    schedule(new WaitCommand(300));
//                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROP));
//                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPEN));
//                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.PICK));

                })
//                .lineToConstantHeading(new Vector2d(40,-38))
//                .lineToConstantHeading(new Vector2d(40,-65))
//                .lineToConstantHeading(new Vector2d(48,-65))
//                .waitSeconds(500)
//                .lineToConstantHeading(new Vector2d(28,-35))
//


                .build();
        //TODO:LEFT3
        TrajectorySequence left3 = drive.trajectorySequenceBuilder(startleft2Pose)

                .lineToConstantHeading(new Vector2d(46,-38))
                .addDisplacementMarker(() -> {
//                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.PICKseUPAR));
//                    schedule(new WaitCommand(300));
//                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROP));
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPEN));
                })
//                .lineToConstantHeading(new Vector2d(40,-38))
//                .addDisplacementMarker(() -> {
////                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.PICKseUPAR));
////                    schedule(new WaitCommand(300));
////                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROP));
//                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPEN));
//                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.PICK));
//
//                })                .lineToConstantHeading(new Vector2d(40,-65))
//                .lineToConstantHeading(new Vector2d(48,-65))
                .build();
//TODO:LEFT4
        TrajectorySequence left4 = drive.trajectorySequenceBuilder(startleft2Pose)

                .lineToConstantHeading(new Vector2d(46,-38))
                .addDisplacementMarker(() -> {
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.PICK));
        schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
                })
                .lineToConstantHeading(new Vector2d(40,-38))
                .lineToConstantHeading(new Vector2d(40,-65))
                .lineToConstantHeading(new Vector2d(49,-65))
                .addDisplacementMarker(() -> {
                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
                })
                .build();




        //TODO: CENTRE CYCLE AUTO

        TrajectorySequence centre = drive.trajectorySequenceBuilder(startPose)
//
                .lineToConstantHeading(new Vector2d(12, -36))
//
                .waitSeconds(100)
                .addDisplacementMarker(() -> {
                    schedule(new ParallelCommandGroup(
                            new rackCommand(intake, intakeSubsystem.RackState.LOW),
                            new gripperCommand(intake, intakeSubsystem.GripperState.OPEN)
//                            new gripperCommand(intake, intakeSubsystem.GripperState.OPEN)
                    ));
                })
                .lineToConstantHeading(new Vector2d(43, -40))

//                .addDisplacementMarker(() -> {
//                    schedule(new gripperCommand(intake, intakeSubsystem.GripperState.OPEN));
//                })
//
////                .lineToConstantHeading(new Vector2d(3, -48))
//                .splineToConstantHeading(new Vector2d(1, -42),0 )
//                .addDisplacementMarker(() -> {
//                    schedule(new ParallelCommandGroup(
//                            new rackCommand(intake, intakeSubsystem.RackState.LOW),
//                            new gripperCommand(intake, intakeSubsystem.GripperState.OPEN)
//                    ));
//                })
//                .waitSeconds(800)
//                .lineToConstantHeading(new Vector2d(30, -58))
//
////                .splineToConstantHeading(new Vector2d(30, -58), 0)
//                .addDisplacementMarker(() -> {
//                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROP));
//                })
//                .waitSeconds(500)
//                .lineToConstantHeading(new Vector2d(28,-40))
//


                .build();

        drive.followTrajectorySequence(left1);
        sleep(300);
        drive.followTrajectorySequence(left2);
        sleep(500);
        drive.followTrajectorySequence(left3);
        sleep(500);
        drive.followTrajectorySequence(left4);
    }

        @Override
        public void initialize () {

        }
    }

