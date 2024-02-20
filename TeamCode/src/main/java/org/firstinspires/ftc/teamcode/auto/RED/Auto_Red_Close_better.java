package org.firstinspires.ftc.teamcode.auto.RED;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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
@Config
@Autonomous(group = "drive")
public class Auto_Red_Close_better extends CommandOpMode {
    intakeSubsystem intake = null;
    outakeSubsystem outake = null;
    HangerAndDrone endgame = null;
    Servo rack;
    SampleMecanumDrive drive = null;

    public static String marker = "RIGHT";
    public static int randompos = 2;


    @Override
    public void runOpMode() throws InterruptedException {
        rack = hardwareMap.get(Servo.class, "rack");
        intake = new intakeSubsystem(hardwareMap, telemetry);
        outake = new outakeSubsystem(hardwareMap, telemetry);
        endgame = new HangerAndDrone(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
//        ElapsedTime runtime = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose2d startPose = new Pose2d(7, -70, Math.toRadians(0));
        Pose2d startleft1Pose = new Pose2d(43, -37, Math.toRadians(0));
        Pose2d startleft2Pose = new Pose2d(44, -37, Math.toRadians(0));
        Pose2d startleft3Pose = new Pose2d(46, -37, Math.toRadians(0));
        Pose2d startcentre1Pose = new Pose2d(41, -39.5, Math.toRadians(0));
        Pose2d startcentre2Pose = new Pose2d(42.7, -39.5, Math.toRadians(0));
        Pose2d startcentre3Pose = new Pose2d(42, -39.5, Math.toRadians(0));
        Pose2d startright1Pose = new Pose2d(23.5,-38, Math.toRadians(0));
        Pose2d startright2Pose = new Pose2d(45,-48.6, Math.toRadians(0));
        Pose2d startright3Pose = new Pose2d(46,-48.6, Math.toRadians(0));


        drive.setPoseEstimate(startPose);
        schedule(new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE));
        schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID)); // for Left only
//        schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK));

                schedule(new rackCommand(intake, intakeSubsystem.RackState.LOWseUPAR));
        schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
        schedule(new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.INIT));



        waitForStart();

        if (isStopRequested()) return;
//TODO: LEFT CYCLE AUTO
        TrajectorySequence left0 = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    schedule(new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE));
                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK)); // for Left only
//        schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK));

                    schedule(new rackCommand(intake, intakeSubsystem.RackState.LOWseUPAR));
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
                    schedule(new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.INIT));
                })
                .build();
//TODO:LEFT1
        TrajectorySequence left1 = drive.trajectorySequenceBuilder(startPose)
//
//                .splineToConstantHeading(new Vector2d(1.5, -38),0)
                .lineToConstantHeading(new Vector2d(8, -38))
                .lineToConstantHeading(new Vector2d(1.5, -38))
                .addDisplacementMarker(() -> {
                    schedule(new ParallelCommandGroup(
//                            new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK),
                            new rackCommand(intake, intakeSubsystem.RackState.LOW),
                            new gripperCommand(intake, intakeSubsystem.GripperState.OPEN)
                    ));
                })
//                .lineToConstantHeading(new Vector2d(43, -38))
                .build();
        //TODO:LEFT2
        TrajectorySequence left2 = drive.trajectorySequenceBuilder(left1.end())

                .lineToConstantHeading(new Vector2d(40,-38.57))//-38.7//39//35.5 for extreme left//.37
                .addDisplacementMarker(() -> {
//                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROP));
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
//                    new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND);


                })
                .build();
        //TODO:LEFT3
        TrajectorySequence left3 = drive.trajectorySequenceBuilder(left2.end())

                .lineToConstantHeading(new Vector2d(41.5,-38.57))
                .addDisplacementMarker(() -> {
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPEN));
                })
                .build();
        //TODO:LEFT4
        TrajectorySequence left4 = drive.trajectorySequenceBuilder(left3.end())
                .addDisplacementMarker(() -> {
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.PICK));
                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
                })
                ///////////////////////////////////////////////////////////
                //TODO:PARK RIGHT
                .lineToConstantHeading(new Vector2d(36,-69.5))
                .lineToConstantHeading(new Vector2d(48.6,-69.5))
                ///////////////////////////////////////////////////////////
//                ///////////////////////////////////////////////////////////
//                //TODO:PARK LEFT
//                .lineToConstantHeading(new Vector2d(40,-27))
//                .lineToConstantHeading(new Vector2d(49,-27))
//                ///////////////////////////////////////////////////////////
                .addDisplacementMarker(() -> {
                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
                })
                .build();




        //TODO: CENTRE CYCLE AUTO
        //TODO:CENTRE1
        TrajectorySequence centre0 = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    schedule(new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE));
                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK)); // for Left only
//        schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK));

                    schedule(new rackCommand(intake, intakeSubsystem.RackState.LOWseUPAR));
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
                    schedule(new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.INIT));
                })
                .build();
        TrajectorySequence centre1 = drive.trajectorySequenceBuilder(startPose)
//
                .lineToConstantHeading(new Vector2d(18, -31))
                .addDisplacementMarker(() -> {
                    schedule(new ParallelCommandGroup(
                            new rackCommand(intake, intakeSubsystem.RackState.LOW),
                            new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK),
                            new gripperCommand(intake, intakeSubsystem.GripperState.OPEN)
//                            new gripperCommand(intake, intakeSubsystem.GripperState.OPEN)
                    ));
                })
//                .lineToConstantHeading(new Vector2d(41.9, -43.5))

                .build();
        //TODO:CENTRE2
        TrajectorySequence centre2 = drive.trajectorySequenceBuilder(centre1.end())

                .lineToConstantHeading(new Vector2d(39,-44.6))//-43.5
                .addDisplacementMarker(() -> {
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROP));
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
//                    new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND);
                })
                .build();
        //TODO:CENTRE3
        TrajectorySequence centre3 = drive.trajectorySequenceBuilder(centre2.end())

                .lineToConstantHeading(new Vector2d(41,-44.6))
                .addDisplacementMarker(() -> {;
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPEN));
                })
                .build();
        //TODO:CENTRE4
        TrajectorySequence centre4 = drive.trajectorySequenceBuilder(centre3.end())


                .addDisplacementMarker(() -> {
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.PICK));
                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
                })
//                .lineToConstantHeading(new Vector2d(40,--38))
                ///////////////////////////////////////////////////////////
                //TODO:PARK RIGHT
                .lineToConstantHeading(new Vector2d(39,-65))
                .lineToConstantHeading(new Vector2d(48.6,-68.5))
                /////////////////////////////////////////////////////////////
//                ///////////////////////////////////////////////////////////
//                //TODO:PARK LEFT
//                .lineToConstantHeading(new Vector2d(40,-27))
//                .lineToConstantHeading(new Vector2d(49,-27))
//                ///////////////////////////////////////////////////////////
                .addDisplacementMarker(() -> {
                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
                })
                .build();

//        //TODO: RIGHT CYCLE AUTO(YELLOW FIRST)
//        //TODO:RIGHT1
//        TrajectorySequence right1 = drive.trajectorySequenceBuilder(startPose)
////                .lineToConstantHeading(new Vector2d(45,-48.6))
//                .lineToConstantHeading(new Vector2d(45,-48.73))
//
//                .addDisplacementMarker(() -> {
//                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROP));
//                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
//                    new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND);
//                })
//
//
//                .build();
//        //TODO:RIGHT2
//        TrajectorySequence right2 = drive.trajectorySequenceBuilder(right1.end())
//
//                .lineToConstantHeading(new Vector2d(46,-48.73))
//                .addDisplacementMarker(() -> {
//                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPEN));
//                })
//                .build();
////        TODO:RIGHT3
//        TrajectorySequence right3 = drive.trajectorySequenceBuilder(right2.end())
//                .lineToConstantHeading(new Vector2d(45,-40))
//                .addDisplacementMarker(() -> {
//                          schedule  (new outArmCommand(outake, outakeSubsystem.OutArmState.PICK));
//
//                })
//                .build();
////        TODO:RIGHT4
//        TrajectorySequence right4 = drive.trajectorySequenceBuilder(right3.end())
//
//                .lineToConstantHeading(new Vector2d(23.5,-38))
//                .addDisplacementMarker(() -> {
//                    schedule(new ParallelCommandGroup(
//                            new rackCommand(intake, intakeSubsystem.RackState.LOW),
//                            new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK),
//                            new gripperCommand(intake, intakeSubsystem.GripperState.OPEN)
//                    ));
//                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK));
//                })
//                .build();

       //TODO: RIGHTAUTO2 (PURPLE FIRST)
        //TODO:RIGHT11
        TrajectorySequence right0 = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    schedule(new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE));
                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK)); // for Left only
//        schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK));

                    schedule(new rackCommand(intake, intakeSubsystem.RackState.LOWseUPAR));
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
                    schedule(new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.INIT));
                })
                .build();
        TrajectorySequence right11 = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(25.5,-39.5))
                .addDisplacementMarker(() -> {
                    schedule(new ParallelCommandGroup(
                            new rackCommand(intake, intakeSubsystem.RackState.LOW),
                            new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK),
                            new gripperCommand(intake, intakeSubsystem.GripperState.OPEN)
                    ));
                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK));
                })
                .build();

        //TODO:RIGHT12
        TrajectorySequence right12 = drive.trajectorySequenceBuilder(startright1Pose)
                .lineToConstantHeading(new Vector2d(40,-49.9))//-49.7
                .addDisplacementMarker(() -> {
                    schedule(new outArmExtensionCommand(outake,outakeSubsystem.OutArmExtensionState.INIT));
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROP));
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
//                    schedule(new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND));//Try without extend and take thoda aage
                })
                .build();


//        TODO:RIGHT13
        TrajectorySequence right13 = drive.trajectorySequenceBuilder(startright2Pose)
                .lineToConstantHeading(new Vector2d(42.5,-49.9))//-\
                // 49//-48.3//-47.6
                .addDisplacementMarker(() -> {
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPEN));
                })
                .build();

//        TODO:RIGHT14
        TrajectorySequence right14 = drive.trajectorySequenceBuilder(startright3Pose)
                .lineToConstantHeading(new Vector2d(39,-49.9))
                .addDisplacementMarker(() -> {
                    schedule(new ParallelCommandGroup(
                            (new outArmCommand(outake, outakeSubsystem.OutArmState.PICK)),
                              new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID)
                    ));
                })
                ///////////////////////////////////////////////////////////
                //TODO:PARK RIGHT
                .lineToConstantHeading(new Vector2d(39,-69.5))
                .lineToConstantHeading(new Vector2d(49,-69.5))
                ///////////////////////////////////////////////////////////
//                ///////////////////////////////////////////////////////////
//                //TODO:PARK LEFT
//                .lineToConstantHeading(new Vector2d(40,-27))
//                .lineToConstantHeading(new Vector2d(49,-27))
//                ///////////////////////////////////////////////////////////

                .build();


////////////////////REMOVE MARKERS////////////////////////////
        //TODO:LEFT
        if(marker=="LEFT"|| randompos==1) {
            drive.followTrajectorySequence(left0);
            sleep(1000);
            drive.followTrajectorySequence(left1);
            sleep(200);
            schedule(new rackCommand(intake, intakeSubsystem.RackState.LOWseUPAR));
            drive.followTrajectorySequence(left2);
            sleep(600);
            schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROPseUPAR));
            sleep(100);
            drive.followTrajectorySequence(left3);
            sleep(900);
            drive.followTrajectorySequence(left4);
            sleep(999999);
        }

//        //TODO:CENTRE
        if(marker=="CENTRE"|| randompos==0 ) {
            drive.followTrajectorySequence(centre0);
            sleep(1000);
            drive.followTrajectorySequence(centre1);
            sleep(200);
            schedule(new rackCommand(intake, intakeSubsystem.RackState.LOWseUPAR));
            drive.followTrajectorySequence(centre2);
            sleep(600);
            schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROPseUPAR));
            sleep(100);
            drive.followTrajectorySequence(centre3);
            sleep(900);
            drive.followTrajectorySequence(centre4);
            sleep(999999);

        }

        //TODO:RIGHT

//        drive.followTrajectorySequence(right1);
//        sleep(300);
//        drive.followTrajectorySequence(right2);
//        sleep(200);
//        drive.followTrajectorySequence(right3);
//        sleep(200);
//        drive.followTrajectorySequence(right4);

////        //TODO RIGHT (PURPLE FIRST)
        if(marker=="RIGHT" || randompos==2 ) {
            drive.followTrajectorySequence(right0);
            sleep(1000);
            drive.followTrajectorySequence(right11);
            sleep(200);
            schedule(new rackCommand(intake, intakeSubsystem.RackState.LOWseUPAR));
            drive.followTrajectorySequence(right12);
            sleep(600);
            schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROPseUPAR));
            sleep(100);
            drive.followTrajectorySequence(right13);
            sleep(900);
            drive.followTrajectorySequence(right14);
            sleep(999999);

        }



    }

    @Override
    public void initialize () {

    }
}

