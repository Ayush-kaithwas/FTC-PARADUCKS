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
public class Red_Close_1cycle extends CommandOpMode {
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
        schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK)); // for Left only
//        schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK));

                schedule(new rackCommand(intake, intakeSubsystem.RackState.LOWseUPAR));
        schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
        schedule(new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.INIT));



        waitForStart();

        if (isStopRequested()) return;
//TODO: LEFT CYCLE AUTO
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

                .lineToConstantHeading(new Vector2d(40,-35.5))//.37
                .addDisplacementMarker(() -> {
//                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROP));
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
//                    new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND);


                })
                .build();
        //TODO:LEFT3
        TrajectorySequence left3 = drive.trajectorySequenceBuilder(left2.end())

                .lineToConstantHeading(new Vector2d(41.5,-35.5))
                .addDisplacementMarker(() -> {
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPEN));
                })
                .build();
        //TODO:LEFT 4
        TrajectorySequence left4 = drive.trajectorySequenceBuilder(left3.end())
                .addDisplacementMarker(() -> {
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.PICK));
                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
                })
                .lineToConstantHeading(new Vector2d(20,-20))//20
                .lineToConstantHeading(new Vector2d(-55,-20))

                .addDisplacementMarker(() -> {
                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.AUTO));
                    schedule(new rackCommand(intake, intakeSubsystem.RackState.AUTO_TOP2));
                    schedule(new gripperCommand(intake, intakeSubsystem.GripperState.OPEN));
                })
                .lineToConstantHeading(new Vector2d(-61,-17.9))//-65

                .build();
        //TODO:LEFT 5
        TrajectorySequence left5 = drive.trajectorySequenceBuilder(left4.end())
                .lineToConstantHeading(new Vector2d(-68.5,-17.9))//-70.4//-17.6
                .lineToConstantHeading(new Vector2d(-69.1,-17.9))//702
                .addDisplacementMarker(() -> {
                    schedule(new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE));
                })
//                .lineToConstantHeading(new Vector2d(-60,-20))

                .build();
        //TODO:LEFT 6
        TrajectorySequence left6 = drive.trajectorySequenceBuilder(left5.end())

                .lineToConstantHeading(new Vector2d(-67,-17.9))
//                .addDisplacementMarker(() -> {
//                    schedule(new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE));
//                })
                .build();
        //TODO:LEFT 7
        TrajectorySequence left7 = drive.trajectorySequenceBuilder(left6.end())

                .lineToConstantHeading(new Vector2d(30,-20))
                .addDisplacementMarker(() -> {
                    schedule(new ParallelCommandGroup(
                            new outArmCommand(outake, outakeSubsystem.OutArmState.PICK),
                            new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPEN),
                            new gripRotateCommand(intake,intakeSubsystem.GripRotateState.MID),
                            new rackCommand(intake, intakeSubsystem.RackState.AUTO_TOP)));
                })
                .build();
        //TODO:LEFT 8
        TrajectorySequence left8 = drive.trajectorySequenceBuilder(left7.end())

                .addDisplacementMarker(() -> {
                    schedule(new ParallelCommandGroup(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MORE_DROP),
                            new rackCommand(intake, intakeSubsystem.RackState.TOP)));
                })
                .lineToConstantHeading(new Vector2d(31.00001,-20))

                .build();
        //TODO:LEFT 9
        TrajectorySequence left9 = drive.trajectorySequenceBuilder(left8.end())

                .addDisplacementMarker(() -> {
                    schedule(new gripperCommand(intake, intakeSubsystem.GripperState.OPEN));
                })

                .lineToConstantHeading(new Vector2d(31.0000001,-20))

                .build();
        //TODO:LEFT 10
        TrajectorySequence left10 = drive.trajectorySequenceBuilder(left9.end())

                .addDisplacementMarker(() -> {
                    schedule(new gripperCommand(intake, intakeSubsystem.GripperState.OPENUP));
                    schedule(new ParallelCommandGroup(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MIDseJYAADA),
                            new gripperCommand(intake, intakeSubsystem.GripperState.OPEN),
                            new gripperCommand(intake, intakeSubsystem.GripperState.OPENUP_BACK)));
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));//new

                })

                .lineToConstantHeading(new Vector2d(32.0001,-20))

                .build();


        //TODO:LEFT 11
        TrajectorySequence left11 = drive.trajectorySequenceBuilder(left10.end())

                .lineToConstantHeading(new Vector2d(36,-35.5))//.37
                .addDisplacementMarker(() -> {

                    new ParallelCommandGroup(
                            new gripperCommand(intake, intakeSubsystem.GripperState.OPENUP),
                            new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID),
                            new rackCommand(intake, intakeSubsystem.RackState.LOWseUPAR),
                            new rackCommand(intake, intakeSubsystem.RackState.LOW),
                            new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE),
                            new gripperCommand(intake, intakeSubsystem.GripperState.INIT));


                })
                .build();
        //TODO:LEFT 12
        TrajectorySequence left12 = drive.trajectorySequenceBuilder(left11.end())

                .lineToConstantHeading(new Vector2d(40,-42.5))//-43.5
                .addDisplacementMarker(() -> {
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROP));
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
//                    new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND);
                })
                .build();
        //TODO:LEFT 13
        TrajectorySequence left13 = drive.trajectorySequenceBuilder(left12.end())

                .lineToConstantHeading(new Vector2d(42.6,-42.5))//42.5
                .addDisplacementMarker(() -> {
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPENRIGHT));
                })
                .lineToConstantHeading(new Vector2d(42.6,-44.5))
                .addDisplacementMarker(() -> {
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPENLEFT));
                })

                .build();
        //TODO:LEFT 14
        TrajectorySequence left14 = drive.trajectorySequenceBuilder(left13.end())
                .addDisplacementMarker(() -> {
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.PICK));
                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
                })
                ///////////////////////////////////////////////////////////
                //TODO:PARK RIGHT
                .lineToConstantHeading(new Vector2d(36,-68.8))
                .lineToConstantHeading(new Vector2d(48.6,-68.8))
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
//        //TODO:LEFT4
//        TrajectorySequence left4 = drive.trajectorySequenceBuilder(left3.end())
//                .addDisplacementMarker(() -> {
//                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.PICK));
//                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
//                })
//                ///////////////////////////////////////////////////////////
//                //TODO:PARK RIGHT
//                .lineToConstantHeading(new Vector2d(36,-69.5))
//                .lineToConstantHeading(new Vector2d(48.6,-69.5))
//                ///////////////////////////////////////////////////////////
////                ///////////////////////////////////////////////////////////
////                //TODO:PARK LEFT
////                .lineToConstantHeading(new Vector2d(40,-27))
////                .lineToConstantHeading(new Vector2d(49,-27))
////                ///////////////////////////////////////////////////////////
//                .addDisplacementMarker(() -> {
//                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
//                })
//                .build();




        //TODO: CENTRE CYCLE AUTO
        //TODO:CENTRE1
        TrajectorySequence centre1 = drive.trajectorySequenceBuilder(startPose)
//
                .lineToConstantHeading(new Vector2d(17, -31))//18
                .addDisplacementMarker(() -> {
                    schedule(new ParallelCommandGroup(
                            new rackCommand(intake, intakeSubsystem.RackState.LOW),
                            new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK),
                            new gripperCommand(intake, intakeSubsystem.GripperState.OPEN),
                    new rackCommand(intake, intakeSubsystem.RackState.LOWseUPAR)

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

        //TODO:CENTRE 4
        TrajectorySequence centre4 = drive.trajectorySequenceBuilder(centre3.end())
                .addDisplacementMarker(() -> {
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.PICK));
                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
                })
                .lineToConstantHeading(new Vector2d(20,-20))//20
                .lineToConstantHeading(new Vector2d(-55,-20))

                .addDisplacementMarker(() -> {
                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.AUTO));
                    schedule(new rackCommand(intake, intakeSubsystem.RackState.AUTO_TOP2));
                    schedule(new gripperCommand(intake, intakeSubsystem.GripperState.OPEN));
                })
                .lineToConstantHeading(new Vector2d(-61,-17.5))//-65

                .build();
        //TODO:CENTRE 5
        TrajectorySequence centre5 = drive.trajectorySequenceBuilder(centre4.end())
                .lineToConstantHeading(new Vector2d(-68.5,-17.5))//-70.4//-17.6
                .lineToConstantHeading(new Vector2d(-69.1,-17.5))//702
                .addDisplacementMarker(() -> {
                    schedule(new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE));
                })
//                .lineToConstantHeading(new Vector2d(-60,-20))

                .build();
        //TODO:CENTRE 6
        TrajectorySequence centre6 = drive.trajectorySequenceBuilder(centre5.end())

                .lineToConstantHeading(new Vector2d(-67,-17.5))
//                .addDisplacementMarker(() -> {
//                    schedule(new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE));
//                })
                .build();
        //TODO:CENTRE 7
        TrajectorySequence centre7 = drive.trajectorySequenceBuilder(centre6.end())

                .lineToConstantHeading(new Vector2d(30,-20))
                .addDisplacementMarker(() -> {
                    schedule(new ParallelCommandGroup(
                            new outArmCommand(outake, outakeSubsystem.OutArmState.PICK),
                            new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPEN),
                            new gripRotateCommand(intake,intakeSubsystem.GripRotateState.MID),
                            new rackCommand(intake, intakeSubsystem.RackState.AUTO_TOP)));
                })
                .build();
        //TODO:CENTRE 8
        TrajectorySequence centre8 = drive.trajectorySequenceBuilder(centre7.end())

                .addDisplacementMarker(() -> {
                            schedule(new ParallelCommandGroup(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MORE_DROP),
                                    new rackCommand(intake, intakeSubsystem.RackState.TOP)));
                })
                .lineToConstantHeading(new Vector2d(31.00001,-20))

                .build();
        //TODO:CENTRE 9
        TrajectorySequence centre9 = drive.trajectorySequenceBuilder(centre8.end())

                .addDisplacementMarker(() -> {
                    schedule(new gripperCommand(intake, intakeSubsystem.GripperState.OPEN));
                })

                .lineToConstantHeading(new Vector2d(31.0000001,-20))

                .build();
        //TODO:CENTRE 10
        TrajectorySequence centre10 = drive.trajectorySequenceBuilder(centre9.end())

                .addDisplacementMarker(() -> {
                    schedule(new gripperCommand(intake, intakeSubsystem.GripperState.OPENUP));
                    schedule(new ParallelCommandGroup(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MIDseJYAADA),
                                    new gripperCommand(intake, intakeSubsystem.GripperState.OPEN),
                                    new gripperCommand(intake, intakeSubsystem.GripperState.OPENUP_BACK)));
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));//new

                        })

                .lineToConstantHeading(new Vector2d(32.0001,-20))

                .build();


        //TODO:CENTRE11
        TrajectorySequence centre11 = drive.trajectorySequenceBuilder(centre10.end())

                .lineToConstantHeading(new Vector2d(36,-35.5))//.37
                .addDisplacementMarker(() -> {

                            new ParallelCommandGroup(
                                    new gripperCommand(intake, intakeSubsystem.GripperState.OPENUP),
                                    new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID),
                                    new rackCommand(intake, intakeSubsystem.RackState.LOWseUPAR),
                                    new rackCommand(intake, intakeSubsystem.RackState.LOW),
                                    new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE),
                                    new gripperCommand(intake, intakeSubsystem.GripperState.INIT));


                })
                .build();
        //TODO:CENTRE12
        TrajectorySequence centre12 = drive.trajectorySequenceBuilder(centre11.end())

                .lineToConstantHeading(new Vector2d(40,-35.5))//-43.5
                .addDisplacementMarker(() -> {
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROP));
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
//                    new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND);
                })
                .build();
        //TODO:CENTRE13
        TrajectorySequence centre13 = drive.trajectorySequenceBuilder(centre12.end())

                .lineToConstantHeading(new Vector2d(42.6,-36.5))//42.5
                .addDisplacementMarker(() -> {
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPENRIGHT));
                })
                .lineToConstantHeading(new Vector2d(42.6,-35.5))
                .addDisplacementMarker(() -> {
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPENLEFT));
                })

                .build();
        //TODO:CENTRE14
        TrajectorySequence centre14 = drive.trajectorySequenceBuilder(centre13.end())
                .addDisplacementMarker(() -> {
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.PICK));
                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
                })
                ///////////////////////////////////////////////////////////
                //TODO:PARK RIGHT
                .lineToConstantHeading(new Vector2d(36,-68.8))
                .lineToConstantHeading(new Vector2d(48.6,-68.8))
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
//        //TODO:CENTRE4
//        TrajectorySequence centre4 = drive.trajectorySequenceBuilder(centre3.end())
//
//
//                .addDisplacementMarker(() -> {
//                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.PICK));
//                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
//                })
////                .lineToConstantHeading(new Vector2d(40,--38))
//                ///////////////////////////////////////////////////////////
//                //TODO:PARK RIGHT
//                .lineToConstantHeading(new Vector2d(39,-65))
//                .lineToConstantHeading(new Vector2d(48.6,-69))
//                /////////////////////////////////////////////////////////////
////                ///////////////////////////////////////////////////////////
////                //TODO:PARK LEFT
////                .lineToConstantHeading(new Vector2d(40,-27))
////                .lineToConstantHeading(new Vector2d(49,-27))
////                ///////////////////////////////////////////////////////////
//                .addDisplacementMarker(() -> {
//                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
//                })
//                .build();

//        //TODO: RIGHT CYCLE AUTO(YELLOW FIRST)
//        //TODO:RIGHT1
//        TrajectorySequence right1 = drive.trajectorySequenceBuilder(startPose)
////                .lineToConstantHeading(new Vector2d(45,-48.6))
//                .lineToConstantHeading(new Vector2d(45,-48.6))
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
//        TrajectorySequence right2 = drive.trajectorySequenceBuilder(startright1Pose)
//
//                .lineToConstantHeading(new Vector2d(46,-48.6))
//                .addDisplacementMarker(() -> {
//                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPEN));
//                })
//                .build();
////        TODO:RIGHT3
//        TrajectorySequence right3 = drive.trajectorySequenceBuilder(startright3Pose)
//                .lineToConstantHeading(new Vector2d(45,-40))
//                .addDisplacementMarker(() -> {
//                          schedule  (new outArmCommand(outake, outakeSubsystem.OutArmState.PICK));
//
//                })
//                .build();
////        TODO:RIGHT4
//        TrajectorySequence right4 = drive.trajectorySequenceBuilder(startright3Pose)
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
        //TODO:RIGHT1
        TrajectorySequence right1 = drive.trajectorySequenceBuilder(startPose)
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

        //TODO:RIGHT2
        TrajectorySequence right2 = drive.trajectorySequenceBuilder(startright1Pose)
                .lineToConstantHeading(new Vector2d(40,-49.7))
                .addDisplacementMarker(() -> {
                    schedule(new outArmExtensionCommand(outake,outakeSubsystem.OutArmExtensionState.INIT));
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROP));
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
//                    schedule(new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND));//Try without extend and take thoda aage
                })
                .build();


//        TODO:RIGHT3
        TrajectorySequence right3 = drive.trajectorySequenceBuilder(startright2Pose)
                .lineToConstantHeading(new Vector2d(42.5,-49.7))//-\
                // 49//-48.3//-47.6
                .addDisplacementMarker(() -> {
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPEN));
                })
                .build();
        //TODO:RIGHT 4
        TrajectorySequence right4 = drive.trajectorySequenceBuilder(right3.end())
                .addDisplacementMarker(() -> {
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.PICK));
                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
                })
                .lineToConstantHeading(new Vector2d(20,-20))//20
                .lineToConstantHeading(new Vector2d(-55,-20))

                .addDisplacementMarker(() -> {
                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.AUTO));
                    schedule(new rackCommand(intake, intakeSubsystem.RackState.AUTO_TOP2));
                    schedule(new gripperCommand(intake, intakeSubsystem.GripperState.OPEN));
                })
                .lineToConstantHeading(new Vector2d(-61,-17.9))//-65

                .build();
        //TODO:RIGHT 5
        TrajectorySequence right5 = drive.trajectorySequenceBuilder(right4.end())
                .lineToConstantHeading(new Vector2d(-68.5,-17.9))//-70.4//-17.6
                .lineToConstantHeading(new Vector2d(-68.85,-17.9))//702
                .addDisplacementMarker(() -> {
                    schedule(new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE));
                })
//                .lineToConstantHeading(new Vector2d(-60,-20))

                .build();
        //TODO:RIGHT 6
        TrajectorySequence right6 = drive.trajectorySequenceBuilder(right5.end())

                .lineToConstantHeading(new Vector2d(-67,-17.9))
//                .addDisplacementMarker(() -> {
//                    schedule(new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE));
//                })
                .build();
        //TODO:RIGHT 7
        TrajectorySequence right7 = drive.trajectorySequenceBuilder(right6.end())

                .lineToConstantHeading(new Vector2d(30,-20))
                .addDisplacementMarker(() -> {
                    schedule(new ParallelCommandGroup(
                            new outArmCommand(outake, outakeSubsystem.OutArmState.PICK),
                            new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPEN),
                            new gripRotateCommand(intake,intakeSubsystem.GripRotateState.MID),
                            new rackCommand(intake, intakeSubsystem.RackState.AUTO_TOP)));
                })
                .build();
        //TODO:RIGHT 8
        TrajectorySequence right8 = drive.trajectorySequenceBuilder(right7.end())

                .addDisplacementMarker(() -> {
                    schedule(new ParallelCommandGroup(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MORE_DROP),
                            new rackCommand(intake, intakeSubsystem.RackState.TOP)));
                })
                .lineToConstantHeading(new Vector2d(31.00001,-20))

                .build();
        //TODO:RIGHT 9
        TrajectorySequence right9 = drive.trajectorySequenceBuilder(right8.end())

                .addDisplacementMarker(() -> {
                    schedule(new gripperCommand(intake, intakeSubsystem.GripperState.OPEN));
                })

                .lineToConstantHeading(new Vector2d(31.0000001,-20))

                .build();
        //TODO:RIGHT 10
        TrajectorySequence right10 = drive.trajectorySequenceBuilder(right9.end())

                .addDisplacementMarker(() -> {
                    schedule(new gripperCommand(intake, intakeSubsystem.GripperState.OPENUP));
                    schedule(new ParallelCommandGroup(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MIDseJYAADA),
                            new gripperCommand(intake, intakeSubsystem.GripperState.OPEN),
                            new gripperCommand(intake, intakeSubsystem.GripperState.OPENUP_BACK)));
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));//new

                })

                .lineToConstantHeading(new Vector2d(32.0001,-20))

                .build();


        //TODO:RIGHT 11
        TrajectorySequence right11 = drive.trajectorySequenceBuilder(right10.end())

                .lineToConstantHeading(new Vector2d(36,-35.5))//.37
                .addDisplacementMarker(() -> {

                    new ParallelCommandGroup(
                            new gripperCommand(intake, intakeSubsystem.GripperState.OPENUP),
                            new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID),
                            new rackCommand(intake, intakeSubsystem.RackState.LOWseUPAR),
                            new rackCommand(intake, intakeSubsystem.RackState.LOW),
                            new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE),
                            new gripperCommand(intake, intakeSubsystem.GripperState.INIT));


                })
                .build();
        //TODO:RIGHT 12
        TrajectorySequence right12 = drive.trajectorySequenceBuilder(right11.end())

                .lineToConstantHeading(new Vector2d(40,-42.5))//-43.5
                .addDisplacementMarker(() -> {
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROP));
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
//                    new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND);
                })
                .build();
        //TODO:RIGHT 13
        TrajectorySequence right13 = drive.trajectorySequenceBuilder(right12.end())

                .lineToConstantHeading(new Vector2d(42.6,-42.5))//42.5
                .addDisplacementMarker(() -> {
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPENRIGHT));
                })
                .lineToConstantHeading(new Vector2d(42.6,-44.5))
                .addDisplacementMarker(() -> {
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPENLEFT));
                })

                .build();
        //TODO:RIGHT 14
        TrajectorySequence right14 = drive.trajectorySequenceBuilder(right13.end())
                .addDisplacementMarker(() -> {
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.PICK));
                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
                })
                ///////////////////////////////////////////////////////////
                //TODO:PARK RIGHT
                .lineToConstantHeading(new Vector2d(36,-68.8))
                .lineToConstantHeading(new Vector2d(48.6,-68.8))
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

////        TODO:RIGHT4
//        TrajectorySequence right14 = drive.trajectorySequenceBuilder(startright3Pose)
//                .lineToConstantHeading(new Vector2d(39,-49.7))
//                .addDisplacementMarker(() -> {
//                    schedule(new ParallelCommandGroup(
//                            (new outArmCommand(outake, outakeSubsystem.OutArmState.PICK)),
//                              new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID)
//                    ));
//                })
//                ///////////////////////////////////////////////////////////
//                //TODO:PARK RIGHT
//                .lineToConstantHeading(new Vector2d(39,-69.5))
//                .lineToConstantHeading(new Vector2d(49,-69.5))
//                ///////////////////////////////////////////////////////////
////                ///////////////////////////////////////////////////////////
////                //TODO:PARK LEFT
////                .lineToConstantHeading(new Vector2d(40,-27))
////                .lineToConstantHeading(new Vector2d(49,-27))
////                ///////////////////////////////////////////////////////////
//
//                .build();


////////////////////REMOVE MARKERS////////////////////////////
        //TODO:LEFT
        if(marker=="LEFT"|| randompos==1) {
            drive.followTrajectorySequence(left1);
            drive.followTrajectorySequence(left2);
            sleep(600);
            schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROPseUPAR));
            sleep(100);
            drive.followTrajectorySequence(left3);
            sleep(900);
            drive.followTrajectorySequence(left4);
            sleep(500);
            drive.followTrajectorySequence(left5);
            sleep(500);
            drive.followTrajectorySequence(left6);
            sleep(1000);
            drive.followTrajectorySequence(left7);
//            sleep(1000);
            sleep(700);
            drive.followTrajectorySequence(left8);
            sleep(600);
            drive.followTrajectorySequence(left9);
            sleep(600);
            schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
            drive.followTrajectorySequence(left10);
            sleep(900);
            drive.followTrajectorySequence(left11);
            sleep(900);
            drive.followTrajectorySequence(left12);
            sleep(900);
            drive.followTrajectorySequence(left13);
            sleep(900);
            drive.followTrajectorySequence(left14);
            sleep(999999);
        }

//        //TODO:CENTRE
        if(marker=="CENTER"|| randompos==0 ) {
            drive.followTrajectorySequence(centre1);
            drive.followTrajectorySequence(centre2);
            sleep(600);
            schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROPseUPAR));
            sleep(100);
            drive.followTrajectorySequence(centre3);
            sleep(500);
            drive.followTrajectorySequence(centre4);
            sleep(500);
            drive.followTrajectorySequence(centre5);
            sleep(500);
            drive.followTrajectorySequence(centre6);
            sleep(1000);
            drive.followTrajectorySequence(centre7);
//            sleep(1000);
            sleep(700);
            drive.followTrajectorySequence(centre8);
            sleep(600);
            drive.followTrajectorySequence(centre9);
            sleep(600);
            schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
            drive.followTrajectorySequence(centre10);
            sleep(900);
            drive.followTrajectorySequence(centre11);
            sleep(900);
            drive.followTrajectorySequence(centre12);
            sleep(900);
            drive.followTrajectorySequence(centre13);
            sleep(900);
            drive.followTrajectorySequence(centre14);
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
            drive.followTrajectorySequence(right1);
            drive.followTrajectorySequence(right2);
            sleep(600);
            schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROPseUPAR));
            sleep(100);
            drive.followTrajectorySequence(right3);
            sleep(900);
            drive.followTrajectorySequence(right4);
            sleep(500);
            drive.followTrajectorySequence(right5);
            sleep(500);
            drive.followTrajectorySequence(right6);
            sleep(1000);
            drive.followTrajectorySequence(right7);
            sleep(700);
            drive.followTrajectorySequence(right8);
            sleep(600);
            drive.followTrajectorySequence(right9);
            sleep(600);
            schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
            drive.followTrajectorySequence(right10);
            sleep(900);
            drive.followTrajectorySequence(right11);
            sleep(900);
            drive.followTrajectorySequence(right12);
            sleep(900);
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




///////VALUES FOR STACK//////////////////
//FOR TOP 1 PIXEL
// rackVal = 0.91
//gripRotate = 0.268888888
// FOR TOP 2 PIXELS
//rackVal = 0.93
// gripRotate = 0.2688888