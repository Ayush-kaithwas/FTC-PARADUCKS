package org.firstinspires.ftc.teamcode.auto.BLUE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commandbase.instantCommands.gripRotateCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.gripperCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outArmCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outArmExtensionCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outGripperCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.rackCommand;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.intakeSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.outakeSubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Disabled
@Autonomous(group = "drive")
public class Blue_FAR_1white extends CommandOpMode{
    intakeSubsystem intake = null;
    outakeSubsystem outake = null;
    Servo rack;
    SampleMecanumDrive drive = null;

    @Override
    public void runOpMode() throws InterruptedException {
        rack = hardwareMap.get(Servo.class, "rack");
        intake = new intakeSubsystem(hardwareMap, telemetry);
        outake = new outakeSubsystem(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose2d startPose = new Pose2d(-31, 70, Math.toRadians(180));
        ////////////////////////////////////////////////////////////////////////////////////////////
        //TODO: RIGHT POSITIONS
        Pose2d startright1Pose = new Pose2d(-30, 60, Math.toRadians(0));
        Pose2d startright2Pose = new Pose2d(44, 35, Math.toRadians(0));
        Pose2d startright3Pose = new Pose2d(46, 35, Math.toRadians(0));
        ////////////////////////////////////////////////////////////////////////////////////////////
        //TODO:CENTRE POSITIONS
        Pose2d startcentre1Pose = new Pose2d(-33, 31, Math.toRadians(-0.000));
        Pose2d startcentre2Pose = new Pose2d(-37,31, Math.toRadians(-0.000001));
        Pose2d startcentre3Pose = new Pose2d(-44, 33.55, Math.toRadians(0));
        Pose2d startcentre4Pose = new Pose2d(-47.2, 33.55, Math.toRadians(0));
        Pose2d startcentre5Pose = new Pose2d(-38, 33.55, Math.toRadians(0));

        ////////////////////////////////////////////////////////////////////////////////////////////
        //TODO: LEFT POSITIONS
        Pose2d startleft1Pose = new Pose2d(-23.4,38, Math.toRadians(0));
        Pose2d startleft2Pose = new Pose2d(45,50, Math.toRadians(0));
        Pose2d startleft3Pose = new Pose2d(46,50, Math.toRadians(0));


        drive.setPoseEstimate(startPose);
        schedule(new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE));
        schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.AUTO)); // for Left only
//        schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK));

        schedule(new rackCommand(intake, intakeSubsystem.RackState.AUTO_TOP_PIXEL));
        schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.PICK));
        schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));


        waitForStart();

        if (isStopRequested()) return;
//TODO: LEFT CYCLE AUTO (WITHOUT WHITE PIXEL)
//TODO:LEFT1
        TrajectorySequence left1 = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-30, 36))
                .lineToConstantHeading(new Vector2d(-26, 36))
                .addDisplacementMarker(() -> {
                    schedule(new ParallelCommandGroup(
                            new rackCommand(intake, intakeSubsystem.RackState.AUTO_TOP_PIXEL),
                            new gripRotateCommand(intake, intakeSubsystem.GripRotateState.AUTO),
                            new gripperCommand(intake, intakeSubsystem.GripperState.OPEN),
                            new rackCommand(intake, intakeSubsystem.RackState.MID)

//                            new gripperCommand(intake, intakeSubsystem.GripperState.OPEN)
                    ));
                })
                .build();
        //TODO:LEFT2
        TrajectorySequence left2 = drive.trajectorySequenceBuilder(startleft1Pose)
                .lineToConstantHeading(new Vector2d(-33,36))
                .addDisplacementMarker(() -> {
                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
//                    new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND);
                })
                .lineToConstantHeading(new Vector2d(-24, 20))
                .lineToConstantHeading(new Vector2d(35,20))
                .lineToConstantHeading(new Vector2d(58, 35.8))
                .build();

        TrajectorySequence left3 = drive.trajectorySequenceBuilder(left2.end())
                .lineToConstantHeading(new Vector2d(66,50))
                .addDisplacementMarker(() -> {
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROP));
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
//                    new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND);
                })
                .build();
        //TODO:LEFT3
        TrajectorySequence left4 = drive.trajectorySequenceBuilder(left3.end())
                .lineToConstantHeading(new Vector2d(66.3,50))
                .addDisplacementMarker(() -> {
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPEN));
                })
                .build();
        //TODO:LEFT4
        TrajectorySequence left5 = drive.trajectorySequenceBuilder(left4.end())
                .lineToConstantHeading(new Vector2d(63,50))
                .addDisplacementMarker(() -> {
                    schedule(new ParallelCommandGroup(
                            (new outArmCommand(outake, outakeSubsystem.OutArmState.PICK)),
                            new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID)
                    ));
                })
                ///////////////////////////////////////////////////////////
                //TODO:PARK LEFT
//                .lineToConstantHeading(new Vector2d(40,65))
//                .lineToConstantHeading(new Vector2d(48.6,65))
                ///////////////////////////////////////////////////////////
//                ///////////////////////////////////////////////////////////
//                //TODO:PARK RIGHT
                .lineToConstantHeading(new Vector2d(60,20))
                .lineToConstantHeading(new Vector2d(69,20))
//                ///////////////////////////////////////////////////////////
                .build();







//////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //TODO: CENTRE CYCLE AUTO(WITH WHITE PIXEL)
        //TODO:CENTRE1
        TrajectorySequence centre1 = drive.trajectorySequenceBuilder(startPose)
//
                .lineToConstantHeading(new Vector2d(-43, 31))
                .lineToConstantHeading(new Vector2d(-33, 31))


                .addDisplacementMarker(() -> {
                    schedule(new ParallelCommandGroup(
                            new rackCommand(intake, intakeSubsystem.RackState.LOW),
                            new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK),
                            new gripperCommand(intake, intakeSubsystem.GripperState.OPEN),
                            new rackCommand(intake, intakeSubsystem.RackState.MID)

//                            new gripperCommand(intake, intakeSubsystem.GripperState.OPEN)
                    ));
                })
                .lineToConstantHeading(new Vector2d(-37, 31))

//                .turn(180)
//                .lineToConstantHeading(new Vector2d(-35,31))
//                .lineToLinearHeading(new Pose2d(-15, 31, Math.toRadians(0)))
                .build();
        //TODO:CENTRE2
        TrajectorySequence centre2 = drive.trajectorySequenceBuilder(startcentre2Pose)
//                .lineToLinearHeading(new Pose2d(-15, 31, Math.toRadians(-15)))
                .addDisplacementMarker(() -> {
                    schedule(new rackCommand(intake, intakeSubsystem.RackState.AUTO_TOP_PIXEL));
                })
                .lineToConstantHeading(new Vector2d(-38,33.55))
                .addDisplacementMarker(() -> {
                    schedule(new ParallelCommandGroup(
//                            new rackCommand(intake, intakeSubsystem.RackState.AUTO_TOP),
                            new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK),
                            new gripperCommand(intake, intakeSubsystem.GripperState.OpenAUTO)
//                            new rackCommand(intake, intakeSubsystem.RackState.TOP)

//                            new gripperCommand(intake, intakeSubsystem.GripperState.OPEN)
                    ));
                })
                .lineToConstantHeading(new Vector2d(-44,33.55))

//                .lineToConstantHeading(new Vector2d(-21,31))


                .build();
        //TODO:CENTRE3
        TrajectorySequence centre3  = drive.trajectorySequenceBuilder(startcentre3Pose)

                .lineToConstantHeading(new Vector2d(-47.2,33.55))
                .addDisplacementMarker(() -> {
//                   schedule( new SequentialCommandGroup(
                      schedule(      new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK));
//                            new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE)));
                })
//                .lineToConstantHeading(new Vector2d(-21,28))

                .build();
        //TODO:CENTRE4
        TrajectorySequence centre4  = drive.trajectorySequenceBuilder(startcentre4Pose)
                .lineToConstantHeading(new Vector2d(-48,33.55))


                .addDisplacementMarker(() -> {
//                    schedule(new pick_AUTO(intake,outake));
//                    new SequentialCommandGroup(
//                            new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK),
                    schedule(new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE));
//                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MIDseJYAADA));

                })
//                .lineToConstantHeading(new Vector2d(-20,32.8))
//                .lineToConstantHeading(new Vector2d(-8,20))


                .build();
        //TODO:CENTRE5
        TrajectorySequence centre5  = drive.trajectorySequenceBuilder(startcentre4Pose)

                .addDisplacementMarker(() -> {
                    schedule(new ParallelCommandGroup(
                        new rackCommand(intake, intakeSubsystem.RackState.TOPseUPAR),
                            new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPEN)
                    ));
                })

                .lineToConstantHeading(new Vector2d(-38,33.55))
//                .lineToConstantHeading(new Vector2d(-32,20))

                .build();
        //TODO:CENTRE6
        TrajectorySequence centre6  = drive.trajectorySequenceBuilder(centre5.end())
                .addDisplacementMarker(() -> {
                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MIDseJYAADA));
//                    schedule( new gripperCommand(intake, intakeSubsystem.GripperState.OPEN));

//                            new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE)));
                })

                .lineToConstantHeading(new Vector2d(-35,33.55))
                .addDisplacementMarker(() -> {
                    schedule( new gripperCommand(intake, intakeSubsystem.GripperState.OPEN));
                })

                .build();
        //TODO:CENTRE7
        TrajectorySequence centre7  = drive.trajectorySequenceBuilder(centre6.end())

                .addDisplacementMarker(() -> {
                   schedule( new ParallelCommandGroup(
                            new gripperCommand(intake, intakeSubsystem.GripperState.OPENUP),
                            new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID),
                            new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE),
                            new rackCommand(intake, intakeSubsystem.RackState.LOWseUPAR),
                            new rackCommand(intake, intakeSubsystem.RackState.LOW),
                            new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE),
                            new gripperCommand(intake, intakeSubsystem.GripperState.INIT)));
                })

                .lineToConstantHeading(new Vector2d(-19,17))
                .lineToConstantHeading(new Vector2d(35,20))
                .lineToConstantHeading(new Vector2d(58, 35.8))
                .build();
        //TODO:CENTRE8
        TrajectorySequence centre8 = drive.trajectorySequenceBuilder(centre7.end())

                .lineToConstantHeading(new Vector2d(66,42.9))
                .addDisplacementMarker(() -> {
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROPseUPAR));
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
//                    new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND);
                })
                .build();
        //TODO:CENTRE9
        TrajectorySequence centre9 = drive.trajectorySequenceBuilder(centre8.end())

                .lineToConstantHeading(new Vector2d(66.3,42.9))
                .addDisplacementMarker(() -> {;
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPEN));
                })
                .build();
        //TODO:CENTRE10
        TrajectorySequence centre10 = drive.trajectorySequenceBuilder(centre9.end())


                .addDisplacementMarker(() -> {
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.PICK));
                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
                })
                ///////////////////////////////////////////////////////////
//                //TODO:PARK LEFT
//                .lineToConstantHeading(new Vector2d(40,65))
//                .lineToConstantHeading(new Vector2d(48.6,65))
                ///////////////////////////////////////////////////////////
//                ///////////////////////////////////////////////////////////
                //TODO:PARK RIGHT
                .lineToConstantHeading(new Vector2d(60,20))
                .lineToConstantHeading(new Vector2d(69,20))
//                ///////////////////////////////////////////////////////////
                .addDisplacementMarker(() -> {
                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
                })
                .build();

//        //TODO:CENTRE8
//        TrajectorySequence centre8  = drive.trajectorySequenceBuilder(startcentre3Pose)
//
//                .lineToConstantHeading(new Vector2d(-19,23))
//                .lineToConstantHeading(new Vector2d(23,23))
//                .lineToConstantHeading(new Vector2d(43,40))
//
//
//                .build();
//        //TODO:CENTRE9
//        TrajectorySequence centre9 = drive.trajectorySequenceBuilder(startcentre1Pose)
//
//                .lineToConstantHeading(new Vector2d(42.5,42.9))
//                .addDisplacementMarker(() -> {
//                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROPseUPAR));
//                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
////                    new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND);
//                })
//                .build();
//        //TODO:CENTRE6
//        TrajectorySequence centre10 = drive.trajectorySequenceBuilder(startcentre2Pose)
//
//                .lineToConstantHeading(new Vector2d(42,42.9))
//                .addDisplacementMarker(() -> {;
//                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPEN));
//                })
//                .build();
//        //TODO:CENTRE7
//        TrajectorySequence centre11 = drive.trajectorySequenceBuilder(startcentre3Pose)
//
//
//                .addDisplacementMarker(() -> {
//                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.PICK));
//                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
//                })
//                ///////////////////////////////////////////////////////////
//                //TODO:PARK LEFT
//                .lineToConstantHeading(new Vector2d(40,68))
//                .lineToConstantHeading(new Vector2d(48.6,68))
//                ///////////////////////////////////////////////////////////
////                ///////////////////////////////////////////////////////////
////                //TODO:PARK RIGHT
////                .lineToConstantHeading(new Vector2d(40,9))
////                .lineToConstantHeading(new Vector2d(49,9))
////                ///////////////////////////////////////////////////////////
//                .addDisplacementMarker(() -> {
//                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
//                })
//                .build();


//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//       //TODO: RIGHT CYCLE AUTO(YELLOW FIRST)
//      //TODO:RIGHT1
//        TrajectorySequence right1 = drive.trajectorySequenceBuilder(startPose)
//                .lineToConstantHeading(new Vector2d(45,48.6))
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
//                .lineToConstantHeading(new Vector2d(46,48.6))
//                .addDisplacementMarker(() -> {
//                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPEN));
//                })
//                .build();
////        TODO:RIGHT3
//        TrajectorySequence right3 = drive.trajectorySequenceBuilder(startright3Pose)
//                .lineToConstantHeading(new Vector2d(45,40))
//                .addDisplacementMarker(() -> {
//                    schedule  (new outArmCommand(outake, outakeSubsystem.OutArmState.PICK));
//
//                })
//                .build();
////        TODO:RIGHT4
//        TrajectorySequence right4 = drive.trajectorySequenceBuilder(startright3Pose)
//
//                .lineToConstantHeading(new Vector2d(23.5,38))
//                .addDisplacementMarker(() -> {
//                    schedule(new ParallelCommandGroup(
//                            new rackCommand(intake, intakeSubsystem.RackState.LOW),
//                            new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK),
//                            new gripperCommand(intake, intakeSubsystem.GripperState.OPEN)
//                    ));
//                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK));
//                })
//                .build();

        //TODO: RIGHTAUTO2(PURPLE FIRST) (WITHOUT WHITE PIXEL)
        //TODO:RIGHT11
        TrajectorySequence right11 = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-30, 60))

//                .lineToConstantHeading(new Vector2d(1.5, 38))
//                .addDisplacementMarker(() -> {
//                    schedule(new ParallelCommandGroup(
////                            new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK),
//                            new rackCommand(intake, intakeSubsystem.RackState.LOW),
//                            new gripperCommand(intake, intakeSubsystem.GripperState.OPEN)
//                    ));
//                })
//
//                .lineToConstantHeading(new Vector2d(43, 35))
                .build();

        //TODO:RIGHT12
        TrajectorySequence right12 = drive.trajectorySequenceBuilder(startright1Pose)
                .lineToConstantHeading(new Vector2d(-26, 50))
                .lineToConstantHeading(new Vector2d(-24, 40))

//                .lineToConstantHeading(new Vector2d(-,38))
                .addDisplacementMarker(() -> {
                    schedule(new ParallelCommandGroup(
//                            new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK),
                            new rackCommand(intake, intakeSubsystem.RackState.LOW),
                            new gripperCommand(intake, intakeSubsystem.GripperState.OPEN)
//                            new rackCommand(intake, intakeSubsystem.RackState.AUTO_TOP)

                    ));
                })

                .lineToConstantHeading(new Vector2d(-23.5, 40))
                .addDisplacementMarker(() -> {
                    schedule(
                            new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
                })
                .lineToConstantHeading(new Vector2d(-24, 20))
                .lineToConstantHeading(new Vector2d(35,20))
                .lineToConstantHeading(new Vector2d(58, 35.8))
                .build();

        //TODO:RIGHT13
        TrajectorySequence right13 = drive.trajectorySequenceBuilder(right12.end())
                .lineToConstantHeading(new Vector2d(66,35.8))
                .addDisplacementMarker(() -> {
//                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROP));
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
                    new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND);
                })
                .build();

//        TODO:RIGHT14
        TrajectorySequence right14 = drive.trajectorySequenceBuilder(right13.end())

                .lineToConstantHeading(new Vector2d(66.3,35.8))
                .addDisplacementMarker(() -> {
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPEN));
                })
                .build();

//        TODO:RIGHT15
        TrajectorySequence right15 = drive.trajectorySequenceBuilder(right14.end())
                .addDisplacementMarker(() -> {
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.PICK));
                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
                })
                ///////////////////////////////////////////////////////////
                //TODO:PARK LEFT
//                .lineToConstantHeading(new Vector2d(40,65))
//                .lineToConstantHeading(new Vector2d(48.6,65))
                ///////////////////////////////////////////////////////////
//                ///////////////////////////////////////////////////////////
//                //TODO:PARK RIGHT
                .lineToConstantHeading(new Vector2d(60,20))
                .lineToConstantHeading(new Vector2d(69,20))
//                ///////////////////////////////////////////////////////////
                .addDisplacementMarker(() -> {
                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
                })
                .build();

//        //TODO:LEFT
//
//        drive.followTrajectorySequence(left1);
//        drive.followTrajectorySequence(left2);
//        sleep(200);
//        drive.followTrajectorySequence(left3);
//        sleep(300);
//        drive.followTrajectorySequence(left4);

//        //TODO:CENTRE
//        drive.followTrajectorySequence(centre1);
//        sleep(200);
//        drive.followTrajectorySequence(centre2);
//        sleep(500);
//        drive.followTrajectorySequence(centre3);
//        sleep(200);
//        drive.followTrajectorySequence(centre4);
//        sleep(1000);
//        drive.followTrajectorySequence(centre5);
//        sleep(500);
//        drive.followTrajectorySequence(centre6);
//        sleep(500);
//        drive.followTrajectorySequence(centre7);
//        sleep(1000);
//        drive.followTrajectorySequence(centre8);
//        sleep(500);
//        drive.followTrajectorySequence(centre9);

        //TODO:RIGHT(YELLOW FIRST)

//        drive.followTrajectorySequence(right1);
//        sleep(300);
//        drive.followTrajectorySequence(right2);
//        sleep(200);
//        drive.followTrajectorySequence(right3);
//        sleep(200);
//        drive.followTrajectorySequence(right4);

//        //TODO RIGHT (PURPLE FIRST)
//        drive.followTrajectorySequence(right11);
//        sleep(100);
//        drive.followTrajectorySequence(right12);
//        sleep(600);
//        drive.followTrajectorySequence(right13);
//        sleep(400);
//        drive.followTrajectorySequence(right14);
//        sleep(600);
//        drive.followTrajectorySequence(right15);




    }

    @Override
    public void initialize () {

    }
}


