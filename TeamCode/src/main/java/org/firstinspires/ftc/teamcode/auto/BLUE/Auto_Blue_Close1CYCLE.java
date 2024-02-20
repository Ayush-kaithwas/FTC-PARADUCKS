package org.firstinspires.ftc.teamcode.auto.BLUE;

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
@Autonomous(group = "Auto_blueClose")
//@Autonomous(group = "drive")
public class Auto_Blue_Close1CYCLE extends CommandOpMode{
    intakeSubsystem intake = null;
    outakeSubsystem outake = null;
    HangerAndDrone endgame = null;
    Servo rack;
    SampleMecanumDrive drive = null;
    String marker = "RIGHT";
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

        Pose2d startPose = new Pose2d(7, 70, Math.toRadians(0));
        Pose2d startright1Pose = new Pose2d(43, 35, Math.toRadians(0));
        Pose2d startright2Pose = new Pose2d(46, 35, Math.toRadians(0));
        Pose2d startright3Pose = new Pose2d(46.5, 35, Math.toRadians(0));
        Pose2d startcentre1Pose = new Pose2d(41, 42.9, Math.toRadians(0));
        Pose2d startcentre2Pose = new Pose2d(42.7, 42.9, Math.toRadians(0));
        Pose2d startcentre3Pose = new Pose2d(42, 42.9, Math.toRadians(0));
        Pose2d startleft1Pose = new Pose2d(23.5,38, Math.toRadians(0));
        Pose2d startleft2Pose = new Pose2d(45,50, Math.toRadians(0));
        Pose2d startleft3Pose = new Pose2d(46,50, Math.toRadians(0));


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
                .lineToConstantHeading(new Vector2d(25.3,38))
                .addDisplacementMarker(() -> {
                    schedule(new ParallelCommandGroup(
                            new rackCommand(intake, intakeSubsystem.RackState.LOW),
                            new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK),
                            new gripperCommand(intake, intakeSubsystem.GripperState.OPEN)
                    ));
                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK));
                })
                .build();
        //TODO:LEFT2
        TrajectorySequence left2 = drive.trajectorySequenceBuilder(left1.end())
                .lineToConstantHeading(new Vector2d(38,50))
                .addDisplacementMarker(() -> {
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROP));
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
//                    new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND);
                })
                .build();
        //TODO:LEFT3
        TrajectorySequence left3 = drive.trajectorySequenceBuilder(left2.end())
                .lineToConstantHeading(new Vector2d(42.5,50))
                .addDisplacementMarker(() -> {
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPEN));
                })
                .build();
        //TODO:LEFT4
        TrajectorySequence left4 = drive.trajectorySequenceBuilder(left3.end())
                .lineToConstantHeading(new Vector2d(37,50))
                .addDisplacementMarker(() -> {
                    schedule(new ParallelCommandGroup(
                            (new outArmCommand(outake, outakeSubsystem.OutArmState.PICK)),
                            new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID)
                    ));
                })
                ///////////////////////////////////////////////////////////
                //TODO:PARK LEFT
                .lineToConstantHeading(new Vector2d(37,68))
                .lineToConstantHeading(new Vector2d(48.6,68))
                ///////////////////////////////////////////////////////////
//                ///////////////////////////////////////////////////////////
//                //TODO:PARK RIGHT
//                .lineToConstantHeading(new Vector2d(40,9))
//                .lineToConstantHeading(new Vector2d(49,9))
//                ///////////////////////////////////////////////////////////
                .build();



        //TODO: CENTRE CYCLE AUTO (done)
        //TODO:CENTRE1
        TrajectorySequence centre1 = drive.trajectorySequenceBuilder(startPose)
//
                .lineToConstantHeading(new Vector2d(19, 31))
                .addDisplacementMarker(() -> {
                    schedule(new ParallelCommandGroup(
                            new rackCommand(intake, intakeSubsystem.RackState.LOW),
                            new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK),
                            new gripperCommand(intake, intakeSubsystem.GripperState.OPEN)
//                            new gripperCommand(intake, intakeSubsystem.GripperState.OPEN)
                    ));
                })
//                .lineToConstantHeading(new Vector2d(41, 42.9))

                .build();
        //TODO:CENTRE2
        TrajectorySequence centre2 = drive.trajectorySequenceBuilder(centre1.end())

                .lineToConstantHeading(new Vector2d(39.5,43.4))//41.4
                .addDisplacementMarker(() -> {
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROP));
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
//                    new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND);
                })
                .build();
        //TODO:CENTRE3
        TrajectorySequence centre3 = drive.trajectorySequenceBuilder(centre2.end())

                .lineToConstantHeading(new Vector2d(42,43.4))
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
                .lineToConstantHeading(new Vector2d(15,15))
//                .lineToConstantHeading(new Vector2d(-60,15))
                .lineToConstantHeading(new Vector2d(-60,21.4))

                .build();
        //TODO:CENTRE5
        TrajectorySequence centre5 = drive.trajectorySequenceBuilder(centre4.end())

                .addDisplacementMarker(() -> {
                    schedule(new ParallelCommandGroup(
                            new gripRotateCommand(intake, intakeSubsystem.GripRotateState.AUTO),
                            new rackCommand(intake, intakeSubsystem.RackState.AUTO_TOP2),
                            new gripperCommand(intake, intakeSubsystem.GripperState.OPEN)));
                })
                .lineToConstantHeading(new Vector2d(-70.8,21.4))
//                .addDisplacementMarker(() -> {
//                    schedule(new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE));
//                })
//                .lineToConstantHeading(new Vector2d(-60,15))
//                .lineToConstantHeading(new Vector2d(20,15))
//                .lineToConstantHeading(new Vector2d(39.5,41.4))

                .build();
        //TODO:CENTRE6
        TrajectorySequence centre6 = drive.trajectorySequenceBuilder(centre5.end())

                .lineToConstantHeading(new Vector2d(-70.7,21.4))
                .addDisplacementMarker(() -> {
                    schedule(new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE));

                })
//                .lineToConstantHeading(new Vector2d(-60,21.8))

                .build();
        //TODO:CENTRE7
        TrajectorySequence centre7 = drive.trajectorySequenceBuilder(centre6.end())

                .addDisplacementMarker(() -> {
                    new ParallelCommandGroup(//new outArmExtensionCommand(outake,outakeSubsystem.OutArmExtensionState.PIXPICK),
                            new outArmCommand(outake, outakeSubsystem.OutArmState.PICK),
                            new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPEN),
                            new gripRotateCommand(intake,intakeSubsystem.GripRotateState.MID),
                            new rackCommand(intake, intakeSubsystem.RackState.TOPseUPAR));

                })
                .lineToConstantHeading(new Vector2d(-65,21.4))

//                .lineToConstantHeading(new Vector2d(-60,21.8))

                .build();
        //TODO:CENTRE8
        TrajectorySequence centre8 = drive.trajectorySequenceBuilder(centre7.end())

                .addDisplacementMarker(() -> {
                    new ParallelCommandGroup(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.DROP),
                            new rackCommand(intake, intakeSubsystem.RackState.TOP));

                })
                .lineToConstantHeading(new Vector2d(-64.9,21.4))
                .addDisplacementMarker(() -> {
                    schedule(new gripperCommand(intake, intakeSubsystem.GripperState.OPENUP));
                    schedule(new ParallelCommandGroup(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MIDseJYAADA),
                            new gripperCommand(intake, intakeSubsystem.GripperState.OPEN),
                            new gripperCommand(intake, intakeSubsystem.GripperState.OPENUP_BACK)));
                })

                .build();
        //TODO:CENTRE9
        TrajectorySequence centre9 = drive.trajectorySequenceBuilder(centre8.end())
                .addDisplacementMarker(() -> {
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));//new
                })

                .lineToConstantHeading(new Vector2d(-60,15))
                .lineToConstantHeading(new Vector2d(20,15))
                .lineToConstantHeading(new Vector2d(39.5,41.4))
//                .lineToConstantHeading(new Vector2d(40.5,41.4))
                .addDisplacementMarker(() -> {
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROP));
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
                })
                .build();

        //TODO: CENTRE8
        TrajectorySequence centre10= drive.trajectorySequenceBuilder(centre9.end())
                .addDisplacementMarker(() -> {
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPENRIGHT));
                })
                .lineToConstantHeading(new Vector2d(42,40))
                .addDisplacementMarker(() -> {
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPENLEFT));
                })
                .build();


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

        //TODO: RIGHTAUTO2(PURPLE FIRST)
        //TODO:RIGHT11
        TrajectorySequence right11 = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(8, 38))
                .lineToConstantHeading(new Vector2d(1.5, 38))
                .addDisplacementMarker(() -> {
                    schedule(new ParallelCommandGroup(
//                            new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK),
                            new rackCommand(intake, intakeSubsystem.RackState.LOW),
                            new gripperCommand(intake, intakeSubsystem.GripperState.OPEN)
                    ));
                })

//                .lineToConstantHeading(new Vector2d(43, 35))
                .build();

        //TODO:RIGHT12
        TrajectorySequence right12 = drive.trajectorySequenceBuilder(right11.end())
                .lineToConstantHeading(new Vector2d(38,38.5))//37.5//35
                .addDisplacementMarker(() -> {
//                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROP));
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
//                    new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND);
                })
                .build();

//        TODO:RIGHT13
        TrajectorySequence right13 = drive.trajectorySequenceBuilder(right12.end())

                .lineToConstantHeading(new Vector2d(41.5,38.5))
                .addDisplacementMarker(() -> {
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPEN));
                })
                .build();

//        TODO:RIGHT14
        TrajectorySequence right14 = drive.trajectorySequenceBuilder(right13.end())
                .addDisplacementMarker(() -> {
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.PICK));
                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
                })
                .lineToConstantHeading(new Vector2d(37,38.5))

                ///////////////////////////////////////////////////////////
                //TODO:PARK LEFT
                .lineToConstantHeading(new Vector2d(37,68))
                .lineToConstantHeading(new Vector2d(48.6,68))
                ///////////////////////////////////////////////////////////
//                ///////////////////////////////////////////////////////////
//                //TODO:PARK RIGHT
//                .lineToConstantHeading(new Vector2d(40,9))
//                .lineToConstantHeading(new Vector2d(49,9))
//                ///////////////////////////////////////////////////////////
                .addDisplacementMarker(() -> {
                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
                })
                .build();



//        //TODO:LEFT
//
        if(marker=="LEFT"|| randompos==1) {
            drive.followTrajectorySequence(left1);
            drive.followTrajectorySequence(left2);
            sleep(600);
            schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROPseUPAR));
            sleep(100);
            drive.followTrajectorySequence(left3);
            sleep(800);
            drive.followTrajectorySequence(left4);
            sleep(999999);

        }

//        //TODO:CENTRE
        if(marker=="CENTRE"|| randompos==0) {
            drive.followTrajectorySequence(centre1);
            drive.followTrajectorySequence(centre2);
            sleep(600);
            schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROPseUPAR));
            sleep(200);
            drive.followTrajectorySequence(centre3);
            sleep(800);
            drive.followTrajectorySequence(centre4);
            sleep(400);
            drive.followTrajectorySequence(centre5);
            sleep(600);
            drive.followTrajectorySequence(centre6);
            sleep(900);
            drive.followTrajectorySequence(centre7);
            sleep(900);
            drive.followTrajectorySequence(centre8);
            sleep(600);
            drive.followTrajectorySequence(centre9);
            sleep(600);
            drive.followTrajectorySequence(centre10);
            sleep(600);



            sleep(999999);


        }

        //TODO:RIGHT(YELLOW FIRST)

//        drive.followTrajectorySequence(right1);
//        sleep(300);
//        drive.followTrajectorySequence(right2);
//        sleep(200);
//        drive.followTrajectorySequence(right3);
//        sleep(200);
//        drive.followTrajectorySequence(right4);

        //TODO RIGHT (PURPLE FIRST)
        if(marker=="RIGHT"|| randompos==2){
        drive.followTrajectorySequence(right11);
        drive.followTrajectorySequence(right12);
        sleep(600);
        schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROPseUPAR));
        sleep(100);
        drive.followTrajectorySequence(right13);
        sleep(800);
        drive.followTrajectorySequence(right14);
        sleep(999999);

        }


    }

    @Override
    public void initialize () {

    }
}


