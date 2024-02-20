package org.firstinspires.ftc.teamcode.auto.BLUE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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
@Config
@Autonomous(group = "drive")
public class Blue_FAR_50pt extends CommandOpMode{
    intakeSubsystem intake = null;
    outakeSubsystem outake = null;
    Servo rack;
    SampleMecanumDrive drive = null;
    public static String marker ="RIGHT";
    public static int randompos = 2;

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
        //TODO:CENTRE POSITIONS(1 WHITE PIXEL)
        Pose2d startcentre1Pose = new Pose2d(-33, 31, Math.toRadians(-0.000));
        Pose2d startcentre2Pose = new Pose2d(-37,31, Math.toRadians(-0.000001));
        Pose2d startcentre3Pose = new Pose2d(-44, 33.55, Math.toRadians(0));
        Pose2d startcentre4Pose = new Pose2d(-47.2, 33.55, Math.toRadians(0));
        Pose2d startcentre5Pose = new Pose2d(-38, 33.55, Math.toRadians(0));

        ////////////////////////////////////////////////////////////////////////////////////////////
        //TODO:CENTRE POSITIONS
        Pose2d NWstartcentre1Pose = new Pose2d(-37, 31, Math.toRadians(-0.000));
//        Pose2d startcentre2Pose = new Pose2d(-37,31, Math.toRadians(-0.000001));
//        Pose2d startcentre3Pose = new Pose2d(-44, 33.55, Math.toRadians(0));
//        Pose2d startcentre4Pose = new Pose2d(-47.2, 33.55, Math.toRadians(0));
//        Pose2d startcentre5Pose = new Pose2d(-38, 33.55, Math.toRadians(0));

        ////////////////////////////////////////////////////////////////////////////////////////////
        //TODO: LEFT POSITIONS
        Pose2d startleft1Pose = new Pose2d(-32,36, Math.toRadians(0));
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
                            new gripperCommand(intake, intakeSubsystem.GripperState.OPEN)
//                            new rackCommand(intake, intakeSubsystem.RackState.MID)

//                            new gripperCommand(intake, intakeSubsystem.GripperState.OPEN)
                    ));
                })
                .lineToConstantHeading(new Vector2d(-32, 36))

                .build();
        //TODO:LEFT2
        TrajectorySequence left2 = drive.trajectorySequenceBuilder(startleft1Pose)
                .lineToConstantHeading(new Vector2d(-30,36))
                .addDisplacementMarker(() -> {
                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
//                    new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND);
                })
                .lineToConstantHeading(new Vector2d(-24, 20))
                .lineToConstantHeading(new Vector2d(45,20))
//                .lineToConstantHeading(new Vector2d(55, 35.8))
                .build();

        TrajectorySequence left3 = drive.trajectorySequenceBuilder(left2.end())
                .lineToConstantHeading(new Vector2d(59.4,48.35))
                .addDisplacementMarker(() -> {
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROP));
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
//                    new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND);
                })
                .build();
        //TODO:LEFT3
        TrajectorySequence left4 = drive.trajectorySequenceBuilder(left3.end())
                .lineToConstantHeading(new Vector2d(61.3,48.35))
                .addDisplacementMarker(() -> {
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPEN));
                })
                .build();
        //TODO:LEFT4
        TrajectorySequence left5 = drive.trajectorySequenceBuilder(left4.end())
                .lineToConstantHeading(new Vector2d(56,45.35))
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
                .lineToConstantHeading(new Vector2d(56,20))
                .lineToConstantHeading(new Vector2d(69.5,20))
//                ///////////////////////////////////////////////////////////
                .build();







//////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //TODO: CENTRE CYCLE AUTO(NO WHITE PIXEL)
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
                            new rackCommand(intake, intakeSubsystem.RackState.LOW)
//                            new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID)


//                            new gripperCommand(intake, intakeSubsystem.GripperState.OPEN)
                    ));
                })
                .lineToConstantHeading(new Vector2d(-40, 31))
                .build();
        //TODO:CENTRE2
        TrajectorySequence centre2 = drive.trajectorySequenceBuilder(NWstartcentre1Pose)
//                .lineToLinearHeading(new Pose2d(-15, 31, Math.toRadians(-15)))
                .lineToConstantHeading(new Vector2d(-32,20))
                .lineToConstantHeading(new Vector2d(35,20))
//                .lineToConstantHeading(new Vector2d(58, 35.8))

//                .lineToConstantHeading(new Vector2d(-21,31))
                .build();
        //TODO:CENTRE3
        TrajectorySequence centre3  = drive.trajectorySequenceBuilder(centre2.end())
                .lineToConstantHeading(new Vector2d(59.5,41.48)) //41.48
                .addDisplacementMarker(() -> {
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROP));
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
//                    new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND);
                })
                .build();
        //TODO:CENTRE4
        TrajectorySequence centre4  = drive.trajectorySequenceBuilder(centre3.end())
                .lineToConstantHeading(new Vector2d(62.9,41.48))
                .addDisplacementMarker(() -> {;
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPEN));
                })
//                .lineToConstantHeading(new Vector2d(-20,32.8))
//                .lineToConstantHeading(new Vector2d(-8,20))
                .build();
        //TODO:CENTRE5
        TrajectorySequence centre5  = drive.trajectorySequenceBuilder(centre4.end())

                .lineToConstantHeading(new Vector2d(60,41.48))
                .addDisplacementMarker(() -> {
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.PICK));
                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
                })
                ///////////////////////////////////////////////////////////
                //TODO:PARK LEFT
//                .lineToConstantHeading(new Vector2d(40,68))
//                .lineToConstantHeading(new Vector2d(48.6,68))
                ///////////////////////////////////////////////////////////
//                ///////////////////////////////////////////////////////////
//                //TODO:PARK RIGHT
                .lineToConstantHeading(new Vector2d(60,20))
                .lineToConstantHeading(new Vector2d(71,20))
//                ///////////////////////////////////////////////////////////
                .addDisplacementMarker(() -> {
                    schedule(new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE));
                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MIDseJYAADA));
                })
                .build();


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

                .lineToConstantHeading(new Vector2d(-22.5, 40))
                .addDisplacementMarker(() -> {
                    schedule(
                            new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
                })
                .lineToConstantHeading(new Vector2d(-24, 18))
                .lineToConstantHeading(new Vector2d(35,18))
//                .lineToConstantHeading(new Vector2d(58, 35.4))
                .build();

        //TODO:RIGHT13
        TrajectorySequence right13 = drive.trajectorySequenceBuilder(right12.end())
                .lineToConstantHeading(new Vector2d(57,33.89))//35.43
                .addDisplacementMarker(() -> {
//                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROP));
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
//                    new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND);
                })
                .build();

//        TODO:RIGHT14
        TrajectorySequence right14 = drive.trajectorySequenceBuilder(right13.end())

                .lineToConstantHeading(new Vector2d(61.7,33.89))
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
        if(marker=="LEFT"|| randompos==1) {
            drive.followTrajectorySequence(left1);
            drive.followTrajectorySequence(left2);
            sleep(4000);    /////////////VARIES ACC TO ALLIANCE
            drive.followTrajectorySequence(left3);
            sleep(600);
            schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.BFDROPCENTREAUTO));
            sleep(100);
            drive.followTrajectorySequence(left4);
            sleep(1000);
            drive.followTrajectorySequence(left5);
            sleep(999999);
        }
        if(marker=="CENTRE"|| randompos==0) {
            //TODO:CENTRE(NO WHITE)
            drive.followTrajectorySequence(centre1);
            sleep(200);
            schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
            sleep(200);
            drive.followTrajectorySequence(centre2);
//            sleep(200);
            sleep(4000);    /////////////VARIES ACC TO ALLIANCE
            drive.followTrajectorySequence(centre3);
            sleep(600);
            schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.BFDROPCENTREAUTO));
            sleep(200);
            drive.followTrajectorySequence(centre4);
            sleep(1000);
            drive.followTrajectorySequence(centre5);
            sleep(999999);

        }


        //TODO:RIGHT(YELLOW FIRST)

//        drive.followTrajectorySequence(right1);
//        sleep(300);
//        drive.followTrajectorySequence(right2);
//        sleep(200);
//        drive.followTrajectorySequence(right3);
//        sleep(200);
//        drive.followTrajectorySequence(right4);.

//        //TODO RIGHT (PURPLE FIRST)
        if (marker=="RIGHT"|| randompos==2) {
        drive.followTrajectorySequence(right11);
        sleep(100);
        drive.followTrajectorySequence(right12);
//        sleep(300);
        sleep(4000);    /////////////VARIES ACC TO ALLIANCE
        drive.followTrajectorySequence(right13);
        sleep(600);
        schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROPseUPAR));
        sleep(200);
        drive.followTrajectorySequence(right14);
        sleep(1000);
        drive.followTrajectorySequence(right15);
            sleep(999999);

        }




    }

    @Override
    public void initialize () {

    }
}


