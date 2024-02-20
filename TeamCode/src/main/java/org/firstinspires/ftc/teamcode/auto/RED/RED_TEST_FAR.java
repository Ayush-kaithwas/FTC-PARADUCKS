package org.firstinspires.ftc.teamcode.auto.RED;

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

import org.firstinspires.ftc.teamcode.commandbase.autoCommands.DepositCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.gripRotateCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.gripperCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outArmCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outGripperCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.rackCommand;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.intakeSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.outakeSubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Disabled
@Config
@Autonomous(group = "RED_FAR-TEST")
public class RED_TEST_FAR extends CommandOpMode{
    intakeSubsystem intake = null;
    outakeSubsystem outake = null;
    Servo rack;
    SampleMecanumDrive drive = null;
    public static String marker = "LEFT";
    public static int randompos = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        rack = hardwareMap.get(Servo.class, "rack");
        intake = new intakeSubsystem(hardwareMap, telemetry);
        outake = new outakeSubsystem(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose2d startPose = new Pose2d(-31, -70, Math.toRadians(180));
        ////////////////////////////////////////////////////////////////////////////////////////////
        //TODO: RIGHT POSITIONS
        Pose2d startleft1Pose = new Pose2d(-30, -60, Math.toRadians(0));
        Pose2d startright2Pose = new Pose2d(44, -35, Math.toRadians(0));
        Pose2d startright3Pose = new Pose2d(46, -35, Math.toRadians(0));
        ////////////////////////////////////////////////////////////////////////////////////////////
        //TODO:CENTRE POSITIONS(1 WHITE PIXEL)
        Pose2d startcentre1Pose = new Pose2d(-33, -31, Math.toRadians(-0.000));
        Pose2d startcentre2Pose = new Pose2d(-37, -31, Math.toRadians(-0.000001));
        Pose2d startcentre3Pose = new Pose2d(-44, -33.55, Math.toRadians(0));
        Pose2d startcentre4Pose = new Pose2d(-47.2, -33.55, Math.toRadians(0));
        Pose2d startcentre5Pose = new Pose2d(-38, -33.55, Math.toRadians(0));

        ////////////////////////////////////////////////////////////////////////////////////////////
        //TODO:CENTRE POSITIONS
        Pose2d NWstartcentre1Pose = new Pose2d(-40, -31, Math.toRadians(-0.000));
//        Pose2d startcentre2Pose = new Pose2d(-37,31, Math.toRadians(-0.000001));
//        Pose2d startcentre3Pose = new Pose2d(-44, 33.55, Math.toRadians(0));
//        Pose2d startcentre4Pose = new Pose2d(-47.2, 33.55, Math.toRadians(0));
//        Pose2d startcentre5Pose = new Pose2d(-38, 33.55, Math.toRadians(0));

        ////////////////////////////////////////////////////////////////////////////////////////////
        //TODO: LEFT POSITIONS
        Pose2d startright1Pose = new Pose2d(-23.4, -38, Math.toRadians(0));
        Pose2d startleft2Pose = new Pose2d(45, -50, Math.toRadians(0));
        Pose2d startleft3Pose = new Pose2d(46, -50, Math.toRadians(0));


        drive.setPoseEstimate(startPose);
        schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.PICK));
        schedule(new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE));
        schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID)); // for Left only
//        schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK));

        schedule(new rackCommand(intake, intakeSubsystem.RackState.LOWseUPAR));
        schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));

        waitForStart();
        if (isStopRequested()) return;

//TODO: RIGHT CYCLE AUTO (WITHOUT WHITE PIXEL)

        //RIGHT0
        TrajectorySequence right0 = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    schedule(new ParallelCommandGroup(

                            new outArmCommand(outake, outakeSubsystem.OutArmState.PICK),
                            new rackCommand(intake, intakeSubsystem.RackState.LOWseUPAR),
                            new gripRotateCommand(intake, intakeSubsystem.GripRotateState.AUTO),
                            new gripperCommand(intake, intakeSubsystem.GripperState.OPEN)
//                            new rackCommand(intake, intakeSubsystem.RackState.MID)

//                            new gripperCommand(intake, intakeSubsystem.GripperState.OPEN)
                    ));
                })
                .build();
//TODO:RIGHT1
        TrajectorySequence right1 = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-30, -36))
                .lineToConstantHeading(new Vector2d(-26, -36))
                .addDisplacementMarker(() -> {
                    schedule(new ParallelCommandGroup(
                            new outArmCommand(outake, outakeSubsystem.OutArmState.PICK),
                    new rackCommand(intake, intakeSubsystem.RackState.LOWseUPAR),
                            new gripRotateCommand(intake, intakeSubsystem.GripRotateState.AUTO),
                            new gripperCommand(intake, intakeSubsystem.GripperState.OPEN)
//                            new rackCommand(intake, intakeSubsystem.RackState.MID)

//                            new gripperCommand(intake, intakeSubsystem.GripperState.OPEN)
                    ));
                })
                .lineToConstantHeading(new Vector2d(-32, -36))

                .build();
        //TODO:RIGHT2
        TrajectorySequence right2 = drive.trajectorySequenceBuilder(startright1Pose)
                .lineToConstantHeading(new Vector2d(-33, -36))
                .addDisplacementMarker(() -> {
                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
//                    new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND);
                })
                .lineToConstantHeading(new Vector2d(-24, -18.5))
                .lineToConstantHeading(new Vector2d(45, -18.5))
//                .lineToConstantHeading(new Vector2d(60, -35.8))
                .build();
//TODO:RIGHT3
        TrajectorySequence right3 = drive.trajectorySequenceBuilder(right2.end())
                .lineToConstantHeading(new Vector2d(58, -48.43))//48.3 //-48
                .addDisplacementMarker(() -> {
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROP));
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
//                    new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND);
                })
                .build();
        //TODO:RIGHT4
        TrajectorySequence right4 = drive.trajectorySequenceBuilder(right3.end())
                .lineToConstantHeading(new Vector2d(62.5, -48.43))
                .addDisplacementMarker(() -> {
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPEN));
                })
                .build();
        //TODO:RIGHT4
        TrajectorySequence right5 = drive.trajectorySequenceBuilder(right4.end())
                .lineToConstantHeading(new Vector2d(56, -48.43))
                .addDisplacementMarker(() -> {
                    schedule(new ParallelCommandGroup(
                            (new outArmCommand(outake, outakeSubsystem.OutArmState.PICK)),
                            new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID)
                    ));
                })
                ///////////////////////////////////////////////////////////
                //TODO:PARK LEFT
                .lineToConstantHeading(new Vector2d(56, -17))
                .lineToConstantHeading(new Vector2d(69, -17))
                ///////////////////////////////////////////////////////////
//                ///////////////////////////////////////////////////////////
//                //TODO:PARK RIGHT
//                .lineToConstantHeading(new Vector2d(60,20))
//                .lineToConstantHeading(new Vector2d(69,20))
//                ///////////////////////////////////////////////////////////
                .build();


////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //TODO: CENTRE CYCLE AUTO(NO WHITE PIXEL)
        //TODO:CENTRE1

        TrajectorySequence centre1 = drive.trajectorySequenceBuilder(startPose)
//
                .lineToConstantHeading(new Vector2d(-43, -32))
                .lineToConstantHeading(new Vector2d(-33, -32))


                .addDisplacementMarker(() -> {
                    schedule(new ParallelCommandGroup(
                            new rackCommand(intake, intakeSubsystem.RackState.LOW),
                            new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK), // for Left only

                            new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK),
                            new gripperCommand(intake, intakeSubsystem.GripperState.OPEN),
                            new rackCommand(intake, intakeSubsystem.RackState.MID)
//                            new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID)


//                            new gripperCommand(intake, intakeSubsystem.GripperState.OPEN)
                    ));
                })
                .lineToConstantHeading(new Vector2d(-42, -31))
                .build();
        //TODO:CENTRE2
        TrajectorySequence centre2 = drive.trajectorySequenceBuilder(NWstartcentre1Pose)
//                .lineToLinearHeading(new Pose2d(-15, 31, Math.toRadians(-15)))
                .lineToConstantHeading(new Vector2d(-30, -17))
                .lineToConstantHeading(new Vector2d(35, -17))
//                .lineToConstantHeading(new Vector2d(-21,31))
                .build();
        //TODO:CENTRE3
        TrajectorySequence centre3 = drive.trajectorySequenceBuilder(centre2.end())
//                .lineToConstantHeading(new Vector2d(58, -42.1))//-35.8
                .lineToConstantHeading(new Vector2d(61.8, -42.1))//62 //-41.9
                .addDisplacementMarker(() -> {
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROP));
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
//                    new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND);
                })
                .build();
        //TODO:CENTRE4
        TrajectorySequence centre4 = drive.trajectorySequenceBuilder(centre3.end())
                .lineToConstantHeading(new Vector2d(63, -42.1))//62.6
                .addDisplacementMarker(() -> {
                    ;
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPEN));
                })
//                .lineToConstantHeading(new Vector2d(-20,32.8))
//                .lineToConstantHeading(new Vector2d(-8,20))
                .build();
        //TODO:CENTRE5
        TrajectorySequence centre5 = drive.trajectorySequenceBuilder(centre4.end())

                .lineToConstantHeading(new Vector2d(60, -42.1))
                .addDisplacementMarker(() -> {
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.PICK));
                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
                })
                ///////////////////////////////////////////////////////////
                //TODO:PARK LEFT
                .lineToConstantHeading(new Vector2d(60, -17))
                .lineToConstantHeading(new Vector2d(70, -17))
                ///////////////////////////////////////////////////////////
//                ///////////////////////////////////////////////////////////
////                //TODO:PARK RIGHT
//                .lineToConstantHeading(new Vector2d(60,-20))
//                .lineToConstantHeading(new Vector2d(71,-20))
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
//        schedule(new DepositCommand(intake,outake));
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

        //TODO: LEFT AUTO2(PURPLE FIRST) (WITHOUT WHITE PIXEL)
        //TODO:LEFT1

        TrajectorySequence left1 = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-30, -60))
                .build();

        //TODO:LEFT2
        TrajectorySequence left2 = drive.trajectorySequenceBuilder(startleft1Pose)
                .lineToConstantHeading(new Vector2d(-26, -50))
                .lineToConstantHeading(new Vector2d(-25.3, -40))

//                .lineToConstantHeading(new Vector2d(-,38))
                .addDisplacementMarker(() -> {
                    schedule(new ParallelCommandGroup(
                            new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK), // for Left only

//                            new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK),
                            new rackCommand(intake, intakeSubsystem.RackState.LOWseUPAR),
                            new gripperCommand(intake, intakeSubsystem.GripperState.OPEN)
//                            new rackCommand(intake, intakeSubsystem.RackState.AUTO_TOP)

                    ));
                })

                .lineToConstantHeading(new Vector2d(-23.5, -37))
                .addDisplacementMarker(() -> {
                    schedule(
                            new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
                })
                .lineToConstantHeading(new Vector2d(-24, -17))
                .lineToConstantHeading(new Vector2d(35, -17))
//                    .lineToConstantHeading(new Vector2d(55, -35.8))
                .build();

        //TODO:LEFT3
        TrajectorySequence left3 = drive.trajectorySequenceBuilder(left2.end())//66.5
                .lineToConstantHeading(new Vector2d(55, -35.8))
                .addDisplacementMarker(() -> {
//                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROP));
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
//                        new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND);
                })
                .build();

//        TODO:LEFT4
        TrajectorySequence left4 = drive.trajectorySequenceBuilder(left3.end())

                .lineToConstantHeading(new Vector2d(61.9, -35.8))//60.5
                .addDisplacementMarker(() -> {
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPEN));
                })
                .build();

//        TODO:LEFT5
        TrajectorySequence left5 = drive.trajectorySequenceBuilder(left4.end())
                .lineToConstantHeading(new Vector2d(57, -35.8))

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
                .lineToConstantHeading(new Vector2d(57, -19))
                .lineToConstantHeading(new Vector2d(69, -19))
//                ///////////////////////////////////////////////////////////
                .addDisplacementMarker(() -> {
                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
                })
                .build();


//        //TODO:RIGHT
        if(marker=="RIGHT"|| randompos==2){
            drive.followTrajectorySequence(right1);
            drive.followTrajectorySequence(right2);
            sleep(3000);
//            sleep(100);
            drive.followTrajectorySequence(right3);
            sleep(600);
            schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.SlightmoreDROP));
            sleep(100);
            drive.followTrajectorySequence(right4);
            sleep(900);
            drive.followTrajectorySequence(right5);
            sleep(999999);
        }

//        //TODO:CENTRE(NO WHITE)
        if(marker=="CENTRE"|| randompos==0) {
            drive.followTrajectorySequence(centre1);
            sleep(200);
            schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
            drive.followTrajectorySequence(centre2);
            sleep(3000);
            drive.followTrajectorySequence(centre3);
            sleep(500);
            schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROPseUPAR));
            sleep(300);
            drive.followTrajectorySequence(centre4);
            sleep(500);
            drive.followTrajectorySequence(centre5);
            schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MIDseJYAADA));
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

//        //TODO LEFT (PURPLE FIRST)
        if(marker=="LEFT"|| randompos==1) {
            drive.followTrajectorySequence(left1);
            sleep(100);
            drive.followTrajectorySequence(left2);
            sleep(3000);
            drive.followTrajectorySequence(left3);
            sleep(600);
            schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.BFDROPCENTREAUTO)); //Dropseupar
            sleep(100);
            drive.followTrajectorySequence(left4);
            sleep(900);
            drive.followTrajectorySequence(left5);
            sleep(999999);

        }




    }

    @Override
    public void initialize () {

    }
}


