package org.firstinspires.ftc.teamcode.auto.RED;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.gripRotateCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.gripperCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outArmCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outArmExtensionCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outGripperCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.rackCommand;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.HangerAndDrone;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.OBJECT_DETECT;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.intakeSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.outakeSubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.Location;
import org.firstinspires.ftc.teamcode.hardware.PropPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(group = "RED")
public class RED_CLOSE_OpenCV extends CommandOpMode {
    intakeSubsystem intake = null;
    outakeSubsystem outake = null;
    HangerAndDrone endgame = null;
    Servo rack;
    SampleMecanumDrive drive = null;
    OBJECT_DETECT detect = null;

    public static String marker = "RIGHT";
    public static int randompos = 2;
    private PropPipeline propPipeline;
    private VisionPortal portal;
    private Location randomization;


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
        Globals.ALLIANCE = Location.RED;
        Globals.SIDE = Location.CLOSE;
        schedule(new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE));
        schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID)); // for Left only
//        schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK));

        schedule(new rackCommand(intake, intakeSubsystem.RackState.LOWseUPAR));
        schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
        schedule(new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.INIT));
        propPipeline = new PropPipeline(telemetry);
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(propPipeline)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();



        while (detect.getCameraState() != VisionPortal.CameraState.STREAMING && portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addLine("initializing... please wait");
            telemetry.update();
        }

        while (opModeInInit()) {
            telemetry.addLine("ready");

            try{
                marker = propPipeline.getLocation().toString();
            }catch (Exception e){
                marker="RIGHT";
            }
            telemetry.addData("position", propPipeline.getLocation());
            if(gamepad1.dpad_left){
                marker = "LEFT";
            }
            if(gamepad1.dpad_up){
                marker = "CENTER";
            }
            if(gamepad1.dpad_right){
                marker = "RIGHT";
            }

            telemetry.addData("marker",marker);
            telemetry.update();
        }

        portal.close();
        waitForStart();



//        randomization = propPipeline.getLocation();


        if (isStopRequested()) return;
//TODO: LEFT CYCLE AUTO
//TODO:LEFT1
        TrajectorySequence left0 = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    schedule(new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE));
                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK)); // for Left only
//        schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK));

                    schedule(new rackCommand(intake, intakeSubsystem.RackState.LOWseUPAR));
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
                    schedule(new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.INIT));
                })
                .lineToConstantHeading(new Vector2d(7, -69))

                .build();



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

                .lineToConstantHeading(new Vector2d(40,-39))//35.5 for extreme left//.37
                .addDisplacementMarker(() -> {
//                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROP));
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
//                    new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND);


                })
                .build();
        //TODO:LEFT3
        TrajectorySequence left3 = drive.trajectorySequenceBuilder(left2.end())

                .lineToConstantHeading(new Vector2d(41.5,-39))
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
//
               .addDisplacementMarker(() -> {
                    schedule(new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE));
                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK)); // for Left only
                    schedule(new rackCommand(intake, intakeSubsystem.RackState.LOWseUPAR));
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
                    schedule(new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.INIT));
                })
                .lineToConstantHeading(new Vector2d(7, -69))

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
                .lineToConstantHeading(new Vector2d(48.6,-69))
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
//
//                .splineToConstantHeading(new Vector2d(1.5, -38),0)
                .addDisplacementMarker(() -> {
                    schedule(new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE));
                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK)); // for Left only
//        schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK));

                    schedule(new rackCommand(intake, intakeSubsystem.RackState.LOWseUPAR));
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
                    schedule(new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.INIT));
                })
                .lineToConstantHeading(new Vector2d(7, -69))

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
                .lineToConstantHeading(new Vector2d(40,-50.9))//-49.7
                .addDisplacementMarker(() -> {
                    schedule(new outArmExtensionCommand(outake,outakeSubsystem.OutArmExtensionState.INIT));
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROP));
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
//                    schedule(new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND));//Try without extend and take thoda aage
                })
                .build();


//        TODO:RIGHT13
        TrajectorySequence right13 = drive.trajectorySequenceBuilder(startright2Pose)
                .lineToConstantHeading(new Vector2d(42.5,-50.9))//-\
                // 49//-48.3//-47.6
                .addDisplacementMarker(() -> {
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPEN));
                })
                .build();

//        TODO:RIGHT14
        TrajectorySequence right14 = drive.trajectorySequenceBuilder(startright3Pose)
                .lineToConstantHeading(new Vector2d(39,-50.9))
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
        if(marker=="LEFT") {
            drive.followTrajectorySequence(left0);
            sleep(1000);
            drive.followTrajectorySequence(left1);
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
        if(marker=="CENTER") {
            drive.followTrajectorySequence(centre0);
            sleep(1000);
            drive.followTrajectorySequence(centre1);
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
        if(marker=="RIGHT") {
            drive.followTrajectorySequence(right0);
            sleep(1000);
            drive.followTrajectorySequence(right11);
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
    public VisionPortal.CameraState getCameraState() {
        if (portal != null) return portal.getCameraState();
        return null;
    }
}

