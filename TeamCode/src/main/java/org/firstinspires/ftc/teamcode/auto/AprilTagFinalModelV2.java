package org.firstinspires.ftc.teamcode.auto;

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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
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
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
@Autonomous(group = "BLUE")
//@Autonomous(group = "drive")
public class AprilTagFinalModelV2 extends CommandOpMode{
    intakeSubsystem intake = null;
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    VisionPortal visionPortal, portal;

    outakeSubsystem outake = null;
    HangerAndDrone endgame = null;
    public AprilTagDetection desiredTag = null;
    Servo rack;
    SampleMecanumDrive drive = null;
    OBJECT_DETECT detect = null;
    String marker = "RIGHT";
    public static int randompos = 2;
    public PropPipeline propPipeline;

    private Location randomization;
    public AprilTagProcessor aprilTag;

    public double DESIRED_DISTANCE = 12;
    public double XCurrent = 23.5;
    double Ycurrent = 43.5; //23.5, 43.4
    double  Hcurrent = Math.toRadians(0);
    double SPEED_GAIN  =  0.04; // TODO ==== CHANGE THESE VALUES ====  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    double STRAFE_GAIN =  0.015 ;   // TODO ==== CHANGE THESE VALUES ====  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    double TURN_GAIN   =  0.05  ;   // TODO ==== CHANGE THESE VALUES ==== Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    double speed = 0;        // Desired forward power/speed (-1 to +1)
    double strafe = 0;        // Desired strafe power/speed (-1 to +1)
    double turn = 0;        // Desired turning power/speed (-1 to +1)

    double MAX_AUTO_SPEED = 1;   //  Clip the approach speed to this max value (adjust for your robot)
    double MAX_AUTO_STRAFE= 1;   //  Clip the approach speed to this max value (adjust for your robot)
    double MAX_AUTO_TURN  = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {

        rack = hardwareMap.get(Servo.class, "rack");
        intake = new intakeSubsystem(hardwareMap, telemetry);
        outake = new outakeSubsystem(hardwareMap, telemetry);
        endgame = new HangerAndDrone(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
//        ElapsedTime runtime = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (USE_WEBCAM)
            setManualExposure(6, 22);  // TODO ==== CHANGE THESE VALUES ==== Use low exposure time to reduce motion blur

        Pose2d startPose = new Pose2d(7, 70, Math.toRadians(0));
        Pose2d startright1Pose = new Pose2d(43, 35, Math.toRadians(0));
        Pose2d startright2Pose = new Pose2d(46, 35, Math.toRadians(0));
        Pose2d startright3Pose = new Pose2d(46.5, 35, Math.toRadians(0));
        Pose2d startcentre1Pose = new Pose2d(41, 42.9, Math.toRadians(0));
        Pose2d startcentre2Pose = new Pose2d(42.7, 42.9, Math.toRadians(0));
        Pose2d startcentre3Pose = new Pose2d(42, 42.9, Math.toRadians(0));
        Pose2d startleft1Pose = new Pose2d(23.5, 38, Math.toRadians(0));
        Pose2d startleft2Pose = new Pose2d(45, 50, Math.toRadians(0));
        Pose2d startleft3Pose = new Pose2d(46, 50, Math.toRadians(0));


        drive.setPoseEstimate(startPose);
        Globals.ALLIANCE = Location.BLUE;
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

            try {
                marker = propPipeline.getLocation().toString();
            } catch (Exception e) {
                marker = "RIGHT";
            }
//            try{
//                marker = propPipeline.getLocation().toString();
//            }catch (Exception rhs){
//                marker="RIGHT";
//            }
            telemetry.addData("position", propPipeline.getLocation());
            if (gamepad1.dpad_left) {
                marker = "LEFT";
            }
            if (gamepad1.dpad_up) {
                marker = "CENTER";
            }
            if (gamepad1.dpad_right) {
                marker = "RIGHT";
            }

            telemetry.addData("marker", marker);
            telemetry.update();
        }

        portal.close();
        sleep(10);
        initAprilTag();


//TODO: LEFT CYCLE AUTO
        TrajectorySequence left0 = drive.trajectorySequenceBuilder(startPose)
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
                .lineToConstantHeading(new Vector2d(7, 69))

                .build();
//TODO:LEFT1
        TrajectorySequence left1 = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(25.3, 38))
                .addDisplacementMarker(() -> {
                    schedule(new ParallelCommandGroup(
                            new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK), // for Left only
                            new rackCommand(intake, intakeSubsystem.RackState.LOW),
                            new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK),
                            new gripperCommand(intake, intakeSubsystem.GripperState.OPEN)
                    ));
                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK));
                })
                .build();
        //TODO:LEFT2
        TrajectorySequence left2 = drive.trajectorySequenceBuilder(left1.end())
                .lineToConstantHeading(new Vector2d(38, 50))
                .addDisplacementMarker(() -> {
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROP));
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
//                    new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND);
                })
                .build();
        //TODO:LEFT3
        TrajectorySequence left3 = drive.trajectorySequenceBuilder(left2.end())
                .lineToConstantHeading(new Vector2d(42.5, 50))
                .addDisplacementMarker(() -> {
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPEN));
                })
                .build();
        //TODO:LEFT4
        TrajectorySequence left4 = drive.trajectorySequenceBuilder(left3.end())
                .lineToConstantHeading(new Vector2d(37, 50))
                .addDisplacementMarker(() -> {
                    schedule(new ParallelCommandGroup(
                            (new outArmCommand(outake, outakeSubsystem.OutArmState.PICK)),
                            new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID)
                    ));
                })
                ///////////////////////////////////////////////////////////
                //TODO:PARK LEFT
                .lineToConstantHeading(new Vector2d(37, 68))
                .lineToConstantHeading(new Vector2d(48.6, 68))
                ///////////////////////////////////////////////////////////
//                ///////////////////////////////////////////////////////////
//                //TODO:PARK RIGHT
//                .lineToConstantHeading(new Vector2d(40,9))
//                .lineToConstantHeading(new Vector2d(49,9))
//                ///////////////////////////////////////////////////////////
                .build();


        //TODO: CENTRE CYCLE AUTO (done)
        TrajectorySequence centre0 = drive.trajectorySequenceBuilder(startPose)
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
                .lineToConstantHeading(new Vector2d(7, 69))

                .build();
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

                .lineToConstantHeading(new Vector2d(23.5, 43.4))//41.4
                .addDisplacementMarker(() -> {
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROP));
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
//                    new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND);
                })
                .build();
        //TODO:CENTRE3
        TrajectorySequence centre3 = drive.trajectorySequenceBuilder(centre2.end())

                .lineToConstantHeading(new Vector2d(42, 43.4))
                .addDisplacementMarker(() -> {
                    ;
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPEN));
                })
                .build();
        //TODO:CENTRE4
        TrajectorySequence centre4 = drive.trajectorySequenceBuilder(centre3.end())


                .addDisplacementMarker(() -> {
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.PICK));
                    schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
                })
                ///////////////////////////////////////////////////////////
                //TODO:PARK LEFT
                .lineToConstantHeading(new Vector2d(36, 68))
                .lineToConstantHeading(new Vector2d(49.6, 68))
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
                .lineToConstantHeading(new Vector2d(7, 69))

                .build();
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
                .lineToConstantHeading(new Vector2d(23.5, 43.5))//37.5//35
                .addDisplacementMarker(() -> {
//                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
                    schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROP));
                    schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
//                    new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND);
                })
                .build();

//        TODO:RIGHT13
        TrajectorySequence right13 = drive.trajectorySequenceBuilder(right12.end())

                .lineToConstantHeading(new Vector2d(41.5, 38.5))
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
                .lineToConstantHeading(new Vector2d(37, 38.5))

                ///////////////////////////////////////////////////////////
                //TODO:PARK LEFT
                .lineToConstantHeading(new Vector2d(37, 68))
                .lineToConstantHeading(new Vector2d(48.6, 68))
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


        waitForStart();
//        //TODO:LEFT
//

        while (opModeIsActive()) {
            if (marker == "LEFT") {
                drive.followTrajectorySequence(left0);
                sleep(1000);
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
            if (marker == "CENTER") {
                drive.followTrajectorySequence(centre0);
                sleep(1000);
                drive.followTrajectorySequence(centre1);
                drive.followTrajectorySequence(centre2);
                sleep(600);
                //List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                runAprilTag(2);
                break;
//                schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROPseUPAR));
//                sleep(200);
//                drive.followTrajectorySequence(centre3);
//                sleep(800);
//                drive.followTrajectorySequence(centre4);
//                sleep(999999);

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
            if (marker == "RIGHT") {
                drive.followTrajectorySequence(right0);
                sleep(1000);
                drive.followTrajectorySequence(right11);
                drive.followTrajectorySequence(right12);
                sleep(600);
                runAprilTag(2);
                break;
//                schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROPseUPAR));
//                sleep(100);
//                drive.followTrajectorySequence(right13);
//                sleep(800);
//                drive.followTrajectorySequence(right14);
//                sleep(999999);

            }


        }
    }
    @Override
    public void initialize () {

    }


    public void runAprilTag(int DESIRED_TAG_ID){

//        initAprilTag();
        TrajectorySequence ResetTraj = drive.trajectorySequenceBuilder(new Pose2d(XCurrent, Ycurrent, Hcurrent))
                .lineToLinearHeading(new Pose2d(XCurrent+0.1, Ycurrent+0.1, Hcurrent ))
                .build();

        while(true) {

            //// TODO TRYING TO DETECT THE APRIL TAG
            drive.update();
            desiredTag = null;

            try {
                // Step through the list of detected tags and look for a matching tag

                List<AprilTagDetection> currentDetections = aprilTag.getDetections();


                for (AprilTagDetection detection : currentDetections) {
                    telemetry.addLine("Inside for block");
                    telemetry.update();
                    // Look to see if we have size info on this tag.
                    if (detection.metadata != null) {
                        telemetry.addLine("Inside if block");
                        telemetry.update();
                        //  Check to see if we want to track towards this tag.
                        if ((DESIRED_TAG_ID < 0) || detection.id == DESIRED_TAG_ID) {
                            desiredTag = detection;
                            break;  //
                        }
                    } else {
                        // This tag is NOT in the library, so we don't have enough information to track to it.
                        telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                        telemetry.update();
                        drive.followTrajectorySequence(ResetTraj);
                        drive.update();
                    }
                }



                //// TODO GETTING THE ERRORS FROM APRIL TAGS

                double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double headingError = desiredTag.ftcPose.bearing;
                double yawError = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                speed = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);


                moveRobot(speed, strafe, turn);

                sleep(10);
                if (rangeError < 0.1) {
                    moveRobot(0, 0, 0);
                    drive.update();
                    break;
                }

            }
            catch (Exception e)
            {
                //  TODO THESE TRAJECTORIES WILL EXECUTE IF THE CAMERA DIDN'T WORKED
                telemetry.addLine("In Catch Block");
                telemetry.update();
                drive.followTrajectorySequence(ResetTraj);
            }
        }

    }

    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.

        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        drive.leftFront.setPower(leftFrontPower);
        drive.rightFront.setPower(rightFrontPower);
        drive.leftRear.setPower(leftBackPower);
        drive.rightRear.setPower(rightBackPower);
    }

}


