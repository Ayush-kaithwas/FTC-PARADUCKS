package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandbase.instantCommands.gripperCommand;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.HangerAndDrone;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.intakeSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.outakeSubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous//(group = "Auto_blueClose")
public class auto101 extends CommandOpMode {
    intakeSubsystem intake = null;
    outakeSubsystem outake = null;
    HangerAndDrone endgame = null;
    Servo rack;
    SampleMecanumDrive drive =null;
    public static double DISTANCE = 5;

    @Override
    public void initialize() {
        rack = hardwareMap.get(Servo.class, "rack");
        intake = new intakeSubsystem(hardwareMap, telemetry);
        outake = new outakeSubsystem(hardwareMap, telemetry);
        endgame = new HangerAndDrone(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        ElapsedTime runtime = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



    }

    public void run() {
            Pose2d startPose = new Pose2d(14.2, 69, Math.toRadians(0));
            drive.setPoseEstimate(startPose);
        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            TrajectorySequence test = drive.trajectorySequenceBuilder(startPose)
                    .forward(DISTANCE)
                    .addDisplacementMarker(() -> {
                        schedule(new gripperCommand(intake, intakeSubsystem.GripperState.OPEN));
                    })
                    .turn(Math.toRadians(-90))
                    .forward(DISTANCE)
                    .build();


            drive.followTrajectorySequence(test);
        }
    }
}



//
//    //  -------------------------------------- LEFT CYCLE ----------------------------------------------
//    TrajectorySequence trajectoryseqLeft = drive.trajectorySequenceBuilder(startPose)
//            .addTemporalMarker(()->{intake.intakeAutoStart();
//                InitFunction();
//            })
//            .lineToConstantHeading(new Vector2d(13.8,41.5))
//            .UNSTABLE_addTemporalMarkerOffset(0.0001, () -> {
//                intake.IntakeMotor.setPower(-0.3);
//                intake.IntakeOnePixel(topPos1+0.072);
//
//                lift.extendTo(50, 0.7);
//                outake.outakeArmPlace();
//            })
//            .waitSeconds(1)
//            .lineToConstantHeading(new Vector2d(45,47)) //backdrop
//            .lineToLinearHeading(new Pose2d(52, 47), SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
//            .resetConstraints()
//            .UNSTABLE_addTemporalMarkerOffset(0.0000001, () -> {
//                intake.IntakeMotor.setPower(0);
//                outake.PixelDropAuto();
//                sleep(200);
//                outake.PixelDropAuto();
//                sleep(200);
//                outake.PixelDropAuto();
//                sleep(200);
//                outake.PixelDropAuto();
//                intake.IntakeMotor.setPower(0);
//
//            })
//            .waitSeconds(2)
//            .UNSTABLE_addTemporalMarkerOffset(0.5, ()->DropMechInactiveWithoutHook(0.7))
//            .setReversed(true)
//// ------------------------------ backdrop to intake -------------------------------------------------------
//
//            .lineToConstantHeading(new Vector2d(36, 20)) //close to start
//            .lineToConstantHeading(new Vector2d(-57, 20))
//            .UNSTABLE_addTemporalMarkerOffset(0.0000001, ()->{
//                intake.IntakeMotor.setPower(-1);
//                intake.IntakeOnePixel(topPos1+0.02);
//            })
//            .waitSeconds(1.5)
//            .setReversed(true)
//            .lineToConstantHeading(new Vector2d(-48,19))
//            .UNSTABLE_addTemporalMarkerOffset(0.000001, ()->{
//                intake.InR.setPosition(0.532);
//                intake.InL.setPosition(0.5);
//            })
//            .waitSeconds(0.3)
//
//            .splineToConstantHeading(new Vector2d(-36, 18),0) //close to start
//            .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{
//                intake.IntakeMotor.setPower(0);
//                intake.intakeInit();
//            })
//            .splineToConstantHeading(new Vector2d(15, 18), 0)
//            .UNSTABLE_addTemporalMarkerOffset(0.000001, () -> {
////                    outake.outakeArm.setPosition(0.5);
////                    sleep(200);
//                lift.extendTo(80, 0.7);
//
//                outake.outakeArmPlace();
//
//            })
//            .lineToConstantHeading(new Vector2d(45,40)) //backdrop
//            .lineToLinearHeading(new Pose2d(52, 40), SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
//            .resetConstraints()
//            .UNSTABLE_addTemporalMarkerOffset(0.0000001, () -> {
//                intake.IntakeMotor.setPower(0);
//                outake.PixelDropAuto();
//                sleep(200);
//                lift.extendToLow();
//                sleep(200);
//                outake.PixelDropAuto();
//                sleep(200);
//                outake.PixelDropAuto();
//                intake.IntakeMotor.setPower(0);
//
//            })
//            .waitSeconds(2)
//            .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{
//                DropMechInactive(0.7);
//
//            })
//            .splineToConstantHeading(new Vector2d(47,47), 0)
//            .waitSeconds(1)
//            .setReversed(true)
//            .build();