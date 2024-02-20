package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandbase.instantCommands.gripperCommand;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.HangerAndDrone;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.intakeSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.outakeSubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
public class auto3 extends CommandOpMode {

    @Override
    public void initialize() {

    }
}


//            robot.periodic();

//            double loop = System.nanoTime();
//            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
//            telemetry.addLine(drive.getPose().toString());
//            telemetry.addData("Runtime: ", endTime == 0 ? timer.seconds() : endTime);
//            loopTime = loop;
//            telemetry.update();

//        }

//

//    public void run {
//            super.run();
//        }
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        while (!isStopRequested()) {
//            TrajectorySequence test = drive.trajectorySequenceBuilder(startPose)
//                    .forward(DISTANCE)
//                    .addDisplacementMarker(() -> {
//                        schedule(new gripperCommand(intake, intakeSubsystem.GripperState.OPEN));
//                    })
//                    .turn(Math.toRadians(-90))
//                    .forward(DISTANCE)
//                    .build();
//
//
//            drive.followTrajectorySequence(test);
//        }
//        CommandScheduler.getInstance().schedule(
//                new SequentialCommandGroup(
////                        new InstantCommand(timer::reset),
//                        // go to yellow pixel scoring pos
//                        new WaitUntilCommand(() -> gamepad1.a),
//                        new PositionCommand(new Pose(37.75, 39.35, Math.PI / 2)),
//                        new WaitUntilCommand(() -> gamepad1.a),
////                                .alongWith(new PurplePixelExtendCommand()),
//
////                        new PurplePixelDepositCommand(),
//
//                        new PositionCommand(new Pose(37.75, 38.36, 0)),
//                        new WaitUntilCommand(() -> gamepad1.a),
//
//    }



