package org.firstinspires.ftc.teamcode.teleop;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.HangerAndDrone;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.intakeSubsystem;

@Disabled
@Config
@TeleOp
public class teleop_bot1 extends LinearOpMode {

    SampleMecanumDrive drive = null;
    intakeSubsystem intake=null;

    HangerAndDrone endgame = null;
    Servo hanger_servo,plane;
    public static double Yvalue = 1;
    public static double Xvalue = 1;
    public static double TurnX  = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
//        lift= new Lifter(hardwareMap,telemetry);
//        intake=new Intake(hardwareMap,telemetry);
//        outake=new Outake(hardwareMap,telemetry);
//        f_extend=new Forward_Extender(hardwareMap,telemetry);
//        endgame = new HangerAndDrone(hardwareMap, telemetry);
//        hanger_servo = hardwareMap.get(Servo.class, "hang");
//        plane = hardwareMap.get(Servo.class, "plane");
//          slider = new Slider(hardwareMap,telemetry);
        telemetry= new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        while (opModeInInit()){

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * Yvalue, //Yvalue
                            -gamepad1.left_stick_x * Xvalue, //Xvalue
                            -gamepad1.right_stick_x * TurnX //TurnX
                    )
            );
            drive.update();

                        //TODO CONTROLS
            boolean UP = gamepad1.dpad_up;
            boolean RIGHT = gamepad1.dpad_right;
            boolean DOWN = gamepad1.dpad_down;
            boolean LEFT = gamepad1.dpad_left;
            boolean RB = gamepad1.right_bumper;
            boolean LB = gamepad1.left_bumper;
            boolean LT = gamepad1.left_trigger>0.8;
            boolean RT = gamepad1.right_trigger>0.8;



        }
    }
}
