package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp
@Config
public class servo_test extends LinearOpMode {

    Servo servo1;
    Servo servo2;

    Servo outArm_extend;
    Servo outArm_rotate;
    Servo servo3;
    private Servo rack,gripper,gripRotate;

    public static double s1pos1 = 0.505; //close
    public static double s1pos2 = 0.482; //open
    public static double s2pos1 = 0.51;//back
    public static double s2pos2 = 0.465; //push

    public static double servoPosition = 0.1;


    @Override
    public void runOpMode() throws InterruptedException {

        servo1 = hardwareMap.get(Servo.class, "s1");
//        servo2 = hardwareMap.get(Servo.class, "s2");
//        outArm_extend= hardwareMap.get(Servo.class, "outEx"); //outEx//port5
//        outArm_rotate= hardwareMap.get(Servo.class, "outRotate");//port2


//        rack = hardwareMap.get(Servo.class, "rack");

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_left) {

                servoPosition += 0.1;
                servoPosition = Math.min(Math.max(servoPosition, 0.0), 1.0);
                servo1.setPosition(servoPosition);
            }
            if (gamepad1.dpad_right) {

                servoPosition -= 0.1;
                servoPosition = Math.min(Math.max(servoPosition, 0.0), 1.0);
                servo1.setPosition(servoPosition);
            }

            if (gamepad1.x) {
                servo1.setPosition(0.4);

            }
            if (gamepad1.y) {
                servo1.setPosition(0.7);
            }
            if (gamepad1.a) {
                servo1.setPosition(0.5);
//                servo2.setPosition(0.5);
            }
//            servo3.setPosition(0.5);
//
//            }
            if (gamepad1.b) {
                servo1.setPosition(0.2);
            }
//            if (gamepad1.dpad_up) {
//                servo1.setPosition(0.9);
//            }
//            if (gamepad1.dpad_down) {
//                servo1.setPosition(1);
//            }
            if (gamepad1.dpad_up) {
                double newPosition = outArm_extend.getPosition() + 0.05;
                newPosition = Math.min(1.0, Math.max(0.0, newPosition));
                outArm_extend.setPosition(newPosition);
                sleep(200);
            }
            if (gamepad1.dpad_down) {
                double newPosition1 = outArm_extend.getPosition() -0.05;
                newPosition1 = Math.min(1.0, Math.max(0.0, newPosition1));
                outArm_extend.setPosition(newPosition1);
                sleep(200);
            }
            if (gamepad1.dpad_up) {
                double newPosition = outArm_extend.getPosition() + 0.05;
                newPosition = Math.min(1.0, Math.max(0.0, newPosition));
                outArm_extend.setPosition(newPosition);
                sleep(200);
            }
            if (gamepad1.dpad_down) {
                double newPosition1 = outArm_extend.getPosition() -0.05;
                newPosition1 = Math.min(1.0, Math.max(0.0, newPosition1));
                outArm_extend.setPosition(newPosition1);
                sleep(200);
            }
            if (gamepad1.left_bumper){

            }
//            telemetry.addDat
        }
    }
}
//}
