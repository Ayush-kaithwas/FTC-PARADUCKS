package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
@Disabled
@TeleOp(name = "servoInc")
@Config
public class servo_inc extends LinearOpMode {

    private Servo rack,gripper,gripRotate;
    private Servo outL,outR,outArm_rotate,outArm_extend,outGripperL,outGripperR;

    public DcMotorEx sliderL, sliderR,extend;
    private double servoInc = 0.01;
    private double servoDec = -0.01;
    public static double rotatepos = 0.73;
    

    @Override
    public void runOpMode() {
        // Initialize your servo here
        rack = hardwareMap.servo.get("rack");
        gripper = hardwareMap.servo.get("gripper");
        gripRotate = hardwareMap.servo.get("gripR");
        outL = hardwareMap.get(Servo.class, "outL"); //port1
        outR = hardwareMap.get(Servo.class,"outR");//port0
        outArm_rotate= hardwareMap.get(Servo.class, "outRotate");//port2
        outArm_extend= hardwareMap.get(Servo.class, "outEx"); //outEx//port5
        outGripperL= hardwareMap.get(Servo.class, "outGripperL");//port4
        outGripperR= hardwareMap.get(Servo.class, "outGripperR");//port3
        sliderL = hardwareMap.get(DcMotorEx.class, "sliderL");
        sliderR = hardwareMap.get(DcMotorEx.class, "sliderR");
        extend = hardwareMap.get(DcMotorEx.class, "extend");

        rack.setPosition(1);
        gripRotate.setPosition(0.5);
        gripper.setPosition(0.5);
        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.start){
//                outR.setPosition(0.5);
//                outL.setPosition(0.5);
//                gripRotate.setPosition(0.);
                double newPosition = gripRotate.getPosition() + 0.01;
                newPosition = Math.min(1.0, Math.max(0.0, newPosition));
//                gripRotate.setPosition(newPosition);
                sleep(200);

            }
            if(gamepad1.back){
                double newPosition = gripRotate.getPosition() -0.01;
                newPosition = Math.min(1.0, Math.max(0.0, newPosition));
                gripRotate.setPosition(newPosition);
                sleep(200);
            }
            if (gamepad1.dpad_up) {
                double newPosition = outArm_extend.getPosition() + servoInc;
                newPosition = Math.min(1.0, Math.max(0.0, newPosition));
                outArm_extend.setPosition(newPosition);
                sleep(200);
            }
            if (gamepad1.dpad_down) {
                double newPosition1 = outArm_extend.getPosition() + servoDec;
                newPosition1 = Math.min(1.0, Math.max(0.0, newPosition1));
                outArm_extend.setPosition(newPosition1);
                sleep(200);
            }
            if(gamepad1.dpad_right){
                double newPosition = outL.getPosition() + 0.05;
                double newPosition1 = outR.getPosition() -0.05;
                newPosition = Math.min(1.0, Math.max(0.0, newPosition));
                newPosition1 = Math.min(1.0, Math.max(0.0, newPosition1));
                outL.setPosition(newPosition);
                outR.setPosition(newPosition1);
                sleep(200);
            }
            if(gamepad1.dpad_left){
                double newPosition = outL.getPosition() -0.05;
                double newPosition1 = outR.getPosition() + 0.05;
                newPosition = Math.min(1.0, Math.max(0.0, newPosition));
                newPosition1 = Math.min(1.0, Math.max(0.0, newPosition1));
                outL.setPosition(newPosition);
                outR.setPosition(newPosition1);
                sleep(200);
            }
            if(gamepad1.x){
//                rack.setPosition(0.5);
//                sleep(1000);
//                rack.setPosition(0.6);
//                sleep(100);
//                rack.setPosition(0.4);
                double newPosition = outArm_rotate.getPosition() + servoInc;
                newPosition = Math.min(1.0, Math.max(0.0, newPosition));
                outArm_rotate.setPosition(newPosition);
                sleep(200);
            }
            if(gamepad1.y){
                double newPosition = outArm_rotate.getPosition() + servoDec;
                newPosition = Math.min(1.0, Math.max(0.0, newPosition));
                outArm_rotate.setPosition(newPosition);
                sleep(200);
            }

            if(gamepad1.a){
                outArm_rotate.setPosition(rotatepos);
                sleep(200);
//                double newPosition = outGripperL.getPosition() + servoInc;
//                newPosition = Math.min(1.0, Math.max(0.0, newPosition));
//                outGripperL.setPosition(newPosition);
//                sleep(200);
            }
            if(gamepad1.b){
                double newPosition = outGripperL.getPosition() + servoDec;
                newPosition = Math.min(1.0, Math.max(0.0, newPosition));
                outGripperL.setPosition(newPosition);
                sleep(200);
            }
            if(gamepad1.left_bumper){
                double newPosition = outGripperR.getPosition() + servoInc;
                newPosition = Math.min(1.0, Math.max(0.0, newPosition));
                outGripperR.setPosition(newPosition);
                sleep(200);
            }
            if(gamepad1.right_bumper){
                double newPosition = outGripperR.getPosition() + servoDec;
                newPosition = Math.min(1.0, Math.max(0.0, newPosition));
                outGripperR.setPosition(newPosition);
                sleep(200);
            }
            if(gamepad2.left_trigger>0.5){
                //DOWN
                double newPosition = rack.getPosition() + 0.01;
                newPosition = Math.min(10.0, Math.max(0.0, newPosition));
                rack.setPosition(newPosition);
                sleep(200);
            }
            if(gamepad2.right_trigger>0.5){
                //UP
                double newPosition = rack.getPosition() -0.01;
                newPosition = Math.min(10.0, Math.max(0.0, newPosition));
                rack.setPosition(newPosition);
                sleep(200);
            }

            if(gamepad2.dpad_up){
//                sliderR_extend();
                    double newPosition = gripRotate.getPosition() + servoInc;
                    newPosition = Math.min(1.0, Math.max(0.0, newPosition));
                    gripRotate.setPosition(newPosition);
                    sleep(200);
                }
                if (gamepad2.dpad_down) {
                    double newPosition1 = gripRotate.getPosition() + servoDec;
                    newPosition1 = Math.min(1.0, Math.max(0.0, newPosition1));
                    gripRotate.setPosition(newPosition1);
                    sleep(200);
                }

//            if(gamepad2.dpad_down){
//                sliderR_retract();
//            }
//            if(gamepad2.dpad_left){
//                sliderL_extend();
//            }
//            if(gamepad2.dpad_right){
//                sliderL_retract();
//            }
//            if (gamepad2.a){
//                sliderR.setTargetPosition(sliderL.getCurrentPosition());
//                sliderL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                sliderR.setTargetPosition(sliderR.getCurrentPosition());
//                sliderL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                sliderR.setPower(0);
//                sliderL.setPower(0);
//            }
//            if(gamepad2.b){
//                extend.setTargetPosition(extend.getCurrentPosition());
//                extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                extend.setPower(0.6);
//            }
            if(gamepad2.start){
//                gripper.setPosition(0.5);
//                outR.setPosition(0.3);
//                outL.setPosition(0.7);
                double newPosition = gripper.getPosition() +0.05;
                newPosition = Math.min(1.0, Math.max(0.0, newPosition));
                gripper.setPosition(newPosition);
                sleep(200);
            }
            if(gamepad2.back){
//                gripper.setPosition(0.5);
//                outR.setPosition(0.7);
//                outL.setPosition(0.3);
                double newPosition = gripper.getPosition() -0.05;
                newPosition = Math.min(1.0, Math.max(0.0, newPosition));
                gripper.setPosition(newPosition);
                sleep(200);
            }

            if(gamepad2.right_bumper){
                gripper.setPosition(0.74); //Open
            }
            if(gamepad2.left_bumper){
                gripper.setPosition(0.35); //Close
            }
            if (gamepad2.a){
                gripper.setPosition(0.74); //Open
                rack.setPosition(0.9);
                gripRotate.setPosition(0.2688888);
            }
            if (gamepad2.b){
                gripper.setPosition(0.74); //Open
                rack.setPosition(0.93);
                gripRotate.setPosition(0.2688888);
            }






            telemetry.addData("outL", outL.getPosition());
            telemetry.addData("outR ", outR.getPosition());
            telemetry.addData("Extend", outArm_extend.getPosition());
            telemetry.addData("outArm_rotate", outArm_rotate.getPosition());
            telemetry.addData("outGripperL ", outGripperL.getPosition());
            telemetry.addData("outGripperR ", outGripperR.getPosition());
            telemetry.addData("rackk ", rack.getPosition());
            telemetry.addData("GripRotate ", gripRotate.getPosition());
            telemetry.addData(" ExtendCurrent", extend.getCurrent(CurrentUnit.AMPS));
            telemetry.addData(" Extend", extend.getCurrentPosition());
            telemetry.addData(" sliderR Current", sliderR.getCurrent(CurrentUnit.AMPS));
            telemetry.addData(" sliderR ", sliderR.getCurrentPosition());
            telemetry.addData(" sliderL Current", sliderL.getCurrent(CurrentUnit.AMPS));
            telemetry.addData(" sliderL ", sliderR.getCurrentPosition());
            telemetry.addData("gripper ", gripper.getPosition());



            telemetry.update();
        }

    }
    public void sliderR_extend(){
//        extend.setTargetPosition(extendMaxPos);
        sliderR.setTargetPosition(sliderR.getCurrentPosition() + 100);
        sliderR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderR.setPower(0.6);
    }
    public void sliderR_retract(){
//        extend.setTargetPosition(extendInitPos);
        sliderR.setTargetPosition(sliderR.getCurrentPosition() - 100);
        sliderR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderR.setPower(0.6);
    }

    public void sliderL_extend(){
//        extend.setTargetPosition(extendInitPos);
        sliderL.setTargetPosition(sliderL.getCurrentPosition() + 100);
        sliderL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderL.setPower(0.6);
    }
    public void sliderL_retract(){
//        extend.setTargetPosition(extendInitPos);
        sliderL.setTargetPosition(sliderL.getCurrentPosition() - 100);
        sliderL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderL.setPower(0.6);
    }
}
//out extend
//0.22 lowest
//1 MAX

//rotate


// outl outR
//pix take
// 0utL =0.8
//outR=0.2
// pix drop
//outL=0.6  //0.5994
//outR=0.4  //0.3994

//Intakke mechanism
//picking pixel
//griprotate
//pick = 0.3
//rack = 0.9


//INTAKE MECHANISM
//DROP PIXEL
//GRIPROTATE =0.8
// Rack = 0.6


////////////////////////////////
////PIXELDROP
//gripper = 0.3499
//rack = 0.0894
//gripRotate = 0.79944
//
//////////
//Pixel drop
//gripper = 0.5