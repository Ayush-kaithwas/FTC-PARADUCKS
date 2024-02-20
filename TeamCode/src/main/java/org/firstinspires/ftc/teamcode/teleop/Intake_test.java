package org.firstinspires.ftc.teamcode.teleop;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.intakeSubsystem;
@Disabled
@Config
@TeleOp
public class Intake_test extends LinearOpMode {

    intakeSubsystem intake = null;

    @Override
    public void runOpMode() throws InterruptedException {
        intake=new intakeSubsystem(hardwareMap,telemetry);

        waitForStart();
        while (opModeIsActive()) {

            //TODO CONTROLS
            boolean UP = gamepad1.dpad_up;
            boolean RIGHT = gamepad1.dpad_right;
            boolean DOWN = gamepad1.dpad_down;
            boolean LEFT = gamepad1.dpad_left;
            boolean RB = gamepad1.right_bumper;
            boolean LB = gamepad1.left_bumper;
            boolean LT = gamepad1.left_trigger>0.5;
            boolean RT = gamepad1.right_trigger>0.5;


            // TODO: Intake Pulley Motor

            if (gamepad1.x) {
//                intake.intakeExtend();

            }
            if (gamepad1.b) {
//                intake.intakeReverse();
            }
            if (gamepad1.y) {
//                intake.intakeExtendOff();
            }
            if (gamepad1.a) {
                intake.rackInit();
            }

            // TODO: Rack Servo
            if(UP){
//                intake.rackServoINC();
                intake.rackServoUP();
            }

            if(DOWN){
//                intake.rackServoDEC();
                intake.rackServoDown();
            }
            if(RB){
//                intake.motor_extension();
            }
            if(LB){
//                intake.motor_retract();
            }
            if(gamepad1.start){
                intake.gripOpen();
            }
            if(gamepad1.back){
                intake.gripClose();
            }
            if(LT){
                intake.topPixelPos();
            }
            if(RT){
                intake.midPixelPos();
            }
            if(RIGHT){
                intake.gripRotate_pixPass();
            }
            if(LEFT){
                intake.gripRotate_pixPick();
            }


            telemetry.addData("Rack_servo", intake.rack.getPosition());
            telemetry.addData("GripRotate_servo", intake.gripRotate.getPosition());
            telemetry.addData("Grip_servo", intake.rack.getPosition());
            telemetry.addData("Extension Motor", intake.extend.getCurrentPosition());
            telemetry.addData("Extension Motor Current", intake.extend.getCurrent(CurrentUnit.AMPS));
            telemetry.update();


        }

    }
}
