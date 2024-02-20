package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commandbase.subsytem.outakeSubsystem;
@Disabled
public class Outake_test extends LinearOpMode {

    outakeSubsystem outake = null;

    @Override
    public void runOpMode() throws InterruptedException {

        outake = new outakeSubsystem(hardwareMap, telemetry);

        waitForStart();
        while (opModeIsActive()) {

            //TODO CONTROLS
            boolean UP = gamepad1.dpad_up;
            boolean RIGHT = gamepad1.dpad_right;
            boolean DOWN = gamepad1.dpad_down;
            boolean LEFT = gamepad1.dpad_left;
            boolean RB = gamepad1.right_bumper;
            boolean LB = gamepad1.left_bumper;
            boolean LT = gamepad1.left_trigger > 0.5;
            boolean RT = gamepad1.right_trigger > 0.5;

            if (gamepad1.x) {
                outake.sliderR_retract();

            }
            if (gamepad1.b) {
                outake.sliderL_retract();
            }
            if (gamepad1.y) {
                outake.sliderR_extend();
            }
            if (gamepad1.a) {
                outake.sliderL_extend();
            }

            // TODO: Rack Servo
            if(UP){
            }

            if(DOWN){

            }
            if(RB){

            }
            if(LB){
            }
            if(gamepad1.start){
                outake.outgrip();
            }
            if(gamepad1.back){
                outake.outdrop();
            }
            if(LT){

            }
            if(RT){

            }
            if(RIGHT){

            }
            if(LEFT){

            }
        }
    }
}
