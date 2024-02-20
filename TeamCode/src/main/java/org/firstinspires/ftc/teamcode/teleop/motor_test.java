package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
@Disabled
@TeleOp
@Config
public class motor_test extends LinearOpMode {
    DcMotorEx motor1;



    @Override
    public void runOpMode() throws InterruptedException {

        motor1 = hardwareMap.get(DcMotorEx.class, "m1");
        waitForStart();
        while (opModeIsActive()) {


            if (gamepad1.x) {
                motor1.setPower(1);

            }
            if (gamepad1.y) {
                motor1.setPower(-1);

            }
            if (gamepad1.a) {
                motor1.setPower(0);

            }
            if (gamepad1.b) {
                motor1.setPower(0.6);

            }
            if (gamepad1.dpad_up) {
//                motor1.setPower(-0.63 );

            }
            if (gamepad1.dpad_down) {

            }
            telemetry.addData("left front", motor1.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}

